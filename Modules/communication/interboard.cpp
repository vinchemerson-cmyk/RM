/**
 * @file interboard.cpp
 * @brief 通过专用 FDCAN 链路转发消息中心主题并监测双板连接。
 *
 * 线路协议 (经典 CAN 标准帧; 云台->底盘 0x301, 底盘->云台 0x302):
 *   [0]     话题 ID (1..255); 0 保留为心跳帧 (无负载)。
 *   [1]     高 4 位 = 消息序号 (0..15 循环), 低 4 位 = 分片编号。
 *   [2..7]  负载分片, 每片最多 kChunkSize=6 字节。
 *   一个话题固定长度最多 kMaxPayload=48 字节 (8 片); 接收端按
 *   "序号一致 + 分片连续 + 50ms 内收齐"重组, 任一条件不满足即丢弃
 *   本轮分片。链路不提供应用层确认/重发, 依赖消息中心的
 *   "最新值邮箱"语义容忍偶发丢帧。
 *
 * 链路状态: 收到任意合法帧即置在线并发布 "interboard/link" 话题;
 * kOfflineMs 内无帧判定离线, 心跳帧单独以 kHeartbeatMs 周期发送。
 * 应用读取 LinkStatus 后须自行实现执行器的失联保护。
 *
 * 总线占用: 该 FDCAN 控制器由本模块独占, 不经过 CAN_ 模板注册表;
 * 通过 CAN_SetDedicatedRxHandler() 直接接管 FIFO0 接收中断, 在中断里
 * 取空硬件 FIFO 后经 osMessageQueue 交给 InterboardTask 处理。
 * 话题注册必须在调度器启动前完成 (Start() 之后), 两板同 ID 的
 * 消息长度必须一致: 一端 RegisterTxTopic, 另一端 RegisterRxTopic。
 */

#include "interboard.h"

#include <array>
#include <cstring>

#include "cmsis_os2.h"
#include "fdcan.h"
#include "RobotConfig.hpp"
#include "can/can.h"
#include "message_center/message_center.h"

namespace interboard
{
	namespace
	{
		constexpr uint32_t kGimbalToChassisId = 0x301; //云台到底盘的CANID
		constexpr uint32_t kChassisToGimbalId = 0x302; //底盘到云台的CANID
		constexpr uint32_t kHeartbeatMs       = 100;   //双板心跳间隔
		constexpr uint32_t kOfflineMs         = 300;   //双板心跳掉线判定间隔
		constexpr uint32_t kAssemblyTimeoutMs = 50;    //CAN消息分片接收允许间隔
		constexpr uint32_t kPollMs            = 2;     //任务检查收发队列的间隔
		constexpr uint8_t  kChunkSize         = 6;     //每个 CAN 分片最多携带的消息内容的字节
		constexpr size_t   kTopicCapacity     = 8;     //最大主题容量

	#if INFANTRY_INTERBOARD_FDCAN == 1
		FDCAN_HandleTypeDef* const canHandle = &hfdcan1;
	#elif INFANTRY_INTERBOARD_FDCAN == 2
		FDCAN_HandleTypeDef* const canHandle = &hfdcan2;
	#elif INFANTRY_INTERBOARD_FDCAN == 3
		FDCAN_HandleTypeDef* const canHandle = &hfdcan3;
	#else
	#error "INFANTRY_INTERBOARD_FDCAN must be 1, 2 or 3"
	#endif

	#if INFANTRY_BOARD_GIMBAL
		constexpr uint32_t kTxId = kGimbalToChassisId; //云台到底盘的CANID
		constexpr uint32_t kRxId = kChassisToGimbalId; //底盘到云台的CANID
	#else
		constexpr uint32_t kTxId = kChassisToGimbalId; //底盘到云台的CANID
		constexpr uint32_t kRxId = kGimbalToChassisId; //云台到底盘的CANID
	#endif

		/** @brief 中断送入任务队列的经典 CAN 帧副本。 */
		struct RxFrame
		{
			uint8_t length;
			uint8_t data[8];
		};

		//todo:评估后期是否需要根据消息长度切换到canfd模式
		/** @brief 单个双板主题的注册信息和接收分片状态。 */
		struct Topic
		{
			uint8_t                          id{};//canid
			uint8_t                          length{};//消息长度
			uint8_t                          tx_sequence{};//发送队列
			uint8_t                          rx_sequence{};//接收队列
			uint8_t                          next_fragment{};//下一个期望接收的分片索引
			uint32_t                         fragment_tick{};
			bool                             assembling{};//组装中标志位
			Publisher_t*                     publisher{};//发布
			Subscriber_t*                    subscriber{};//订阅
			std::array<uint8_t, kMaxPayload> assembly{};
		};

		std::array<Topic, kTopicCapacity> txTopics{};
		std::array<Topic, kTopicCapacity> rxTopics{};
		size_t                            txCount         = 0;
		size_t                            rxCount         = 0;
		osMessageQueueId_t                rxQueue         = nullptr;
		osThreadId_t                      taskHandle      = nullptr;
		Publisher_t*                      statusPublisher = nullptr;
		LinkStatus                        status{};
		char                              statusTopicName[] = "interboard/link";

		constexpr osThreadAttr_t taskAttrs = {
			.name       = "InterboardTask",
			.stack_size = 2048U,
			.priority   = osPriorityAboveNormal,
		};

		/** @brief 将 HAL DLC 值换算为经典 CAN 有效字节数。 */
		uint8_t DlcLength(uint32_t dlc)
		{
			return dlc <= FDCAN_DLC_BYTES_8 ? static_cast<uint8_t>(dlc) : 0;
		}

		/**
         * @brief 发送一帧：首字节为主题 ID，第二字节保存序号和分片编号。
         * @return true 表示帧已放入 FDCAN 发送 FIFO。
         */
		bool Send(uint8_t id, uint8_t sequence, uint8_t fragment, const uint8_t* bytes, uint8_t count)
		{
			if (HAL_FDCAN_GetTxFifoFreeLevel(canHandle) == 0)
				return false;

			FDCAN_TxHeaderTypeDef header{};
			header.Identifier          = kTxId;
			header.IdType              = FDCAN_STANDARD_ID;
			header.TxFrameType         = FDCAN_DATA_FRAME;
			header.DataLength          = count + 2U;
			header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
			header.BitRateSwitch       = FDCAN_BRS_OFF;
			header.FDFormat            = FDCAN_CLASSIC_CAN;
			header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;

			uint8_t frame[8]{};
			frame[0] = id;
			frame[1] = static_cast<uint8_t>((sequence << 4U) | fragment);
			if (count != 0)
				std::memcpy(frame + 2, bytes, count);
			if (HAL_FDCAN_AddMessageToTxFifoQ(canHandle, &header, frame) != HAL_OK)
				return false;
			++status.tx_frames;
			return true;
		}

		/** @brief 在 FDCAN 回调中取出接收帧并送往任务队列。 */
		void OnCanRx(FDCAN_HandleTypeDef* handle, uint32_t fifo)
		{
			// 此 FDCAN 控制器由双板链路独占；在中断里取空 FIFO，后续处理交给任务。
			while (HAL_FDCAN_GetRxFifoFillLevel(handle, fifo) != 0U)
			{
				FDCAN_RxHeaderTypeDef header{};
				RxFrame               frame{};
				if (HAL_FDCAN_GetRxMessage(handle, fifo, &header, frame.data) != HAL_OK)
					break;
				if (header.Identifier != kRxId || header.IdType != FDCAN_STANDARD_ID ||
					header.RxFrameType != FDCAN_DATA_FRAME || header.FDFormat != FDCAN_CLASSIC_CAN)
					continue;
				frame.length = DlcLength(header.DataLength);
				if (frame.length < 2 || frame.length > 8 || rxQueue == nullptr)
					continue;
				(void)osMessageQueuePut(rxQueue, &frame, 0, 0);
			}
		}

		/** @brief 更新链路状态，并按主题、序号和分片顺序重组有效载荷。 */
		void Process(const RxFrame& frame)
		{
			++status.rx_frames;
			status.last_rx_tick   = osKernelGetTickCount();
			const bool wasOffline = status.online == 0;
			status.online         = 1;
			if (wasOffline || frame.data[0] == 0)
				PubPushMessage(statusPublisher, &status);
			if (frame.data[0] == 0)
				return;

			for (size_t i = 0; i < rxCount; ++i)
			{
				Topic& topic = rxTopics[i];
				if (topic.id != frame.data[0])
					continue;

				const uint8_t sequence      = frame.data[1] >> 4U;
				const uint8_t fragment      = frame.data[1] & 0x0FU;
				const uint8_t fragmentCount = static_cast<uint8_t>((topic.length + kChunkSize - 1U) / kChunkSize);
				if (fragment >= fragmentCount)
					return;
				// 首片开启新一轮重组；后续分片必须按序且在超时前到达。
				if (fragment == 0)
				{
					topic.assembling    = true;
					topic.rx_sequence   = sequence;
					topic.next_fragment = 0;
				}
				if (!topic.assembling || topic.rx_sequence != sequence ||
					fragment != topic.next_fragment ||
					(fragment != 0 && osKernelGetTickCount() - topic.fragment_tick > kAssemblyTimeoutMs))
					return;

				const uint8_t offset = static_cast<uint8_t>(fragment * kChunkSize);
				const uint8_t count  = static_cast<uint8_t>(topic.length - offset < kChunkSize
																? topic.length - offset
																: kChunkSize);
				if (frame.length != count + 2U)
				{
					topic.assembling = false;
					return;
				}
				std::memcpy(topic.assembly.data() + offset, frame.data + 2, count);
				topic.fragment_tick = osKernelGetTickCount();
				++topic.next_fragment;
				if (topic.next_fragment == fragmentCount)
				{
					topic.assembling = false;
					PubPushMessage(topic.publisher, topic.assembly.data());
				}
				return;
			}
		}

		/** @brief 周期处理接收帧、心跳、离线判定和待发送主题。 */
		void Task(void*)
		{
			uint32_t heartbeatTick = osKernelGetTickCount() - kHeartbeatMs;
			for (;;)
			{
				RxFrame frame{};
				while (osMessageQueueGet(rxQueue, &frame, nullptr, 0) == osOK)
					Process(frame);

				const uint32_t now = osKernelGetTickCount();
				if (now - heartbeatTick >= kHeartbeatMs && Send(0, 0, 0, nullptr, 0))
					heartbeatTick = now;
				if (status.online && now - status.last_rx_tick >= kOfflineMs)
				{
					status.online = 0;
					PubPushMessage(statusPublisher, &status);
				}

				for (size_t i = 0; i < txCount; ++i)
				{
					Topic&        topic        = txTopics[i];
					const uint8_t framesNeeded = static_cast<uint8_t>((topic.length + kChunkSize - 1U) / kChunkSize);
					if (HAL_FDCAN_GetTxFifoFreeLevel(canHandle) < framesNeeded)
						continue;
					// 从板内消息中心取一次数据，再拆成最多 6 字节一片发送。
					std::array<uint8_t, kMaxPayload> value{};
					if (!SubGetMessage(topic.subscriber, value.data()))
						continue;
					const uint8_t sequence = topic.tx_sequence++ & 0x0FU;
					for (uint8_t offset = 0, fragment = 0; offset < topic.length;
						 offset         = static_cast<uint8_t>(offset + kChunkSize), ++fragment)
					{
						const uint8_t count = static_cast<uint8_t>(topic.length - offset < kChunkSize
																	   ? topic.length - offset
																	   : kChunkSize);
						if (!Send(topic.id, sequence, fragment, value.data() + offset, count))
							break; // 接收端会丢弃未收齐的这一轮分片。
					}
				}
				osDelay(kPollMs);
			}
		}

		/**
		 * @brief 在任务运行前绑定主题 ID 与板内消息中心的发布者或订阅者。
		 * @param transmit 是否为"发送"
		 */
		bool RegisterTopic(std::array<Topic, kTopicCapacity>& topics, size_t& used,
						   uint8_t                            id, char*       name, uint8_t length, bool transmit)
		{
			if (taskHandle == nullptr || osKernelGetState() == osKernelRunning ||
				id == 0 || name == nullptr || length == 0 || length > kMaxPayload ||
				used == topics.size())
				return false;
			for (size_t i = 0; i < used; ++i)
				if (topics[i].id == id)
					return false;

			Topic& topic = topics[used];
			topic.id     = id;
			topic.length = length;
			if (transmit)
				topic.subscriber = SubRegister(name, length);
			else
				topic.publisher = PubRegister(name, length);
			if ((transmit && topic.subscriber == nullptr) || (!transmit && topic.publisher == nullptr))
				return false;
			++used;
			return true;
		}
	} // namespace

	bool Start()
	{
		if (taskHandle != nullptr)
			return true;
		if (canHandle->Init.RxFifo0ElmtSize != FDCAN_DATA_BYTES_8 ||
			canHandle->Init.TxElmtSize != FDCAN_DATA_BYTES_8)
			return false;

		rxQueue = osMessageQueueNew(16, sizeof(RxFrame), nullptr);
		if (rxQueue == nullptr)
			return false;
		statusPublisher = PubRegister(statusTopicName, sizeof(LinkStatus));
		if (statusPublisher == nullptr)
			return false;

		FDCAN_FilterTypeDef filter{};
		filter.IdType       = FDCAN_STANDARD_ID;
		filter.FilterIndex  = 0;
		filter.FilterType   = FDCAN_FILTER_DUAL;
		filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
		filter.FilterID1    = kRxId;
		filter.FilterID2    = kRxId;
		if (HAL_FDCAN_ConfigFilter(canHandle, &filter) != HAL_OK ||
			HAL_FDCAN_ConfigGlobalFilter(canHandle, FDCAN_REJECT, FDCAN_REJECT,
										 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
			return false;
		CAN_SetDedicatedRxHandler(canHandle, OnCanRx);
		if (HAL_FDCAN_ActivateNotification(canHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK ||
			HAL_FDCAN_Start(canHandle) != HAL_OK)
		{
			CAN_SetDedicatedRxHandler(nullptr, nullptr);
			return false;
		}
		taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
		if (taskHandle == nullptr)
		{
			(void)HAL_FDCAN_Stop(canHandle);
			CAN_SetDedicatedRxHandler(nullptr, nullptr);
			return false;
		}
		return true;
	}

	bool RegisterTxTopic(uint8_t id, char* name, uint8_t length)
	{
		return RegisterTopic(txTopics, txCount, id, name, length, true);
	}

	bool RegisterRxTopic(uint8_t id, char* name, uint8_t length)
	{
		return RegisterTopic(rxTopics, rxCount, id, name, length, false);
	}
} // namespace interboard
