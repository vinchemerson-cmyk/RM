/**
 * @file rc.cpp
 * @brief UART5 DMA+Idle 接收 DR16 DBUS，发布一致的遥控状态快照。
 */
#include "rc.h"
#include "dbus.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "usart/usart.hpp"

namespace
{
	osThreadId_t             taskHandle   = nullptr;
	usart*                   uartInstance = nullptr;
	rc::DbusStreamParser     parser{};
	uint32_t                 lastReceiveTick{};
	constexpr osThreadAttr_t taskAttrs = {
		.name = "RCTask", .stack_size = 2048U, .priority = osPriorityAboveNormal,
	};

	/** @brief 在中断中重组帧；只在验证成功时更新在线时间。 */
	void OnReceive(usart* instance)
	{
		if (instance == nullptr)
			return;
		const auto&    rx   = instance->Receive();
		const uint32_t size = rx.rx_size <= rx.rx_buffer.size() ? rx.rx_size : rx.rx_buffer.size();
		const uint32_t now  = osKernelGetTickCount();
		if (now - lastReceiveTick > rc::OFFLINE_TIMEOUT_MS)
			parser.Reset();
		lastReceiveTick = now;
		rc::RC_Data_s decoded{};
		if (parser.Push(rx.rx_buffer.data(), size, decoded) != 0U)
		{
			decoded.last_update_tick = now;
			rc::RC                   = decoded;
		}
		// BSP 在此回调返回后统一重启普通模式 DMA，避免重复 Abort/启动。
	}

	/** @brief 保留遥控任务入口；掉线由每个消费者即时查询。 */
	void Task(void*)
	{
		for (;;)
			osDelay(rc::TASK_PERIOD_MS);
	}
}

rc::RC_Data_s rc::RC{};

rc::RC_Data_s rc::Snapshot()
{
	taskENTER_CRITICAL();
	const RC_Data_s snapshot = RC;
	taskEXIT_CRITICAL();
	return snapshot;
}

float rc::Normalize(const uint16_t raw)
{
	const int32_t delta = static_cast<int32_t>(raw) - CHANNEL_CENTER;
	if (delta >= -static_cast<int32_t>(CHANNEL_DEADBAND) &&
		delta <= static_cast<int32_t>(CHANNEL_DEADBAND))
		return 0.0F;
	if (raw <= CHANNEL_MIN)
		return NORMALIZED_MIN;
	if (raw >= CHANNEL_MAX)
		return NORMALIZED_MAX;
	return static_cast<float>(delta) / static_cast<float>(CHANNEL_MAX - CHANNEL_CENTER);
}

float rc::RightHorizontal() { return Normalize(Snapshot().channels[0]); }
float rc::RightVertical() { return Normalize(Snapshot().channels[1]); }
float rc::LeftHorizontal() { return Normalize(Snapshot().channels[2]); }
float rc::LeftVertical() { return Normalize(Snapshot().channels[3]); }

bool rc::IsOnline()
{
	const RC_Data_s state = Snapshot();
	return state.received && osKernelGetTickCount() - state.last_update_tick <= OFFLINE_TIMEOUT_MS;
}

void rc::CreateRCTask()
{
	if (taskHandle == nullptr)
		taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
	if (taskHandle == nullptr)
		Error_Handler();
}

void rc::Init()
{
	if (uartInstance == nullptr)
	{
		constexpr usartConfig::Config config = {
			.usart_handle        = &huart5, .tx_mode = usartConfig::NO_TRANSFER,
			.rx_data_len         = usartConfig::RX_BUF_SIZE,
			.usart_callback_func = OnReceive, .module_instance = &uartInstance,
		};
		uartInstance = usart::Register(config);
	}
	CreateRCTask();
}
