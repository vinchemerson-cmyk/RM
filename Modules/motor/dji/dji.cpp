/**
 * @file dji.cpp
 * @brief DJI M3508 与 M2006 的反馈解析、闭环计算和指令打包。
 *
 * 职责划分:
 *   - 反馈路径: FDCAN 中断 -> CAN_ 模板按 FilterIndex 投递 ->
 *     MotorFeedbackCallback() 解析 8 字节反馈帧, 维护单圈角度、
 *     转速、连续角度 total_angle 与掉线时间戳。回调只做解析与标量
 *     赋值, 绝不阻塞。
 *   - 控制路径: 上层任务每个周期调 DJI_Motor_Control(), 按物理总线和指令帧分组
 *     计算各电机 16 位输出并打包进对应电流指令帧 (C610/C620: 0x200/0x1FF)。
 *     同帧最多 4 台电机, 输出槽位由反馈 ID 决定。
 *
 * 连续角度与重锚 (angle_epoch):
 *   相邻两次反馈间用 13 位编码器增量解卷绕累计 total_angle; 仅首帧
 *   初始化单圈基准并递增 angle_epoch。测试阶段移除反馈超时重锚和停机；
 *   中断期间若电机转过半圈以上，恢复后的累计角度可能不准确。
 *
 * 实例管理: 电机实例来自 16 槽静态池 (无动态分配), CAN 回调经
 * can_config.module_handle 直接回指电机实例, 无需额外映射表。
 */

//
// Created by 19923 on 2026/9/26.
//

#include "dji.h"

#include <cstring>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "motor/angle_target.h"


namespace
{
	/** @brief C620 和 C610 的反馈帧 DLC 均为 8 字节。 */
	constexpr uint8_t kFeedbackLength = 8U;

	/** @brief M3508 与 M2006 (C620/C610 电调) 的电流指令范围. */
	constexpr float kC620CurrentLimit = 16384.0F;
	constexpr float kC610CurrentLimit = 10000.0F;

	/** @brief 每个型号的指令帧 ID 起点、反馈 ID 起点和输出上限. */
	struct MotorTypeSpec
	{
		DJI::Motor_Type_e type;
		float             output_limit; // 16 位输出的绝对值上限.
	};

	/**大疆电机命令地址*/
	struct DjiCommandAddress
	{
		uint32_t frame_id{}; //指令帧
		uint8_t  slot{};     //CAN指令帧槽位
	};

	/**
	* @brief C620/C610 电调的指令帧与反馈 ID 分配.
	* @note 当前只支持 M3508 与 M2006: 二者共用 0x200 (ID1-4) / 0x1FF (ID5-8) 电流帧,
	*       反馈 ID 为 0x201..0x208。GM6020 已移除, 云台改用达妙 DM 电机 (MIT 协议)。
	*/
	constexpr MotorTypeSpec kMotorSpecs[] = {
		{
			.type = DJI::M3508, .output_limit = kC620CurrentLimit
		},

		{
			.type = DJI::M2006, .output_limit = kC610CurrentLimit
		},
	};

	/**
	 * @brief 由反馈 ID 反推该电机的指令帧 ID 与槽位.
	 * @param feedback_id 电机反馈帧的 CAN ID, 取值 0x201..0x208.
	 * @param address 输出: 指令帧 ID 与该电机在帧内的槽位.
	 * @return 反馈 ID 超出 0x201..0x208 时返回 false.
	 * @note ID1-4 走 0x200, ID5-8 走 0x1FF; 槽位为 (id - 0x201) % 4.
	 */
	bool DjiResolveCommand(uint32_t feedback_id, DjiCommandAddress& address)
	{
		constexpr uint32_t     first = 0x201U;
		constexpr uint32_t		count = 8U;
		if (feedback_id < first || feedback_id >= first + count)
			return false;
		const uint32_t offset = feedback_id - first;
		address.frame_id      = (offset < 4U) ? 0x200U : 0x1FFU;
		address.slot          = static_cast<uint8_t>(offset % 4U);
		return true;
	}

	/**
	* @brief 取出型号对应的帧与反馈分配.
	* @return 找到时返回对应表项; 型号非法时返回 nullptr.
	* @note 1. 必须用引用遍历
	*/

	// region 函数
	// for (const MotorTypeSpec& spec : kMotorSpecs)  // ✅ 正确
	// 如果写成：
	//
	// for (const MotorTypeSpec spec : kMotorSpecs)   // ❌ 危险
	// 	return &spec;
	// spec 是循环内的局部拷贝。
	//
	// &spec 是拷贝的地址，循环迭代结束或函数返回后立即悬空。
	//
	// 返回悬空指针，使用会崩溃或读到垃圾。
	//
	// 这是本函数最容易犯的错，引用是必须的。
	// endregion

	const MotorTypeSpec* FindSpec(const DJI::Motor_Type_e motor_type)
	{
		for (const MotorTypeSpec& spec : kMotorSpecs)
		{
			if (spec.type == motor_type)
				return &spec;
		}
		return nullptr;
	}

	/**
	 * @brief 把 16 位输出限制在型号允许范围内.
	* @note 超出范围按边界截断, 不调用 Error_Handler, 避免控制量异常导致整机停机.
	*/
	int16_t ClampOutput(const float output, const float limit)
	{
		if (output > limit)
			return static_cast<int16_t>(limit);
		if (output < -limit)
			return static_cast<int16_t>(-limit);
		return static_cast<int16_t>(output);
	}

	/**
	* @brief 接收回调: 把反馈帧解析进引用该过滤器的电机实例.
	* @param attribute_instance BSP 注册表内的本实例属性.
	* @note 注册属性通过 module_handle 指向电机实例。
	* @note 本函数在 FDCAN 中断内执行, 只做解析和标量赋值, 不做任何阻塞操作.
	 */
	void MotorFeedbackCallback(classicCanType* attribute_instance)
	{
		if (attribute_instance == nullptr || attribute_instance->module_handle == nullptr)
			return;
		auto* motor = static_cast<DJI_Motor_s*>(attribute_instance->module_handle);
		// 首次注册启动 CAN 后可能立即进中断，此时返回句柄尚未写回 motor。
		if (motor->can_instance == nullptr) //空指针检查
			return;
		auto& instance = *motor->can_instance; //解引用成引用

		const uint8_t* data = instance.output.rx_buffer.data(); //取接收缓冲区首地址

		//丢弃DLC不等于8的反馈帧
		if (instance.output.rx_len != kFeedbackLength)
			return;

		//把两个字节按大端序（Big-Endian）拼成一个 16 位有符号整数
		const auto read_be16 = [](const uint8_t* bytes) -> int16_t
		{
			return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8U) | bytes[1]);
		};

		const int16_t raw_angle   = read_be16(data);
		const int16_t raw_speed   = read_be16(data + 2);
		const int16_t raw_current = read_be16(data + 4);

		const bool reverse = motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE;

		// 单圈机械角: 取 13 位计数值换算到 [0, 360) 度; 完整 16 位原值保留在 raw 字段.
		const float angle_deg = static_cast<float>(static_cast<uint16_t>(raw_angle) & 0x1FFFU) / 8192.0F * 360.0F;
		// RPM 换算为度每秒.
		const float speed_dps = static_cast<float>(raw_speed) * 6.0F;
		// C620/C610 的反馈电流均为 1 LSB = 20A/32768.
		const float current_a = static_cast<float>(raw_current) * (20.0F / 32768.0F);

		const uint32_t    tick     = osKernelGetTickCount();
		Motor_Feedback_s& feedback = motor->feedback;
		//收到反馈,长度校验通过
		if (!feedback.angle_initialized)
		{
			// 首帧建立角度基准；反馈中断后不自动停机或重锚。
			feedback.total_angle       = reverse ? -angle_deg : angle_deg;
			feedback.angle_initialized = true;
			++feedback.angle_epoch;
		} else
		{
			//正常:按最短增量累加
			const int16_t delta = MotorEncoderDelta(static_cast<uint16_t>(feedback.raw_angle),
													static_cast<uint16_t>(raw_angle));
			const float delta_deg = static_cast<float>(delta) * (360.0F / 8192.0F);
			feedback.total_angle  += reverse ? -delta_deg : delta_deg;
		}
		feedback.raw_angle   = raw_angle;
		feedback.raw_speed   = raw_speed;
		feedback.raw_current = raw_current;
		feedback.angle       = reverse ? -angle_deg : angle_deg;
		feedback.speed       = reverse ? -speed_dps : speed_dps;
		feedback.current     = reverse ? -current_a : current_a;
		// 当前反馈结构以 0 表示“未提供温度”，不能把它当作实测 0°C。
		feedback.temperature = motor->motor_type == DJI::M2006
								   ? 0.0F
								   : static_cast<float>(data[6]);
		// DATA[7] 不参与解析；C620 的温度从 DATA[6] 读取, M2006 无温度反馈。
		feedback.last_update_tick = tick;

		instance.feedback_tick = tick;
	}

	/**
	 * @brief 按闭环设置计算单个电机的 16 位输出.
	 * @return 已限幅并应用方向标志的输出值.
	 * @note 闭环组合非法时返回 0；测试模式使用最近一次反馈，不判定掉线。
	 */
	float ComputeOutput(DJI_Motor_s& motor, const Motor_Feedback_s& feedback, const float dt_s)
	{
		const DJ_Motor_Control_Setting_s& setting    = motor.motor_settings;
		DJ_Motor_Controller_s&            controller = motor.controller;

		//速度反馈
		const float speed_feedback = setting.speed_feedback_source == OTHER_FEED
										 ? *controller.other_speed_feedback_ptr
										 : feedback.speed;
		//角度反馈
		const float angle_feedback = setting.angle_feedback_source == OTHER_FEED
										 ? *controller.other_angle_feedback_ptr
										 : feedback.total_angle;
		const float speed_feedforward =
			setting.feedforward_flag & SPEED_FEEDFORWARD ? *controller.speed_feedforward_ptr : 0.0F;
		const float current_feedforward =
			setting.feedforward_flag & CURRENT_FEEDFORWARD ? *controller.current_feedforward_ptr : 0.0F;

		float output = 0.0F;
		switch (setting.close_loop_type)
		{
			case OPEN_LOOP:
			case CURRENT_LOOP:
				// 开环与电流环都把期望值直接当作输出, 开环常用于标定和方向确认.
				output = setting.current;
				break;

			case SPEED_AND_CURRENT_LOOP:
				{
					controller.pid_ref = setting.speed;
					output             = controller.speed_PID.Update(setting.speed + speed_feedforward,
																	 speed_feedback, dt_s) +
										 current_feedforward + setting.current;
					break;
				}

			case ANGLE_AND_SPEED_LOOP:
			case ALL_THERE_LOOP:
				{
					controller.pid_ref    = setting.angle;
					const float speed_ref =
						controller.angle_PID.Update(setting.angle, angle_feedback, dt_s);
					output = controller.speed_PID.Update(speed_ref + speed_feedforward,
														 speed_feedback, dt_s) +
							 current_feedforward + setting.current;
					break;
				}

			case SPEED_LOOP:
			case ANGLE_LOOP:
			default:
				// 仅角度环无法直接映射到电流指令, 视为配置错误, 输出保持为零.
				output = 0.0F;
				break;
		}

		if (setting.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
			output = -output;
		return output;
	}
} // namespace

DJI_Motor_s* DJI_Motor_Init(const DJI::Motor_Init_Config_s& config_instance)
{
	const MotorTypeSpec* spec = FindSpec(config_instance.motor_type);
	if (spec == nullptr || config_instance.can_init_config.can_handle == nullptr)
		return nullptr;

	DjiCommandAddress address{};
	if (!DjiResolveCommand(config_instance.can_init_config.can_id, address) ||
		config_instance.can_init_config.can_handle->Init.FrameFormat != FDCAN_FRAME_CLASSIC)
		return nullptr;

	// 电机实例必须是静态存储, 因为 CAN 回调会长期持有其地址.
	static DJI_Motor_s motor_pool[16];
	static uint8_t     motor_pool_used = 0;
	if (motor_pool_used >= 16U)
		return nullptr;

	for (uint8_t i = 0; i < motor_pool_used; ++i)
		if (motor_pool[i].can_instance->dependency.hcan == config_instance.can_init_config.can_handle &&
			motor_pool[i].can_instance->attribute.can_id == config_instance.can_init_config.can_id)
			return nullptr;

	DJI_Motor_s& motor = motor_pool[motor_pool_used];

	motor.motor_type     = config_instance.motor_type;
	motor.motor_settings = config_instance.controller_setting_init_config;

	// 未使用的环允许全零参数；当前闭环所需的 PID 必须已成功配置。
	if (!DJ_Motor_Controller_InitPids(motor.controller, config_instance.controller_param_init_config))
		return nullptr;

	switch (motor.motor_settings.close_loop_type)
	{
		case OPEN_LOOP:
		case CURRENT_LOOP:
			break;
		case SPEED_AND_CURRENT_LOOP:
			if (!motor.controller.speed_PID.IsConfigured())
				return nullptr;
			break;
		case ANGLE_AND_SPEED_LOOP:
		case ALL_THERE_LOOP:
			if (!motor.controller.angle_PID.IsConfigured() ||
				!motor.controller.speed_PID.IsConfigured())
				return nullptr;
			break;
		default:
			return nullptr;
	}

	// 指针型前馈与外部反馈必须有效, 否则控制计算会解引用空指针.
	motor.controller.other_angle_feedback_ptr = config_instance.controller_param_init_config.other_angle_feedback_ptr;
	motor.controller.other_speed_feedback_ptr = config_instance.controller_param_init_config.other_speed_feedback_ptr;
	motor.controller.speed_feedforward_ptr    = config_instance.controller_param_init_config.speed_feedforward_ptr;
	motor.controller.current_feedforward_ptr  = config_instance.controller_param_init_config.current_feedforward_ptr;

	if (motor.motor_settings.speed_feedback_source == OTHER_FEED &&
		motor.controller.other_speed_feedback_ptr == nullptr)
		return nullptr;
	if (motor.motor_settings.angle_feedback_source == OTHER_FEED &&
		motor.controller.other_angle_feedback_ptr == nullptr)
		return nullptr;
	if ((motor.motor_settings.feedforward_flag & SPEED_FEEDFORWARD) &&
		motor.controller.speed_feedforward_ptr == nullptr)
		return nullptr;
	if ((motor.motor_settings.feedforward_flag & CURRENT_FEEDFORWARD) &&
		motor.controller.current_feedforward_ptr == nullptr)
		return nullptr;

	// 反馈 ID 决定 CAN 过滤器, 因此由上层在 can_init_config.rx_id 中给出.
	// tx_id 仅作为兜底初值, 实际发送时由 DJI_Motor_Control() 按指令帧覆盖.
	classicCanConfigType can_config = config_instance.can_init_config;
	can_config.master_id                = address.frame_id;
	can_config.can_callback_func    = MotorFeedbackCallback;
	// module_handle 回指本电机, 使回调无需额外全局映射即可找到实例.
	can_config.module_handle        = &motor;

	motor.can_instance = CAN_<canConfig::classicCan>::Register(can_config);
	if (motor.can_instance == nullptr)
		return nullptr;

	++motor_pool_used;
	return &motor;
}

void DJI_Motor_Enable(DJI_Motor_s* motor)
{
	if (motor == nullptr)
		return;
	motor->controller.pid_ref = 0.0F;
	motor->controller.current_PID.Reset();
	motor->controller.speed_PID.Reset();
	motor->controller.angle_PID.Reset();
	motor->enabled = true;
}

void DJI_Motor_Disable(DJI_Motor_s* motor)
{
	if (motor == nullptr)
		return;
	motor->motor_settings.current = 0.0F;
	motor->motor_settings.speed   = 0.0F;
	motor->motor_settings.angle   = 0.0F;
	motor->controller.current_PID.Reset();
	motor->controller.speed_PID.Reset();
	motor->controller.angle_PID.Reset();
	motor->enabled = false;
}

void DJI_Motor_Control(DJI_Motor_s** motor_list, const uint8_t count, const float dt_s)
{
	//初始化与前置检查
	if (motor_list == nullptr || count == 0U || !(dt_s > 0.0F))
		return;
	struct CommandFrame
	{
		FDCAN_HandleTypeDef*        bus{};
		CAN_<classicCan>::Instance* sender{};//发送人
		uint32_t                    id{};
		std::array<uint8_t, 8>      payload{};
	};
	// 同一物理总线 + 同一帧 ID 只发送一次，M3508/M2006 可共用 0x200。
	std::array<CommandFrame, 16> frames{};
	uint8_t                      frameCount = 0U;
		//遍历电机,计算输出合并帧
	for (uint8_t i = 0; i < count; ++i)
	{
		DJI_Motor_s* motor = motor_list[i];
		if (motor == nullptr || motor->can_instance == nullptr)
			continue;

		const MotorTypeSpec* spec = FindSpec(motor->motor_type);//查找规格
		DjiCommandAddress    address{};

		if (spec == nullptr || !DjiResolveCommand(motor->can_instance->attribute.can_id, address))
			continue;

		//合并帧
		FDCAN_HandleTypeDef* bus        = motor->can_instance->dependency.hcan;
		uint8_t              frameIndex = 0U;

		while (frameIndex < frameCount &&
			   (frames[frameIndex].bus != bus || frames[frameIndex].id != address.frame_id))
			++frameIndex;

		if (frameIndex == frameCount)
		{
			if (frameCount >= frames.size())
				return;
			frames[frameIndex].bus    = bus;
			frames[frameIndex].sender = motor->can_instance;
			frames[frameIndex].id     = address.frame_id;
			++frameCount;
		}
		//安全拷贝
		taskENTER_CRITICAL();
		const Motor_Feedback_s feedback = motor->feedback; //接收反馈
		taskEXIT_CRITICAL();

		//目标新鲜度检查与计算
		float      output       = 0.0F;//目标控制量
		const bool positionLoop = motor->motor_settings.close_loop_type == ANGLE_AND_SPEED_LOOP ||
								  motor->motor_settings.close_loop_type == ALL_THERE_LOOP;
		const bool targetFresh = !positionLoop || motor->motor_settings.angle_epoch == feedback.angle_epoch;

		//计算输出
		if (motor->enabled && targetFresh)
			output = ComputeOutput(*motor, feedback, dt_s);

		//限幅与量化
		const auto value = static_cast<uint16_t>(ClampOutput(output, spec->output_limit));
		frames[frameIndex].payload[address.slot * 2U] = static_cast<uint8_t>(value >> 8U);
		frames[frameIndex].payload[address.slot * 2U + 1U] = static_cast<uint8_t>(value);
	}
	for (uint8_t i = 0; i < frameCount; ++i)
	{
		frames[i].sender->attribute.master_id = frames[i].id;
		classicCan::Input input{};
		input.tx_len      = classicCan::DATA_SIZE;
		input.tx_buffer   = frames[i].payload;
		// FIFO 满时本周期不阻塞，下个控制周期重新发送最新值。canConfig::classicCan::Attribute
		const auto result = frames[i].sender->Transmit(input);//发送控制帧
		for (uint8_t j = 0; j < count; ++j)
		{
			auto*             motor = motor_list[j]; //遍历电机
			DjiCommandAddress address{};             //准备地址结构
			if (motor == nullptr || motor->can_instance == nullptr ||
				motor->can_instance->dependency.hcan != frames[i].bus ||
				!DjiResolveCommand(motor->can_instance->attribute.can_id, address) ||
				address.frame_id != frames[i].id) //电机空指针;CAN示例空指针；CAN总线不匹配；地址解析失败；帧ID不匹配
				continue;
			++motor->tx_enqueue_attempts;
			if (result != HAL_OK) ++motor->tx_enqueue_failures;
		}
	}
}
