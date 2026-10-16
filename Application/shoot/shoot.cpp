/** @file shoot.cpp @brief 将遥控拨杆、反馈与发射状态机接入 DJI CAN 闭环。 */
#include "shoot.h"
#include "ShootConfig.hpp"
#include "motor/dji/dji.h"
#include "rc/rc.h"
#include "safety/safety.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

namespace
{
	osThreadId_t             taskHandle = nullptr;
	DJI_Motor_s*             motors[3]{}; // 左摩擦轮、右摩擦轮、拨弹盘
	shoot::Controller        controller{shoot::kControlConfig};
	constexpr osThreadAttr_t taskAttrs{
		.name = "ShootTask", .stack_size = 3072U, .priority = osPriorityHigh,
	};

	/** @brief 将方向变换同时应用到输出和反馈，保持速度闭环负反馈。 */
	DJI::Motor_Init_Config_s MakeConfig(DJI::Motor_Type_e type, uint32_t id, bool reverse)
	{
		DJI::Motor_Init_Config_s config{};
		config.motor_type                                        = type;
		config.can_init_config.can_handle                        = shoot::kCanHandle;
		config.can_init_config.rx_id                             = id;
		config.controller_setting_init_config.close_loop_type    = SPEED_AND_CURRENT_LOOP;
		config.controller_setting_init_config.outer_loop_type    = SPEED_AND_CURRENT_LOOP;
		config.controller_setting_init_config.motor_reverse_flag =
			reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
		config.controller_param_init_config.speed_PID = type == DJI::M2006 ? shoot::kFeederSpeedPid : shoot::kWheelSpeedPid;
		if (type == DJI::M2006)
			config.controller_param_init_config.angle_PID = shoot::kFeederAnglePid;
		return config;
	}

	/** @brief 模式切换时复位 PID；停机后每周期仍发送零电流槽位。 */
	void SetMotor(DJI_Motor_s* motor, bool enabled, bool position, float speed, float angle,
				  uint32_t     epoch = 0U)
	{
		if (motor == nullptr) return;
		if (!enabled)
		{
			if (motor->enabled) DJI_Motor_Disable(motor);
			return;
		}
		const Closeloop_Type_e mode = position ? ANGLE_AND_SPEED_LOOP : SPEED_AND_CURRENT_LOOP;
		if (motor->motor_settings.close_loop_type != mode && motor->enabled)
			DJI_Motor_Disable(motor);
		motor->motor_settings.close_loop_type = mode;
		motor->motor_settings.outer_loop_type = mode;
		motor->motor_settings.speed           = speed;
		motor->motor_settings.angle           = angle;
		motor->motor_settings.angle_epoch     = epoch;
		if (!motor->enabled) DJI_Motor_Enable(motor);
	}

	void Task(void*)
	{
		uint32_t lastTick = osKernelGetTickCount();
		for (;;)
		{
			const uint32_t now = osKernelGetTickCount();
			float          dt  = static_cast<float>(now - lastTick) / 1000.0F;
			lastTick           = now;
			if (dt <= 0.0F) dt = 0.001F;
			const auto       remote = rc::Snapshot();
			Motor_Feedback_s feedback[3]{};
			taskENTER_CRITICAL();
			for (uint8_t i = 0U; i < 3U; ++i)
				if (motors[i] != nullptr) feedback[i] = motors[i]->feedback;
			taskEXIT_CRITICAL();
			const shoot::Trigger trigger = remote.switch_right == rc::Right_Switch_e::MIDDLE
											   ? shoot::Trigger::Single
											   : remote.switch_right == rc::Right_Switch_e::DOWN
													 ? shoot::Trigger::Continuous
													 : shoot::Trigger::Stop;
			const shoot::ControlInput input{
				.allowed            = rc::IsOnline() && safety::ShouldOutput(),
				.wheels_requested   = remote.switch_left == rc::Left_Switch_e::UP,
				.trigger            = trigger,
				.left_speed_dps     = feedback[0].speed, .right_speed_dps    = feedback[1].speed,
				.feeder_speed_dps   = feedback[2].speed, .feeder_angle_deg   = feedback[2].total_angle,
				.feeder_current_raw = feedback[2].raw_current, .feeder_epoch = feedback[2].angle_epoch,
			};
			const auto output = controller.Update(input, now, dt);
			SetMotor(motors[0], output.wheels_enabled, false, output.wheel_speed_dps, 0.0F);
			SetMotor(motors[1], output.wheels_enabled, false, output.wheel_speed_dps, 0.0F);
			SetMotor(motors[2], output.feeder_enabled, output.feeder_position_mode,
					 output.feeder_speed_dps, output.feeder_angle_deg, feedback[2].angle_epoch);
			DJI_Motor_Control(motors, 3U, dt);
			const uint32_t next = now + 1U;
			if (static_cast<int32_t>(next - osKernelGetTickCount()) > 0)
				osDelayUntil(next);
			else
				osDelay(1U);
		}
	}
}

void shoot::Init()
{
	if (taskHandle != nullptr) return;
	motors[0] = DJI_Motor_Init(MakeConfig(DJI::M3508, kLeftFeedbackId, kLeftReverse));
	motors[1] = DJI_Motor_Init(MakeConfig(DJI::M3508, kRightFeedbackId, kRightReverse));
	if constexpr (kEnableFeeder)
		motors[2] = DJI_Motor_Init(MakeConfig(DJI::M2006, kFeederFeedbackId, kFeederReverse));
	if (motors[0] == nullptr || motors[1] == nullptr || (kEnableFeeder && motors[2] == nullptr))
		Error_Handler();
	taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
	if (taskHandle == nullptr) Error_Handler();
}
