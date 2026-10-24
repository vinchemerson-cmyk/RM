//
// Created by 19923 on 2026/10/12.
//

#include "chassis.h"

#include "ChassisConfig.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "communication/encoder_telemetry.h"
#include "motor/speed_target.h"
#include "motor/dji/dji.h"


struct DJI_Motor_s;

namespace
{
	osThreadId_t taskHandle = nullptr;

	struct chassissinglemotor
	{
		DJI_Motor_s*                 motor{};
		MotorSpeedTarget             speedTarget;
		// 速度 PI 由 motor->controller.speed_PID 持有，每个电机独立维护积分。
		Motor_Feedback_s             motorFeedback;
		uint32_t                     ControTick{};
		serialTasks::EncoderSnapshot encoderSnapshot{};
	};

	chassissinglemotor fl, fr, bl, br;

	constexpr osThreadAttr_t taskAttrs = {
		.name       = "ChassisTask",
		.stack_size = 2048U,
		.priority   = osPriorityHigh,
	};


	// 注册完成后，在控制周期内由 fl/fr/bl/br.motor 组装局部指针数组再统一发送。

	DJI::Motor_Init_Config_s MakeMotorConfig(const DJI::Motor_Type_e motor_type, const uint32_t feedback_id,
											 const bool              reverse)
	{
		DJI::Motor_Init_Config_s config{};
		config.motor_type = motor_type;

		// 使用电机自身反馈，不使用外部反馈与前馈通道。
		config.controller_param_init_config.other_angle_feedback_ptr = nullptr; //其他角度反馈
		config.controller_param_init_config.other_speed_feedback_ptr = nullptr; //其他速度反馈
		config.controller_param_init_config.speed_feedforward_ptr    = nullptr; //速度前馈
		config.controller_param_init_config.current_feedforward_ptr  = nullptr; //电流前馈

		config.controller_param_init_config.speed_PID = kChassisWheelSpeedPi;
		// MCU 计算速度 PI；电流环由电调闭合，不启用 MCU 角度环或电流 PID。
		config.controller_setting_init_config.close_loop_type    = SPEED_AND_CURRENT_LOOP;
		config.controller_setting_init_config.outer_loop_type    = SPEED_LOOP;
		config.controller_setting_init_config.motor_reverse_flag =
			reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
		config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
		config.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
		config.controller_setting_init_config.feedforward_flag      = FEEDFORWARD_NONE;

		config.can_init_config.can_handle = kChassisCanHandle;
		config.can_init_config.can_id     = feedback_id;
		return config;
	}

	Motor_Feedback_s FeedbackSnapshot(const DJI_Motor_s& motor)
	{
		taskENTER_CRITICAL();
		const Motor_Feedback_s feedback = motor.feedback;
		taskEXIT_CRITICAL();
		return feedback;
	}

	void StopWheel(DJI_Motor_s& motor, MotorSpeedTarget& target)
	{
		// Disable 已清除驱动内部 PI 历史；另清速度指令和斜坡状态。
		DJI_Motor_Disable(&motor);
		target.Reset();
	}



}
