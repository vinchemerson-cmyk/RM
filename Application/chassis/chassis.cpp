//
// Created by 19923 on 2026/10/12.
//

#include "chassis.h"

#include "bsp_log.h"
#include "ChassisConfig.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "task.h"
#include "algorithm/lqi.h"
#include "algorithm/lqi_controller.h"
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
		LQIController                lqicontroller;
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


	DJI_Motor_s* motors[] = {
		fl.motor,
		fr.motor,
		bl.motor,
		br.motor,
	};

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

		config.controller_setting_init_config.close_loop_type    = CURRENT_LOOP;
		config.controller_setting_init_config.outer_loop_type    = CURRENT_LOOP;
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

	void InitializeLqi(const char*              axis, const LQI_Model_s& model,
					   const LQI_Init_Config_s& source, LQIController&   controller,
					   ChassisLqiGainSource      gainSource)
	{
		controller.Disable();
		if (gainSource == ChaasisLqiGainSource::Manual)
		{
			if (!controller.Configure(source))
			{
				LOGERROR("[chassis] %s manual LQI K/limits invalid; axis disabled", axis);
				return;
			}
			LOGINFO("[chassis] %s LQI ready, manual K", axis);
			return;
		}
		if (gainSource != ChassisLqiGainSource::BootSolve)
		{
			LOGERROR("[chassis] %s LQI gain source invalid; axis disabled", axis);
			return;
		}
		LQI_Solve_Result_s result{};
		const auto         status = LQI_Solve(model, source, result, kChassisqiSolverOptions);
		if (status != LQI_Solve_Status::Success)
		{
			LOGERROR("[chassis] %s LQI solve failed: status=%u, iterations=%u; axis disabled",
					 axis, static_cast<unsigned>(status), static_cast<unsigned>(result.iterations));
			return;
		}
		auto config       = source;
		config.k_angle    = result.gain[0];
		config.k_speed    = result.gain[1];
		config.k_integral = result.gain[2];
		if (!controller.Configure(config))
		{
			LOGERROR("[chassis] %s LQI limits/config invalid; axis disabled", axis);
			return;
		}
		LOGINFO("[chassis] %s LQI ready, iterations=%u", axis, static_cast<unsigned>(result.iterations));
	}

	void StopWheel(DJI_Motor_s& motor,MotorSpeedTarget& Target,LQIController& controller)
	{
		DJI_Motor_Disable(&motor);
		Target.Reset();
		controller.Reset();
	}



}
