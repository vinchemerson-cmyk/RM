//
// Created by 19923 on 2026/10/12.
//

#include "chassis.h"

#include "cmsis_os2.h"
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
		DJI_Motor_s* motor{};
		MotorSpeedTarget speedTarget;
		LQIController lqicontroller;
		Motor_Feedback_s motorFeedback;
		uint32_t ControTick{};
		serialTasks::EncoderSnapshot encoderSnapshot{};
	};

	chassissinglemotor fl,fr,bl,br;

	constexpr osThreadAttr_t taskAttrs = {
		.name       = "ChassisTask",
		.stack_size = 2048U,
		.priority   = osPriorityHigh,
	};


	DJI_Motor_s* chassisMotors[4] = {nullptr, nullptr, nullptr, nullptr};




}

