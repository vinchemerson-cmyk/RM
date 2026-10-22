//
// Created by 19923 on 2026/10/12.
//

#include "chassis.h"

#include "cmsis_os2.h"
#include "RobotConfig.hpp"


struct DJI_Motor_s;

namespace
{
	osThreadId_t taskHandle = nullptr;
	DJI_Motor_s* fl = nullptr;
	DJI_Motor_s* fr = nullptr;
	DJI_Motor_s* bl = nullptr;
	DJI_Motor_s* br = nullptr;

	DJI_Motor_s* chassisMotors[4] = {nullptr, nullptr, nullptr, nullptr};

	MotorSpeedTarget flSpeedTarget{};
	MotorSpeedTarget frSpeedTarget{};
	MotorSpeedTarget blSpeedTarget{};
	MotorSpeedTarget brSpeedTarget{};



}

