/**
 * @file other_motors.cpp
 * @brief 非 DJI 电机的 PID 配置入口。
 */

#include "other_motors.h"

bool DMMotor_Controller_InitPids(DMMotor_Controller_s& controller,
                                 const DMMotor_Controller_Init_s& config)
{
    const bool speed_ok = controller.speed_PID.Configure(config.speed_PID);
    const bool angle_ok = controller.angle_PID.Configure(config.angle_PID);
    if (speed_ok && angle_ok)
        return true;

    controller.speed_PID.Disable();
    controller.angle_PID.Disable();
    return false;
}
