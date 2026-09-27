/**
 * @file motor_common.cpp
 * @brief 配置各型号电机共用的三个 PID 控制环。
 */

#include "motor_common.h"

bool Motor_Controller_InitPids(Motor_Controller_s& controller,
                               const Motor_Controller_Init_s& config)
{
    const bool current_ok = controller.current_PID.Configure(config.current_PID);
    const bool speed_ok = controller.speed_PID.Configure(config.speed_PID);
    const bool angle_ok = controller.angle_PID.Configure(config.angle_PID);
    if (current_ok && speed_ok && angle_ok)
        return true;

    controller.current_PID.Disable();
    controller.speed_PID.Disable();
    controller.angle_PID.Disable();
    return false;
}
