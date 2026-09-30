/**
 * @file motor_common.cpp
 * @brief 配置各型号电机共用的三个 PID 控制环。
 */

#include "motor_common.h"

namespace
{
/** @brief 全零参数表示该控制环未启用。 */
bool IsUnusedPid(const PID_Init_Config_s& config)
{
    return config.kp == 0.0F && config.ki == 0.0F && config.kd == 0.0F &&
           config.output_min == 0.0F && config.output_max == 0.0F &&
           config.integral_min == 0.0F && config.integral_max == 0.0F;
}

bool ConfigureOptionalPid(PIDInstance& instance, const PID_Init_Config_s& config)
{
    if (IsUnusedPid(config))
    {
        instance.Disable();
        return true;
    }
    return instance.Configure(config);
}
} // namespace

bool Motor_Controller_InitPids(Motor_Controller_s& controller,
                               const Motor_Controller_Init_s& config)
{
    const bool current_ok = ConfigureOptionalPid(controller.current_PID, config.current_PID);
    const bool speed_ok   = ConfigureOptionalPid(controller.speed_PID, config.speed_PID);
    const bool angle_ok   = ConfigureOptionalPid(controller.angle_PID, config.angle_PID);
    if (current_ok && speed_ok && angle_ok)
        return true;

    controller.current_PID.Disable();
    controller.speed_PID.Disable();
    controller.angle_PID.Disable();
    return false;
}
