#include "dji_controller.h"

namespace
{
bool ConfigureOptional(PIDInstance& pid, const PID_Init_Config_s& config)
{
    // 真正全零才视为未配置；异常值不能静默变成停用环。
    if (config.kp == 0.0F && config.ki == 0.0F && config.kd == 0.0F &&
        config.output_min == 0.0F && config.output_max == 0.0F &&
        config.integral_min == 0.0F && config.integral_max == 0.0F)
    { pid.Disable(); return true; }
    return pid.Configure(config);
}
bool ConfigureAll(DJ_Motor_Controller_s& controller, const DJ_Motor_Controller_Init_s& config)
{
    const bool current_ok = ConfigureOptional(controller.current_PID, config.current_PID);
    const bool speed_ok = ConfigureOptional(controller.speed_PID, config.speed_PID);
    const bool angle_ok = ConfigureOptional(controller.angle_PID, config.angle_PID);
    return current_ok && speed_ok && angle_ok;
}
}

bool DJ_Motor_Controller_InitPids(DJ_Motor_Controller_s& controller,
                                const DJ_Motor_Controller_Init_s& config)
{
    controller.current_PID.Disable();
    controller.speed_PID.Disable();
    controller.angle_PID.Disable();
    if (ConfigureAll(controller, config)) return true;
    controller.current_PID.Disable();
    controller.speed_PID.Disable();
    controller.angle_PID.Disable();
    return false;
}
