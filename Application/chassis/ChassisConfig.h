//
// Created by 19923 on 2026/10/24.
//

#ifndef INFANTRY_H723_CHASSISCONFIG_H
#define INFANTRY_H723_CHASSISCONFIG_H
#include "algorithm/pid.h"
#include "fdcan.h"
#include "stm32h7xx_hal_fdcan.h"

inline FDCAN_HandleTypeDef* const kChassisCanHandle = &hfdcan1;

/**
 * @brief 四轮速度 PI 的联调初值，尚未经过底盘负载验证。
 * @note 当前 DJI 驱动的速度目标和反馈均为转子 °/s，输出为电调电流指令码，非 A。
 *       应用层输出轴目标接入时须在边界乘实际减速比，不得直接混用两种速度。
 *       后续根据电机型号、负载和功率预算调整；此配置不会注册或使能电机。
 */
inline constexpr PID_Init_Config_s kChassisWheelSpeedPi{
    .kp           = 0.04F,
    .ki           = 0.05F,
    .kd           = 0.0F,
    .output_min   = -4000.0F,
    .output_max   = 4000.0F,
    .integral_min = -2000.0F,
    .integral_max = 2000.0F,
};
static_assert(kChassisWheelSpeedPi.kd == 0.0F, "Chassis wheel control uses PI only");

#endif //INFANTRY_H723_CHASSISCONFIG_H
