//
// Created by 19923 on 2026/10/24.
//

#ifndef INFANTRY_H723_CHASSISCONFIG_H
#define INFANTRY_H723_CHASSISCONFIG_H
#include "chassis_control.h"
#include "fdcan.h"
#include "stm32h7xx_hal_fdcan.h"
#include "algorithm/pid.h"
#include "motor/dji/dji.h"
#include "rc/rc.h"

inline FDCAN_HandleTypeDef* const kChassisCanHandle = &hfdcan1;

// 实物未确认：保持false，Init不注册电机、不创建任务；不接入Robot.cpp。
inline constexpr bool kChassisHardwareConfirmed = false;
struct ChassisWheelHardware
{
    DJI::Motor_Type_e type{DJI::MOTOR_TYPE_NONE};
    uint32_t feedback_id{}; // M3508/M2006: 0x201..0x208；0表示未填写。
    bool reverse{};
    float gear_ratio{};    // 转子速度 / 输出轴速度，须填写实际值。
};
// FL、FR、BL、BR；同总线同指令帧必须由本任务独占发送。
inline constexpr std::array<ChassisWheelHardware, chassis::kWheelCount> kChassisWheels{};
inline constexpr std::array<chassis::WheelGeometry, chassis::kWheelCount> kChassisGeometry{};
inline constexpr chassis::MotionLimits kChassisMotionLimits{};

inline constexpr unsigned kChassisForwardChannel    = 1U;
inline constexpr unsigned kChassisLeftChannel       = 0U;
inline constexpr unsigned kChassisYawChannel        = 2U;
inline constexpr float    kChassisForwardSign       = 1.0F;
inline constexpr float    kChassisLeftSign          = -1.0F;
inline constexpr float    kChassisYawSign           = -1.0F;
inline constexpr float    kChassisStickDeadband     = 0.03F;
inline constexpr auto     kChassisRunSwitch         = rc::Right_Switch_e::MIDDLE;
inline constexpr uint32_t kChassisFeedbackTimeoutMs = 100U;

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

inline constexpr float kChassisControlPeriodS = 0.001F;

inline constexpr float kChassisMaxDtS = 0.1F;

#endif //INFANTRY_H723_CHASSISCONFIG_H
