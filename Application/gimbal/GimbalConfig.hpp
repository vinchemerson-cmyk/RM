/**
 * @file GimbalConfig.hpp
 * @brief 云台 yaw/pitch 轴的硬件分配与控制参数。
 *
 * @note 本文件集中存放"必须与实车一致"的硬件事实, 上机前请逐条核对。
 */

#ifndef INFANTRY_H723_GIMBAL_CONFIG_HPP
#define INFANTRY_H723_GIMBAL_CONFIG_HPP

#include <cstdint>

#include "main.h"
#include "fdcan.h"
#include "algorithm/pid.h"
#include "motor/angle_target.h"

/**
 * @brief 云台电机全部使用 GM6020, 由右摇杆控制角速度。
 *
 * 核对清单 (以下取值需与实车接线一致, 猜错会导致电机不转或抖动):
 *   1. 电机挂在哪一路 FDCAN:
 *      FDCAN1 已被本模块占用; FDCAN2 被双板通信 interboard 独占 (RobotConfig.hpp);
 *      FDCAN3 当前空闲, 若云台实际接在 FDCAN3 请改 kGimbalCanHandle。
 *   2. 每台 GM6020 的电调 ID 决定其反馈 ID: ID1->0x205, ID2->0x206, ID3->0x207, ID4->0x208。
 *      本文件默认 yaw=ID1, pitch=ID2。反馈 ID 必须唯一, 且必须是 0x205..0x208 之一。
 *   3. 转向: 若某轴转动方向相反, 把对应电机的 kYawMotorReverse / kPitchMotorReverse 置 true。
 *   4. PID 增益为经验起点, 必须实车整定。
 */

/** @brief 云台使用的 FDCAN 控制器。 */
inline FDCAN_HandleTypeDef* const kGimbalCanHandle = &hfdcan1;

/** @brief yaw 轴电机的反馈 ID (对应 GM6020 电调 ID1)。 */
inline constexpr uint32_t kYawMotorFeedbackId = 0x205U;

/** @brief pitch 轴电机的反馈 ID (对应 GM6020 电调 ID2)。 */
inline constexpr uint32_t kPitchMotorFeedbackId = 0x206U;

/** @brief 电机安装方向: 置 true 表示该轴输出与期望方向相反。 */
inline constexpr bool kYawMotorReverse   = false;
inline constexpr bool kPitchMotorReverse = false;

/** @brief GM6020 位于指令帧 0x1FF 的第 0 和第 1 个槽位, 即 ID1 与 ID2。 */

/** @brief 摇杆最大角速度指令, 单位度每秒; 对应摇杆打满。 */
inline constexpr float kYawMaxSpeedDps   = 400.0F;
inline constexpr float kPitchMaxSpeedDps = 300.0F;

/** @brief yaw 轴速度环 PID 初值。 */
inline constexpr float kYawSpeedKp = 14.0F;
inline constexpr float kYawSpeedKi = 0.05F;
inline constexpr float kYawSpeedKd = 0.0F;

/** @brief pitch 轴速度环 PID 初值。 */
inline constexpr float kPitchSpeedKp = 12.0F;
inline constexpr float kPitchSpeedKi = 0.05F;
inline constexpr float kPitchSpeedKd = 0.0F;

/**
 * @brief 角度环参数预置；输入角度为度，输出为速度目标（度每秒）。
 * @note 摇杆默认控制角速度；位置模式须先完成下面的行程标定。
 */
inline constexpr PID_Init_Config_s kYawAnglePid{
    .kp = 3.0F, .ki = 0.0F, .kd = 0.0F,
    .output_min = -120.0F, .output_max = 120.0F, .integral_min = 0.0F, .integral_max = 0.0F
};
inline constexpr PID_Init_Config_s kPitchAnglePid{
    .kp = 3.0F, .ki = 0.0F, .kd = 0.0F,
    .output_min = -80.0F, .output_max = 80.0F, .integral_min = 0.0F, .integral_max = 0.0F
};

/**
 * @brief 位置模式开关。俯仰机械限位和 yaw 旋转范围尚未实测，保持关闭。
 * @note 开启前先记录物理零位的 13 位编码器值，并实测相对零位的可动角度。
 */
inline constexpr bool kYawPositionMode   = false; //YAW的位置模式，
inline constexpr bool kPitchPositionMode = false; //PITCH的位置模式，梁桐是糖宝66666

/**
 * @brief 相对物理零位的行程。continuous 仅用于已确认可以连续旋转的 yaw 轴。
 * @note 非连续轴的行程必须落在零点 ±180 度内；跨越多圈需另配绝对位置传感器。
 */
inline constexpr MotorAngleLimits kYawAngleLimits{};   //YAW轴限位
inline constexpr MotorAngleLimits kPitchAngleLimits{}; //PITCH轴限位

//如果开启偏航位置模式 (kYawPositionMode=true)，则必须配置好偏航角度限制
static_assert(!kYawPositionMode || kYawAngleLimits.IsConfigured(),
              "Yaw position mode requires measured travel or confirmed continuous rotation");

//开启俯仰位置控制模式，就必须配置非连续型的实测机械角度限位，编译期静态检查，不满足则编译失败。
static_assert(!kPitchPositionMode ||
              (!kPitchAngleLimits.continuous && kPitchAngleLimits.IsConfigured()),
              "Pitch position mode requires measured mechanical limits");

/** @brief 位置模式下摇杆打满时目标角度的最大变化率，单位度每秒。 */
inline constexpr float kYawTargetRateDps   = 60.0F;
inline constexpr float kPitchTargetRateDps = 40.0F;

/** @brief 超过该时间未收到电机反馈时，关闭该轴输出，单位毫秒。 */
inline constexpr uint32_t kGimbalFeedbackTimeoutMs = 100U;

/** @brief 云台控制周期与 PI 输出限幅 (与 GM6020 电压指令范围一致)。 */
inline constexpr float kGimbalControlPeriodS = 0.001F;
inline constexpr float kGimbalOutputLimit    = 25000.0F;

/** @brief 单次控制周期允许的最大 dt, 超过则认为调度异常并跳过本次输出。 */
inline constexpr float kGimbalMaxDtS = 0.05F;

#endif // INFANTRY_H723_GIMBAL_CONFIG_HPP
