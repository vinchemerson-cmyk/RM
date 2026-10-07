/**
 * @file GimbalConfig.hpp
 * @brief 云台 yaw/pitch 轴的硬件分配与控制参数。
 *
 */

#ifndef INFANTRY_H723_GIMBAL_CONFIG_HPP
#define INFANTRY_H723_GIMBAL_CONFIG_HPP

#include <cstdint>

#include "fdcan.h"
#include "main.h"
#include "algorithm/pid.h"
#include "algorithm/lqi_config.h"
#include "algorithm/lqi.h"
#include "motor/angle_target.h"
#include "motor/dji/dji_protocol.h"

/**
 * @brief 云台电机全部使用 GM6020, 由右摇杆控制角速度。
 *
 * 核对清单 (以下取值需与实车接线一致, 猜错会导致电机不转或抖动):
 *   1. 电机挂在哪一路 FDCAN:
 *      FDCAN1 接 pitch; FDCAN2 接双摩擦轮，双板通信当前关闭 (RobotConfig.hpp);
 *      FDCAN3 当前空闲, 若云台实际接在 FDCAN3 请改 kGimbalCanHandle。
 *   2. 每台 GM6020 的电调 ID 决定其反馈 ID: ID1->0x205, ID2->0x206, ID3->0x207, ID4->0x208。
 *      用户确认 yaw=ID2, pitch=ID1。反馈 ID 必须唯一, 且必须是 0x205..0x208 之一。
 *   3. 转向: 若某轴转动方向相反, 把对应电机的 kYawMotorReverse / kPitchMotorReverse 置 true。
 *   4. 两轴使用 LQI；必须填写实车离散模型与 Q/R，旧 PID 参数保留但不参与控制。
 */

/** @brief 云台使用的 FDCAN 控制器。 */
inline FDCAN_HandleTypeDef* const kGimbalCanHandle = &hfdcan1;

/** @brief pitch暂调速度环并保留行程方向保护；yaw连续位置控制不变。 */
inline constexpr bool kEnableYaw = true;
inline constexpr bool kEnablePitch = true;

/** @brief 独立于 yaw 运动使能保留编码器采集；禁用运动后仍可打印反馈。 */
inline constexpr bool kReadYawEncoder = true;

/** @brief 读取零位时只注册两轴反馈并发送零电流，不执行云台运动；标定完成后才能关闭。 */
inline constexpr bool kEncoderCalibrationOnly = false;

/** @brief yaw 轴电机的反馈 ID (对应 GM6020 电调 ID2)。 */
inline constexpr uint32_t kYawMotorFeedbackId = 0x206U;

/** @brief pitch 轴电机的反馈 ID (对应 GM6020 电调 ID1)。 */
inline constexpr uint32_t kPitchMotorFeedbackId = 0x205U;

/** @brief 电机安装方向: 置 true 表示该轴输出与期望方向相反。 */
inline constexpr bool kYawMotorReverse   = true;
inline constexpr bool kPitchMotorReverse = false;

/** @brief GM6020 电流帧 0x1FE 的第 0/1 个槽位为 pitch ID1 / yaw ID2。 */

/** @brief 摇杆最大角速度指令, 单位度每秒; 对应摇杆打满。 */
inline constexpr float kYawMaxSpeedDps   = 400.0F;
inline constexpr float kPitchMaxSpeedDps = 120.0F; // 单调速度环：打满4RPM，先支撑云台短时试拨。每24.0F为1RPM

/**
 * @brief 偏激进响应的试调起点，单位 A/(°/s)，转换为电流指令码。
 * @note 1RPM=6°/s；Kp=0.02 时速度跳变一格仅影响0.12A，旧0.5会影响3A。
 *       提高外环响应而非提高电流上限；两轴速度积分暂关，稳定性需实车确认。
 */
inline constexpr float kYawSpeedKp = 0.02F;
inline constexpr float kYawSpeedKi = 0.0F;
inline constexpr float kYawSpeedKd = 0.0F;

/** @brief pitch单调速度环：Kp=0.005A/(°/s)， */
inline constexpr float kPitchSpeedKp = 0.01F * kDjiGm6020CommandUnitsPerAmp;
inline constexpr float kPitchSpeedKi = 3.0F;
inline constexpr float kPitchSpeedKd = 0.0F;

/**
 * @brief 角度环参数预置；输入角度为度，输出为速度目标（度每秒）。
 * @note 摇杆默认控制角速度；位置模式须先完成下面的行程标定。
 */
inline constexpr PID_Init_Config_s kYawAnglePid{
    .kp = 12.0F, .ki = 0.0F, .kd = 0.0F,
    .output_min = -240.0F, .output_max = 240.0F, .integral_min = 0.0F, .integral_max = 0.0F
};
inline constexpr PID_Init_Config_s kPitchAnglePid{
    .kp = 10.0F, .ki = 0.0F, .kd = 0.0F,
    .output_min = -90.0F, .output_max = 90.0F, .integral_min = 0.0F, .integral_max = 0.0F
};

/**
 * @brief 位置模式请求；标定模式保留该配置，但禁止两轴运动。
 * @note 开启前先记录物理零位的 13 位编码器值，并实测相对零位的可动角度。
 */
inline constexpr bool kYawPositionMode   = false; // yaw 单独速度反馈测试，不保持位置。
inline constexpr bool kPitchPositionMode = true; // 当前使用 LQI 位置目标；松杆保持。

/**
 * @brief 相对物理零位的行程。continuous 仅用于已确认可以连续旋转的 yaw 轴。
 * @note 非连续轴的行程必须落在零点 ±180 度内；跨越多圈需另配绝对位置传感器。
 */
inline constexpr uint16_t kPitchZeroRaw = 893U;
inline constexpr uint16_t kPitchTopRaw = 120U;
inline constexpr uint16_t kPitchBottomRaw = 1490U;
inline constexpr uint16_t kYawZeroRaw = 3741U;
/** @brief 上、下实测顶点各向行程内留 10° 保护区；保护区内仅允许向水平零位方向运动。 */
inline constexpr float kPitchLimitMarginDeg = 10.0F;

inline constexpr float kPitchTopDeg =
    (static_cast<int32_t>(kPitchTopRaw) - kPitchZeroRaw) * (360.0F / 8192.0F);
inline constexpr float kPitchBottomDeg =
    (static_cast<int32_t>(kPitchBottomRaw) - kPitchZeroRaw) * (360.0F / 8192.0F);
static_assert(kPitchTopDeg + kPitchLimitMarginDeg < 0.0F &&
              kPitchBottomDeg - kPitchLimitMarginDeg > 0.0F,
              "Pitch endpoint guard zones must leave the horizontal zero inside the travel");

inline constexpr MotorAngleLimits kYawAngleLimits{
    .calibrated = true, .continuous = true, .zero_raw = kYawZeroRaw
};
inline constexpr MotorAngleLimits kPitchAngleLimits{
    .calibrated = true,
    .continuous = false,
    .zero_raw = kPitchZeroRaw,
    .min_deg = (kPitchMotorReverse ? -kPitchBottomDeg : kPitchTopDeg) + kPitchLimitMarginDeg,
    .max_deg = (kPitchMotorReverse ? -kPitchTopDeg : kPitchBottomDeg) - kPitchLimitMarginDeg
};

//如果开启偏航位置模式 (kYawPositionMode=true)，则必须配置好偏航角度限制
static_assert(kEncoderCalibrationOnly || !kEnableYaw || !kYawPositionMode || kYawAngleLimits.IsConfigured(),
              "Yaw position mode requires measured travel or confirmed continuous rotation");

//开启俯仰位置控制模式，就必须配置非连续型的实测机械角度限位，编译期静态检查，不满足则编译失败。
static_assert(kEncoderCalibrationOnly || !kEnablePitch || !kPitchPositionMode ||
              (!kPitchAngleLimits.continuous && kPitchAngleLimits.IsConfigured()),
              "Pitch position mode requires measured mechanical limits");

/** @brief 位置模式下摇杆打满时目标角度的最大变化率，单位度每秒。 */
inline constexpr float kYawTargetRateDps   = 180.0F;
inline constexpr float kPitchTargetRateDps = 120.0F;
/** @brief yaw 速度环测试：摇杆打满 60°/s，独立限流 ±0.5A。 */
inline constexpr float kYawSpeedTestMaxDps = 60.0F;
inline constexpr float kYawSpeedTestCurrentLimitA = 0.5F;

/** @brief 云台控制周期及低速联调电流限幅；与协议最大 ±3A 分开配置。 */
inline constexpr float kGimbalControlPeriodS = 0.001F;
inline constexpr float kGimbalCurrentLimitA  = 3.0F;
inline constexpr float kGimbalOutputLimit    = kGimbalCurrentLimitA * kDjiGm6020CommandUnitsPerAmp;
inline constexpr float kGimbalIntegralLimit  = 0.25F * kDjiGm6020CommandUnitsPerAmp;
static_assert(kGimbalCurrentLimitA > 0.0F && kGimbalCurrentLimitA <= 3.0F,
              "GM6020 test current limit must be within (0, 3] A");

/** @brief 单次控制周期允许的最大 dt, 超过则认为调度异常并跳过本次输出。 */
inline constexpr float kGimbalMaxDtS = 0.05F;

/**
 * @brief pitch/yaw LQI 参数；Manual 使用手填 K，BootSolve 根据模型和 Q/R 求解 K。
 * @note 当前手填模式不依赖 A/B、Q/R，不执行启动求解；全零 K 会被拒绝。
 *       积分上下限为零，待整定后填写。
 *       实时输出使用 A，再转换为 GM6020 电流指令码；原 PID 参数已不参与两轴控制。
 */
enum class GimbalLqiGainSource
{
    Manual,
    BootSolve
};

inline constexpr GimbalLqiGainSource kPitchLqiGainSource = GimbalLqiGainSource::Manual;
inline constexpr GimbalLqiGainSource kYawLqiGainSource = GimbalLqiGainSource::Manual;

/**
 * @brief pitch Manual K 按原串级 PID 的未限幅比例项换算为 A。
 * @note 旧指令码 = Kp_speed*(Kp_angle*(目标-实际)-速度)。
 *       因此 k_angle=Kp_angle*Kp_speed/码每安培，k_speed=Kp_speed/码每安培。
 *       不等价于还原旧角度环速度限幅或速度环积分；当前 LQI 积分范围仍为零。
 */
inline constexpr LQI_Init_Config_s kPitchLqiConfig{
    .k_angle = kPitchAnglePid.kp * kPitchSpeedKp / kDjiGm6020CommandUnitsPerAmp,
    .k_speed = kPitchSpeedKp / kDjiGm6020CommandUnitsPerAmp, .k_integral = 0.01F,
    .q_angle = 0.04F, .q_speed = 0.000278F, .q_integral = 0.01F, .r_current = 1.0F,
    .sample_period_s = kGimbalControlPeriodS,
    .output_min_a = -kGimbalCurrentLimitA, .output_max_a = kGimbalCurrentLimitA,
    .integral_min_deg_s = 0.0F, .integral_max_deg_s = 0.0F
};
inline constexpr LQI_Init_Config_s kYawLqiConfig{
    // yaw 仅调速度反馈：I=0.003*(目标速度-实际速度)，关闭位置和积分。
    .k_angle = 0.0F,
    .k_speed = 0.003F, .k_integral = 0.0F,
	.q_angle = 0.04F, .q_speed = 0.000278F, .q_integral = 0.01F, .r_current = 1.0F,
    .sample_period_s = kGimbalControlPeriodS,
    .output_min_a = -kYawSpeedTestCurrentLimitA, .output_max_a = kYawSpeedTestCurrentLimitA,
    .integral_min_deg_s = 0.0F, .integral_max_deg_s = 0.0F
};

/**
 * @brief LQI 的实车离散模型入口；A/B 全零表示尚未辨识，求解器会拒绝。
 * @note
 *		 填写角度/速度模型 x[k+1]=A*x[k]+B*I[k]，输入电流单位 A。
 *       模型状态与电流须采用电机方向处理后的逻辑坐标。
 *       仅 BootSolve 模式每次上电求解一次；占位模型或求解失败时对应轴禁止输出。
 *       Manual 模式忽略模型，直接使用上面的 K。
 * @todo 在BootSolve模式下填写A/B
 * @note
 *		 A11：当前角度对下一时刻角度的影响。
 *		 A12：当前速度让角度变化多少。
 *		 A21：当前角度对下一时刻速度的影响，例如回复力。
 *		 A22：速度保留多少，反映惯性与阻尼的作用。
 *		 B1：电流对下一时刻角度的影响。
 *		 B2：电流对下一时刻速度的影响。
 */
inline constexpr LQI_Model_s kPitchLqiModel{
    .a = {}, .b = {}, .sample_period_s = kGimbalControlPeriodS
};
inline constexpr LQI_Model_s kYawLqiModel{
    .a = {}, .b = {}, .sample_period_s = kGimbalControlPeriodS
};

/** @brief 上电单次增益求解的迭代上限与收敛阈值。 */
inline constexpr LQI_Solver_Options_s kGimbalLqiSolverOptions{};

static_assert(kPitchPositionMode, "Pitch LQI requires position targets");
static_assert(kYawPositionMode || kYawLqiGainSource == GimbalLqiGainSource::Manual,
              "Yaw speed-only test requires manual gains");
static_assert(kYawSpeedTestCurrentLimitA > 0.0F && kYawSpeedTestCurrentLimitA <= kGimbalCurrentLimitA,
              "Yaw speed test current limit must remain within the gimbal current limit");

#endif // INFANTRY_H723_GIMBAL_CONFIG_HPP
