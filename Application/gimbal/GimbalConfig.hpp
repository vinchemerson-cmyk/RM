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
#include "yawonetest.hpp"
#include "algorithm/lqi.h"
#include "algorithm/lqi_config.h"
#include "motor/angle_target.h"

/**
 *@brief 重复遥控指令对照测试：默认关闭，不改变 DBUS 解析和安全状态。
 */
inline constexpr bool                            kYawRemoteCommandTestEnabled = false;
inline constexpr gimbal::RemoteCommandTestConfig kYawRemoteCommandTestConfig{};
static_assert(kYawRemoteCommandTestConfig.yaw_stick > 0.0F &&
			  kYawRemoteCommandTestConfig.yaw_stick <= 1.0F);
static_assert(kYawRemoteCommandTestConfig.move_ms > 0U);
static_assert(static_cast<uint64_t>(kYawRemoteCommandTestConfig.initial_hold_ms) +
			  2ULL * kYawRemoteCommandTestConfig.move_ms +
			  2ULL * kYawRemoteCommandTestConfig.hold_ms < 0x80000000ULL);

/**
 * @brief
 *
 * 核对清单 (以下取值需与实车接线一致, 猜错会导致电机不转或抖动):
 *   1. 电机挂在哪一路 FDCAN:
 *      FDCAN1 接 pitch; FDCAN2 接双摩擦轮，双板通信当前关闭 (RobotConfig.hpp);
 *      FDCAN3 当前空闲, 若云台实际接在 FDCAN3 请改 kGimbalCanHandle。
 *   2. DM 电机的 master_id 是 MIT 命令 ID，can_id 是反馈帧 CAN ID，两者不能混用。
 *      下列反馈 ID 仍沿用原接线记录，必须根据达妙调试助手重新核对。
 *   3. 转向: 若某轴转动方向相反, 把对应电机的 Reverse 配置置 true。
 *   4. 两轴使用 LQI 转矩控制；Manual 直接填写 K，BootSolve 须填写实车离散模型与 Q/R。
 */

/** @brief 云台使用的 FDCAN 控制器。 */
inline FDCAN_HandleTypeDef* const kGimbalCanHandle = &hfdcan3;

/** @brief 两轴 LQI 运动使能；pitch 保留行程方向保护。 */
inline constexpr bool kEnableYaw   = true;
inline constexpr bool kEnableAimPitch = true;
inline constexpr bool kEnableFoldingPitch = true; // 控制与实测限位尚未实现，保持禁用。

/** @brief 独立于 yaw 运动使能保留编码器采集；禁用运动后仍可打印反馈。 */
inline constexpr bool kReadYawEncoder = true;

/** @brief 读取零位时只注册两轴反馈并发送零转矩，不执行云台运动；标定完成后才能关闭。 */

/** @brief DM MIT 命令 ID 与反馈 CAN ID；必须按实车配置核对。 */
// TODO: 暂按原 yaw=ID2、pitch=ID1 的记录填写，须与达妙调试助手一致。
inline constexpr uint32_t kYawMotorMasterId      = 2U;
inline constexpr uint32_t kAimPitchMotorMasterId = 1U;
inline constexpr uint32_t kFoldingPitchMotorMasterId = 3U;
inline constexpr uint32_t kYawMotorFeedbackId      = 0x206U; // TODO: 原 DJI 记录，待核对 DM 配置。
inline constexpr uint32_t kAimPitchMotorFeedbackId = 0x205U; // TODO: 原 DJI 记录，待核对 DM 配置。
inline constexpr uint32_t kFoldingPitchMotorFeedbackId = 0x207U;

static_assert(kYawMotorMasterId >= 1U && kYawMotorMasterId <= 15U &&
			  kAimPitchMotorMasterId >= 1U && kAimPitchMotorMasterId <= 15U,
			  "DM MIT master ID must be in [1, 15]");
static_assert(kYawMotorMasterId != kAimPitchMotorMasterId &&
			  kYawMotorFeedbackId != kAimPitchMotorFeedbackId,
			  "Yaw and pitch DM command/feedback IDs must be unique");

/** @brief 电机安装方向: 置 true 表示该轴输出与期望方向相反。 */
inline constexpr bool kYawMotorReverse   = true;
inline constexpr bool kAimPitchMotorReverse = false;
inline constexpr bool kFoldingPitchMotorReverse = false;

/** @brief 每台 DM 电机使用独立的 MIT 命令帧，不再共享 GM6020 的 0x1FE 电流帧。 */

/**
 * @brief 位置模式请求；标定模式保留该配置，但禁止两轴运动。
 * @note 开启前先记录物理零位的 13 位编码器值，并实测相对零位的可动角度。
 */
inline constexpr bool kYawPositionMode   = true; // 与 pitch 一样使用位置目标，松杆保持。
inline constexpr bool kAimPitchPositionMode = true; // 当前使用 LQI 位置目标；松杆保持。
inline constexpr bool kFoldingPitchPositionMode = true;

/**
 * @brief 相对物理零位的行程。continuous 仅用于已确认可以连续旋转的 yaw 轴。
 * @note 非连续轴的行程必须落在零点 ±180 度内；跨越多圈需另配绝对位置传感器。
 * @fixme 根据机械安装确定
 */
inline constexpr uint16_t kAimPitchZeroRaw   = 893U;
inline constexpr uint16_t kFoldingPitchZeroRaw   = 0U;
inline constexpr uint16_t kPitchTopRaw    = 120U;
inline constexpr uint16_t kPitchBottomRaw = 1490U;
inline constexpr uint16_t kYawZeroRaw     = 3741U;

/** @brief 上、下实测顶点各向行程内留 7° 保护区；保护区内仅允许向水平零位方向运动。
 * @todo 等装车
 */
inline constexpr float kAimPitchLimitMarginDeg     = 7.0F;
inline constexpr float kFoldingPitchLimitMarginDeg = 7.0F;

inline constexpr float kAimPitchTopDeg =
	(static_cast<int32_t>(kPitchTopRaw) - kAimPitchZeroRaw) * (360.0F / 8192.0F);
inline constexpr float kAimPitchBottomDeg =
	(static_cast<int32_t>(kPitchBottomRaw) - kAimPitchZeroRaw) * (360.0F / 8192.0F);

static_assert(kAimPitchTopDeg + kAimPitchLimitMarginDeg < 0.0F &&
			  kAimPitchBottomDeg - kAimPitchLimitMarginDeg > 0.0F,
			  "Pitch endpoint guard zones must leave the horizontal zero inside the travel");

inline constexpr MotorAngleLimits kYawAngleLimits{
	.calibrated = true, .continuous = true, .zero_raw = kYawZeroRaw
};
inline constexpr MotorAngleLimits kAimPitchAngleLimits{
	.calibrated = true,
	.continuous = false,
	.zero_raw   = kAimPitchZeroRaw,
	.min_deg    = (kAimPitchMotorReverse ? -kAimPitchBottomDeg : kAimPitchTopDeg) + kAimPitchLimitMarginDeg,
	.max_deg    = (kAimPitchMotorReverse ? -kAimPitchTopDeg : kAimPitchBottomDeg) - kAimPitchLimitMarginDeg
};
// FoldingPitch 尚无实测零点和行程，保留未标定配置；启用前必须补齐。
inline constexpr uint16_t kFoldingPitchBottomDeg =0;
inline constexpr uint16_t kFoldingPitchTopDeg = 29;

inline constexpr MotorAngleLimits kFoldingPitchAngleLimits{
	.calibrated = true,
	.continuous = false,
	.zero_raw     = kFoldingPitchZeroRaw,
	.min_deg    = (kFoldingPitchMotorReverse ? -kFoldingPitchBottomDeg : kFoldingPitchTopDeg) + kFoldingPitchLimitMarginDeg,
	.max_deg    = (kFoldingPitchMotorReverse ? -kFoldingPitchTopDeg : kFoldingPitchBottomDeg) - kFoldingPitchLimitMarginDeg,
};
// static_assert(!kEnableFoldingPitch || kFoldingPitchAngleLimits.IsConfigured(),
// 			  "FoldingPitch requires measured limits before it can be enabled");

//如果开启偏航位置模式 (kYawPositionMode=true)，则必须配置好偏航角度限制
static_assert(!kEnableYaw || !kYawPositionMode || kYawAngleLimits.IsConfigured(),
			  "Yaw position mode requires measured travel or confirmed continuous rotation");

//开启俯仰位置控制模式，就必须配置非连续型的实测机械角度限位，编译期静态检查，不满足则编译失败。
static_assert(!kEnableAimPitch || !kAimPitchPositionMode ||
			  (!kAimPitchAngleLimits.continuous && kAimPitchAngleLimits.IsConfigured()),
			  "Pitch position mode requires measured mechanical limits");

/** @brief 位置模式下摇杆打满时目标角度的最大变化率，单位度每秒。 */
inline constexpr float kYawTargetRateDps          = 120.0F; // 与 pitch 一致。
inline constexpr float kAimPitchTargetRateDps     = 120.0F;
inline constexpr float kFoldingPitchTargetRateDps = 120.0F;

/** @brief 备用 yaw 速度测试参数；当前位置模式不使用这两个参数。 */
inline constexpr float kYawSpeedTestMaxDps       = 60.0F;
inline constexpr float kYawSpeedTestTorqueLimitNm = 2.0F;

// 固定转矩单次诊断；默认关闭，不自动执行。
inline constexpr bool     kYawFixedTorqueTestEnabled        = false;
inline constexpr float    kYawFixedTorqueTestNm             = 1.3F;
inline constexpr uint32_t kYawFixedTorqueTestDurationMs     = 200U;
inline constexpr float    kYawFixedTorqueTestStickThreshold = 0.2F;


static_assert(kYawFixedTorqueTestNm > 0.0F && kYawFixedTorqueTestNm <= 3.0F,
			  "Fixed torque diagnostic must stay within the provisional 3 Nm limit");
static_assert(!kYawFixedTorqueTestEnabled || kYawFixedTorqueTestNm <= kYawSpeedTestTorqueLimitNm,
			  "Enabled fixed torque diagnostic must stay within the yaw torque limit");
static_assert(kYawFixedTorqueTestDurationMs > 0U && kYawFixedTorqueTestDurationMs <= 500U,
			  "Fixed torque diagnostic duration must stay within 500ms");

/** @brief 云台控制周期及 LQI 转矩限幅；DM 增益必须重新整定后才能实车运动。 */
inline constexpr float kGimbalControlPeriodS = 0.001F;
inline constexpr float kGimbalTorqueLimitNm  = 3.0F;
static_assert(kGimbalTorqueLimitNm > 0.0F && kGimbalTorqueLimitNm <= 3.0F,
			  "Provisional DM torque limit must be within (0, 3] Nm");

/** @brief 单次控制周期允许的最大 dt, 超过则认为调度异常并跳过本次输出。 */
inline constexpr float kGimbalMaxDtS = 0.05F;

/**
 * @brief pitch/yaw LQI 参数；Manual 使用手填 K，BootSolve 根据模型和 Q/R 求解 K。
 * @note 当前手填模式不依赖 A/B、Q/R，不执行启动求解；全零 K 会被拒绝。
 *       积分上下限为零，待整定后填写。
 *       实时输出直接作为 DM MIT 转矩指令，单位 N·m；原 PID 参数已不参与两轴控制。
 */
enum class GimbalLqiGainSource
{
	Manual,
	BootSolve
};

inline constexpr auto kAimPitchLqiGainSource     = GimbalLqiGainSource::Manual;
inline constexpr auto kFoldingPitchLqiGainSource = GimbalLqiGainSource::Manual;
inline constexpr auto kYawLqiGainSource          = GimbalLqiGainSource::Manual;

//pitch 独立 LQI 增益：位置项 N·m/度，速度项 N·m/(度/秒)；原数值仅作台架起点。
inline constexpr LQI_Init_Config_s kAimPitchLqiConfig{
	.k_angle            = 0.1F,
	.k_speed            = 0.01F, .k_integral = 0.01F,
	.q_angle            = 0.04F, .q_speed    = 0.000278F, .q_integral = 0.01F, .r_current = 1.0F,
	.sample_period_s    = kGimbalControlPeriodS,
	.output_min_a       = -kGimbalTorqueLimitNm, .output_max_a = kGimbalTorqueLimitNm,
	.integral_min_deg_s = 0.0F, .integral_max_deg_s            = 0.0F
};

inline constexpr LQI_Init_Config_s kFoldingPitchLqiConfig{
	.k_angle            = 0.1F,
	.k_speed            = 0.01F, .k_integral = 0.01F,
	.q_angle            = 0.04F, .q_speed    = 0.000278F, .q_integral = 0.01F, .r_current = 1.0F,
	.sample_period_s    = kGimbalControlPeriodS,
	.output_min_a       = -kGimbalTorqueLimitNm, .output_max_a = kGimbalTorqueLimitNm,
	.integral_min_deg_s = 0.0F, .integral_max_deg_s            = 0.0F
};

// yaw 配置；
inline constexpr LQI_Init_Config_s kYawLqiConfig{
	.k_angle            = 0.28F,
	.k_speed            = 0.003F,
	.k_integral         = 0.01F,
	.q_angle            = 0.04F,
	.q_speed            = 0.000278F,
	.q_integral         = 0.01F,
	.r_current          = 1.0F,
	.sample_period_s    = kGimbalControlPeriodS,
	.output_min_a       = -kGimbalTorqueLimitNm,
	.output_max_a       = kGimbalTorqueLimitNm,
	.integral_min_deg_s = 0.0F,
	.integral_max_deg_s = 0.0F
};

// 仅 yaw 正常位置模式使用；目标变化率前馈，单位 N·m/(度/秒)。
inline constexpr float kYawTargetSpeedFeedforwardNmPerDps = 0.002F;
static_assert(kYawTargetSpeedFeedforwardNmPerDps >= 0.0F &&
			  kYawTargetSpeedFeedforwardNmPerDps <= kGimbalTorqueLimitNm / kYawTargetRateDps);

/**
 * @brief LQI 的实车离散模型入口；A/B 全零表示尚未辨识，求解器会拒绝。
 * @note
 *		 填写角度/速度模型 x[k+1]=A*x[k]+B*T[k]，输入转矩单位 N·m。
 *       模型状态与转矩须采用电机方向处理后的逻辑坐标。
 *       仅 BootSolve 模式每次上电求解一次；占位模型或求解失败时对应轴禁止输出。
 *       Manual 模式忽略模型，直接使用上面的 K。
 * @todo 在BootSolve模式下填写A/B
 * @note
 *		 A11：当前角度对下一时刻角度的影响。
 *		 A12：当前速度让角度变化多少。
 *		 A21：当前角度对下一时刻速度的影响，例如回复力。
 *		 A22：速度保留多少，反映惯性与阻尼的作用。
 *		 B1：转矩对下一时刻角度的影响。
 *		 B2：转矩对下一时刻速度的影响。
 */
inline constexpr LQI_Model_s kAimPitchLqiModel{
	.a = {}, .b = {}, .sample_period_s = kGimbalControlPeriodS
};
inline constexpr LQI_Model_s kFoldingPitchLqiModel{
	.a = {}, .b = {}, .sample_period_s = kGimbalControlPeriodS
};
inline constexpr LQI_Model_s kYawLqiModel{
	.a = {}, .b = {}, .sample_period_s = kGimbalControlPeriodS
};

/** @brief 上电单次增益求解的迭代上限与收敛阈值。 */
inline constexpr LQI_Solver_Options_s kGimbalLqiSolverOptions{};

static_assert(kAimPitchPositionMode, "Pitch LQI requires position targets");
static_assert(kYawPositionMode || kYawLqiGainSource == GimbalLqiGainSource::Manual,
			  "Yaw speed-only test requires manual gains");
static_assert(kYawSpeedTestTorqueLimitNm > 0.0F && kYawSpeedTestTorqueLimitNm <= kGimbalTorqueLimitNm,
			  "Yaw speed test current limit must remain within the gimbal current limit");

#endif // INFANTRY_H723_GIMBAL_CONFIG_HPP
