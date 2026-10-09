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
#include "RemoteCommandTest.hpp"
#include "algorithm/lqi.h"
#include "algorithm/lqi_config.h"
#include "motor/angle_target.h"
#include "motor/dji/dji_protocol.h"

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
 * @brief 云台电机全部使用 GM6020, 由右摇杆控制角速度。
 *
 * 核对清单 (以下取值需与实车接线一致, 猜错会导致电机不转或抖动):
 *   1. 电机挂在哪一路 FDCAN:
 *      FDCAN1 接 pitch; FDCAN2 接双摩擦轮，双板通信当前关闭 (RobotConfig.hpp);
 *      FDCAN3 当前空闲, 若云台实际接在 FDCAN3 请改 kGimbalCanHandle。
 *   2. 每台 GM6020 的电调 ID 决定其反馈 ID: ID1->0x205, ID2->0x206, ID3->0x207, ID4->0x208。
 *      用户确认 yaw=ID2, pitch=ID1。反馈 ID 必须唯一, 且必须是 0x205..0x208 之一。
 *   3. 转向: 若某轴转动方向相反, 把对应电机的 kYawMotorReverse / kPitchMotorReverse 置 true。
 *   4. 两轴使用 LQI；Manual 直接填写 K，BootSolve 须填写实车离散模型与 Q/R。
 */

/** @brief 云台使用的 FDCAN 控制器。 */
inline FDCAN_HandleTypeDef* const kGimbalCanHandle = &hfdcan1;

/** @brief 两轴 LQI 运动使能；pitch 保留行程方向保护。 */
inline constexpr bool kEnableYaw   = true;
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

/**
 * @brief 位置模式请求；标定模式保留该配置，但禁止两轴运动。
 * @note 开启前先记录物理零位的 13 位编码器值，并实测相对零位的可动角度。
 */
inline constexpr bool kYawPositionMode   = true; // 与 pitch 一样使用位置目标，松杆保持。
inline constexpr bool kPitchPositionMode = true; // 当前使用 LQI 位置目标；松杆保持。

/**
 * @brief 相对物理零位的行程。continuous 仅用于已确认可以连续旋转的 yaw 轴。
 * @note 非连续轴的行程必须落在零点 ±180 度内；跨越多圈需另配绝对位置传感器。
 */
inline constexpr uint16_t kPitchZeroRaw        = 893U;
inline constexpr uint16_t kPitchTopRaw         = 120U;
inline constexpr uint16_t kPitchBottomRaw      = 1490U;
inline constexpr uint16_t kYawZeroRaw          = 3741U;

/** @brief 上、下实测顶点各向行程内留 10° 保护区；保护区内仅允许向水平零位方向运动。 */
inline constexpr float    kPitchLimitMarginDeg = 7.0F;

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
	.zero_raw   = kPitchZeroRaw,
	.min_deg    = (kPitchMotorReverse ? -kPitchBottomDeg : kPitchTopDeg) + kPitchLimitMarginDeg,
	.max_deg    = (kPitchMotorReverse ? -kPitchTopDeg : kPitchBottomDeg) - kPitchLimitMarginDeg
};

//如果开启偏航位置模式 (kYawPositionMode=true)，则必须配置好偏航角度限制
static_assert(kEncoderCalibrationOnly || !kEnableYaw || !kYawPositionMode || kYawAngleLimits.IsConfigured(),
			  "Yaw position mode requires measured travel or confirmed continuous rotation");

//开启俯仰位置控制模式，就必须配置非连续型的实测机械角度限位，编译期静态检查，不满足则编译失败。
static_assert(kEncoderCalibrationOnly || !kEnablePitch || !kPitchPositionMode ||
			  (!kPitchAngleLimits.continuous && kPitchAngleLimits.IsConfigured()),
			  "Pitch position mode requires measured mechanical limits");

/** @brief 位置模式下摇杆打满时目标角度的最大变化率，单位度每秒。 */
inline constexpr float kYawTargetRateDps          = 120.0F; // 与 pitch 一致。
inline constexpr float kPitchTargetRateDps        = 120.0F;
/** @brief 备用 yaw 速度测试参数；当前位置模式不使用这两个参数。 */
inline constexpr float kYawSpeedTestMaxDps        = 60.0F;
inline constexpr float kYawSpeedTestCurrentLimitA = 2.0F;

/** @brief 固定电流单次诊断；默认关闭，不自动执行。 */
inline constexpr bool     kYawFixedCurrentTestEnabled        = false; //是否开启固定电流单次诊断
inline constexpr float    kYawFixedCurrentTestA              = 1.3F;  //测试电流幅值，单位A，方向由摇杆决定
inline constexpr uint32_t kYawFixedCurrentTestDurationMs     = 200U;  // 单次测试最长持续时间，单位ms
inline constexpr float    kYawFixedCurrentTestStickThreshold = 0.2F;  // 触发阈值：归一化摇杆绝对值达到0.2
static_assert(kYawFixedCurrentTestA > 0.0F && kYawFixedCurrentTestA <= 3.0F,
			  "Fixed current diagnostic must stay within the GM6020 protocol limit");
static_assert(!kYawFixedCurrentTestEnabled || kYawFixedCurrentTestA <= kYawSpeedTestCurrentLimitA,
			  "Enabled fixed current diagnostic must stay within the yaw current limit");
static_assert(kYawFixedCurrentTestDurationMs > 0U && kYawFixedCurrentTestDurationMs <= 500U,
			  "Fixed current diagnostic duration must stay within 500ms");

/** @brief 云台控制周期及 LQI 电流限幅；与协议最大 ±3A 分开配置。 */
inline constexpr float kGimbalControlPeriodS = 0.001F;
inline constexpr float kGimbalCurrentLimitA  = 3.0F;
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
inline constexpr GimbalLqiGainSource kYawLqiGainSource   = GimbalLqiGainSource::Manual;

/** @brief pitch 独立 LQI 增益：位置项 A/度，速度项 A/(度/秒)；保持原有效数值。 */
inline constexpr LQI_Init_Config_s kPitchLqiConfig{
	.k_angle            = 0.1F,
	.k_speed            = 0.01F, .k_integral = 0.01F,
	.q_angle            = 0.04F, .q_speed = 0.000278F, .q_integral = 0.01F, .r_current = 1.0F,
	.sample_period_s    = kGimbalControlPeriodS,
	.output_min_a       = -kGimbalCurrentLimitA, .output_max_a = kGimbalCurrentLimitA,
	.integral_min_deg_s = 0.0F, .integral_max_deg_s            = 0.0F
};
// yaw 独立配置；不复制 pitch，修改任一轴 K、Q/R 或积分范围不影响另一轴。
inline constexpr LQI_Init_Config_s kYawLqiConfig{
	.k_angle            = 0.28F,
	.k_speed            = 0.003F,
	.k_integral         = 0.01F,
	.q_angle            = 0.04F,
	.q_speed            = 0.000278F,
	.q_integral         = 0.01F,
	.r_current          = 1.0F,
	.sample_period_s    = kGimbalControlPeriodS,
	.output_min_a       = -3.0F,
	.output_max_a       = 3.0F,
	.integral_min_deg_s = 0.0F,
	.integral_max_deg_s = 0.0F
};

// 仅 yaw 正常位置模式使用；目标变化率前馈，单位 A/(度/秒)。
inline constexpr float kYawTargetSpeedFeedforwardAperDps = 0.002F;
static_assert(kYawTargetSpeedFeedforwardAperDps >= 0.0F &&
			  kYawTargetSpeedFeedforwardAperDps <= kGimbalCurrentLimitA / kYawTargetRateDps);

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
