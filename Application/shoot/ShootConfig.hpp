/** @file ShootConfig.hpp @brief 发射机构硬件默认值及控制参数；上板前按接线、拨盘槽数核对。 */
#pragma once
#include "fdcan.h"
#include "shoot_control.h"
#include "algorithm/pid.h"

namespace shoot
{
	inline FDCAN_HandleTypeDef* const kCanHandle         = &hfdcan2;
	/** @brief 三电机测试阶段不注册或驱动拨弹盘。 */
	inline constexpr bool             kEnableFeeder      = true;  //测拨弹盘是否启用
	inline constexpr uint32_t         kFeederFeedbackId  = 0x201U; // todo 拨弹盘电机	ID
	inline constexpr uint32_t         kRightFeedbackId   = 0x202U; // todo M3508/C620 ID2
	inline constexpr uint32_t         kLeftFeedbackId    = 0x201U; // todo M3508/C620 ID1
	inline constexpr bool             kFeederReverse     = false;
	inline constexpr bool             kRightReverse      = false;
	inline constexpr bool             kLeftReverse       = true;
	inline constexpr float            kFeederGearRatio   = 36.0F;//齿轮比
	inline constexpr uint32_t         kFeederPocketCount = 8U;//拨弹盘槽数
	inline constexpr ControlConfig    kControlConfig{
		.enable_feeder             = kEnableFeeder,
		.wheel_speed_dps           = 6000.0F, // 三电机联调：转子 1000 RPM，不使用发射用 6000 RPM。
		.wheel_acceleration_dps2   = 6000.0F, // 1000 RPM/s。
		.wheel_ready_tolerance_dps = 300.0F,
		.single_shot_angle_deg     = 360.0F * kFeederGearRatio / kFeederPocketCount,
	};
	inline constexpr PID_Init_Config_s kWheelSpeedPid{
		.kp           = 0.04F, .ki              = 0.05F, .kd = 0.0F,
		.output_min   = -8000.0F, .output_max   = 8000.0F,
		.integral_min = -4000.0F, .integral_max = 4000.0F,
	};
	inline constexpr PID_Init_Config_s kFeederSpeedPid{
		.kp           = 1.0F, .ki               = 0.5F, .kd = 0.0F,
		.output_min   = -8000.0F, .output_max   = 8000.0F,
		.integral_min = -3000.0F, .integral_max = 3000.0F,
	};
	inline constexpr PID_Init_Config_s kFeederAnglePid{
		.kp           = 0.005F, .ki                                   = 0.5F, .kd = 0.0F,
		.output_min   = -kControlConfig.feeder_speed_dps, .output_max = kControlConfig.feeder_speed_dps,
		.integral_min = 0.0F, .integral_max                           = 0.0F,
	};
}
