/**
 * @file pwmConfig.hpp
 * @brief 定义 PWM 设备注册所需的属性和 HAL 依赖。
 */

#ifndef PWM_CONFIG_HPP
#define PWM_CONFIG_HPP

#include <cstdint>

extern "C" {
#include "main.h"
}

// HAL PWM 定时器句柄类型.
using bspPwmHandleTypeDef = TIM_HandleTypeDef;

namespace pwmConfig
{
	static constexpr uint8_t MAX_INSTANCE = 8; // 最多注册 8 路 PWM 逻辑实例.

	// PWM 输出通道选择.
	enum class channel : uint32_t
	{
		CHANNEL_1 = TIM_CHANNEL_1,
		CHANNEL_2 = TIM_CHANNEL_2, //buzzer
		CHANNEL_3 = TIM_CHANNEL_3,
		CHANNEL_4 = TIM_CHANNEL_4,
	};

	// PWM 逻辑实例的持久属性.
	struct Attribute
	{
		channel use_channel{}; // 绑定的 PWM 输出通道.
	};

	// PWM 运行时依赖.
	struct Dependency
	{
		bspPwmHandleTypeDef* tim_handle{}; // 绑定的 HAL 定时器句柄.
	};

	// PWM 注册配置.
	struct Config
	{
		bspPwmHandleTypeDef* tim_handle{};  // 已由 CubeMX 初始化的定时器句柄.
		channel              use_channel{}; // 已由 CubeMX 配置为 PWM 的输出通道.
	};
} // namespace pwmConfig

using pwmType       = pwmConfig::Attribute;
using pwmConfigType = pwmConfig::Config;

#endif
