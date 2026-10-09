/**
 * @file pid.cpp
 * @brief PID 的限幅、条件积分抗饱和和测量微分实现。
 */

#include "pid.h"

#include <cmath>

namespace
{
	float Clamp(float value, float minimum, float maximum)
	{
		return value < minimum ? minimum : (value > maximum ? maximum : value);
	}
}

bool PIDInstance::Configure(const PID_Init_Config_s& config)
{
	configured_ = false;
	Reset();
	if (!std::isfinite(config.kp) || !std::isfinite(config.ki) || !std::isfinite(config.kd) ||
		!std::isfinite(config.output_min) || !std::isfinite(config.output_max) ||
		!std::isfinite(config.integral_min) || !std::isfinite(config.integral_max) ||
		config.kp < 0.0F || config.ki < 0.0F || config.kd < 0.0F ||
		config.output_min >= config.output_max || config.integral_min > config.integral_max ||
		config.output_min > 0.0F || config.output_max < 0.0F ||
		config.integral_min > 0.0F || config.integral_max < 0.0F)
		return false;

	config_     = config;
	configured_ = true;
	return true;
}

void PIDInstance::Reset()
{
	integral_         = 0.0F;
	last_measurement_ = 0.0F;
	output_           = 0.0F;
	has_measurement_  = false;
}

void PIDInstance::Disable()
{
	configured_ = false;
	Reset();
}

float PIDInstance::Update(float target, float measurement, float dt_s)
{
	if (!configured_ || !std::isfinite(target) || !std::isfinite(measurement) ||
		!std::isfinite(dt_s) || dt_s <= 0.0F)
	{
		Reset();
		return 0.0F;
	}

	const float error        = target - measurement;
	const float proportional = config_.kp * error;
	const float derivative   = has_measurement_
								 ? -config_.kd * (measurement - last_measurement_) / dt_s
								 : 0.0F;
	const float candidate_integral = Clamp(integral_ + config_.ki * error * dt_s,
										   config_.integral_min, config_.integral_max);
	if (!std::isfinite(proportional) || !std::isfinite(derivative) ||
		!std::isfinite(candidate_integral))
	{
		Reset();
		return 0.0F;
	}

	const float candidate_output = proportional + candidate_integral + derivative;
	if (!std::isfinite(candidate_output))
	{
		Reset();
		return 0.0F;
	}

	// 饱和方向与误差同向时暂停积分；退出饱和时仍允许积分恢复。
	if (!((candidate_output > config_.output_max && error > 0.0F) ||
		  (candidate_output < config_.output_min && error < 0.0F)))
		integral_ = candidate_integral;

	output_ = Clamp(proportional + integral_ + derivative,
					config_.output_min, config_.output_max);
	last_measurement_ = measurement;
	has_measurement_  = true;
	return output_;
}
