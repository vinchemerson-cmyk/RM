//
// Created by 19923 on 2026/10/23.
//

#ifndef INFANTRY_H723_SPEED_TARGET_H
#define INFANTRY_H723_SPEED_TARGET_H
#include <cmath>

struct MotorSpeedTargetLimits
{
	float max_speed_dps;
	float acceleration_dps2;
	float dcceleration_dps2;

	[[nodiscard]] bool IsValid() const
	{
		return std::isfinite(max_speed_dps) &&
			   std::isfinite(acceleration_dps2) &&
			   std::isfinite(dcceleration_dps2) &&
			   max_speed_dps > 0.0f &&
			   acceleration_dps2 > 0.0f &&
			   dcceleration_dps2 > 0.0f;
	}
};

class MotorSpeedTarget
{
	public:
		void Reset()
		{
			target_dps_ = 0.0F;
		}

		float Step(float requested_dps,
			const MotorSpeedTargetLimits& limits,
			float dt_s)
		{
			if (!limits.IsValid() ||
			!std::isfinite(requested_dps) ||
			!std::isfinite(dt_s) || dt_s <= 0.0F)
			{
				Reset();
				return target_dps_;
			}
			if (requested_dps > limits.max_speed_dps)
				requested_dps = limits.max_speed_dps;
			else if (requested_dps < -limits.max_speed_dps)
				requested_dps = -limits.max_speed_dps;

			// 反向时先减速到零，下一周期再向相反方向加速。
			const bool reversing =
				(target_dps_ > 0.0F && requested_dps < 0.0F) ||
				(target_dps_ < 0.0F && requested_dps > 0.0F);

			const float goal = reversing ? 0.0F : requested_dps;
			const bool accelerating =
				std::fabs(goal) > std::fabs(target_dps_);

			const float rate = accelerating
				? limits.acceleration_dps2
				: limits.dcceleration_dps2;

			const float max_change = rate * dt_s;
			float change = goal - target_dps_;

			if (change > max_change)
				change = max_change;
			else if (change < -max_change)
				change = -max_change;

			target_dps_ += change;
			return target_dps_;
		}

		[[nodiscard]] float Target() const
		{
			return target_dps_;
		}

	private:
		float target_dps_{};
};


#endif //INFANTRY_H723_SPEED_TARGET_H
