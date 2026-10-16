/**
 * @file angle_target.h
 * @brief 用连续编码器角度生成受机械行程约束的位置目标。
 */

#ifndef INFANTRY_H723_ANGLE_TARGET_H
#define INFANTRY_H723_ANGLE_TARGET_H

#include <cmath>


/**
 * @brief 计算跨越 0/8191 边界时的最短编码器增量。
 * @param previous 上次 13 位编码器读数。
 * @param current 当前 13 位编码器读数。
 * @return [-4095, 4096] 范围内的计数差。
 * @note 两次有效反馈之间转动必须小于半圈，否则无法判定真实转数。
 */
inline int16_t MotorEncoderDelta(uint16_t previous, uint16_t current)
{
	int32_t delta = static_cast<int32_t>(current & 0x1FFFU) -
					static_cast<int32_t>(previous & 0x1FFFU);
	if (delta > 4096)
		delta -= 8192;
	else if (delta < -4096)
		delta += 8192;
	return static_cast<int16_t>(delta);
}

/**
 * @brief 相对编码器零点的机械行程；continuous 只能在确认可连续旋转后启用。
 * @note 非连续轴的 min_deg/max_deg 必须实测，且各自在零点的 ±180 度内。
 */
struct MotorAngleLimits
{
	bool     calibrated{}; //是否初始化
	bool     continuous{};//连续
	uint16_t zero_raw{}; //todo
	float    min_deg{};  //todo
	float    max_deg{};  //todo

	/** @brief 是否初始化 */
	[[nodiscard]] constexpr bool IsConfigured() const
	{
		return continuous || (calibrated && zero_raw < 8192U &&
							  min_deg > -180.0F && max_deg < 180.0F && min_deg < max_deg);
	}

	/**
	 * @brief 速度调试时端点禁止向外驱动，允许向行程内退回；不是位置保持环。
	 * @param raw 当前原始编码器值，调用方须先确认收到过合法反馈。
	 * @param feedback_reverse 是否反转反馈坐标。
	 * @param speed_dps 反馈坐标中的目标速度；越界或到端点时零目标也撤输出。
	 * @return 标定有效、输入合法且速度方向允许时返回true。
	 */
	[[nodiscard]] bool AllowsSpeed(uint16_t raw, bool feedback_reverse, float speed_dps) const
	{
		if (!IsConfigured() || raw >= 8192U || !std::isfinite(speed_dps))
			return false;
		if (continuous)
			return true;
		const float relative = static_cast<float>(MotorEncoderDelta(zero_raw, raw)) *
							   (360.0F / 8192.0F) * (feedback_reverse ? -1.0F : 1.0F);
		if (relative <= min_deg)
			return speed_dps > 0.0F;
		if (relative >= max_deg)
			return speed_dps < 0.0F;
		return true;
	}
};



/** @brief 记录角度目标及校准零点在连续角度坐标中的偏移。 */
class MotorAngleTarget
{
	public:
		/** @brief 清空目标；首次反馈或重新使能时必须重新同步。 */
		void Reset()
		{
			ready_           = false;
			target_deg_      = 0.0F;
			zero_offset_deg_ = 0.0F;
		}

		/**
		 * @brief 将目标对齐当前实测角度，避免启动时自行寻找零点。
		 * @param limits
		 * @return 校准有效且当前位置处于行程内时返回 true。
		 */
		bool Sync(const MotorAngleLimits& limits, float total_deg, uint16_t raw,
				  uint32_t                epoch, bool   feedback_reverse)
		{
			Reset();

			//是否限位没配置好或角度不是有限值
			if (!limits.IsConfigured() || !std::isfinite(total_deg))
				return false;

			if (!limits.continuous)
			{
				const float relative = static_cast<float>(MotorEncoderDelta(limits.zero_raw, raw)) *
									   (360.0F / 8192.0F) * (feedback_reverse ? -1.0F : 1.0F);
				if (relative < limits.min_deg || relative > limits.max_deg)
					return false;
				zero_offset_deg_ = total_deg - relative;
			}

			target_deg_ = total_deg;
			epoch_      = epoch;
			ready_      = true;
			return true;
		}

		/**
		 * @brief 按摇杆速度积分目标，并将非连续轴目标限制在实测边界内。
		 * @return 更新后的连续角度目标；调用前必须成功 Sync()。
		 */
		float Step(const MotorAngleLimits& limits, float stick, float max_rate_dps, float dt_s)
		{
			if (!ready_ || !std::isfinite(stick) || !std::isfinite(max_rate_dps) ||
				!std::isfinite(dt_s) || max_rate_dps < 0.0F || dt_s <= 0.0F)
				return target_deg_;

			if (stick > 1.0F) stick = 1.0F;
			if (stick < -1.0F) stick = -1.0F;
			float next = target_deg_ + stick * max_rate_dps * dt_s;
			if (!limits.continuous)
			{
				const float lower = zero_offset_deg_ + limits.min_deg;
				const float upper = zero_offset_deg_ + limits.max_deg;
				if (next < lower) next = lower;
				if (next > upper) next = upper;
			}
			target_deg_ = next;
			return target_deg_;
		}

		/** @brief 判断当前连续角度是否仍在标定行程内。 */
		[[nodiscard]] bool Contains(const MotorAngleLimits& limits, float total_deg) const
		{
			if (!ready_ || !std::isfinite(total_deg))
				return false;
			return limits.continuous ||
				   (total_deg >= zero_offset_deg_ + limits.min_deg &&
					total_deg <= zero_offset_deg_ + limits.max_deg);
		}

		[[nodiscard]] bool     Ready() const { return ready_; }
		[[nodiscard]] uint32_t Epoch() const { return epoch_; }
		[[nodiscard]] float    Target() const { return target_deg_; }

	private:
		float    target_deg_{};
		float    zero_offset_deg_{};
		uint32_t epoch_{};
		bool     ready_{};
};

#endif // INFANTRY_H723_ANGLE_TARGET_H
