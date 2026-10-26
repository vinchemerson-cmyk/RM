//
// Created by 19923 on 2026/10/23.
//

#ifndef INFANTRY_H723_SPEED_TARGET_H
#define INFANTRY_H723_SPEED_TARGET_H
#include <cmath>

/**
 * @brief 单轴速度目标的限速/斜坡参数；这是目标整形，不是PI增益或电流上限。
 * @note 该结构成员没有默认值；使用MotorSpeedTargetLimits limits{}可清零，但随后仍须填写正数。
 *       目标、速度上限必须属于同一轴/同一单位层；应用层建议使用输出轴°/s。
 */
struct MotorSpeedTargetLimits
{
	float max_speed_dps;     // 目标速度绝对值上限，°/s，正反方向共用同一上限。
	float acceleration_dps2; // 增大速度幅值时的最大变化率，°/s²。
	float dcceleration_dps2; // 降低幅值/反向先退至零时的最大变化率，°/s²；保留现有成员拼写。

	/** @brief 三项都必须有限且严格大于0，避免NaN、无穷大、零上限进入斜坡计算。 */
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

/**
 * @brief 保存上一次速度目标，每周期把新指令平滑推进到可执行目标。
 * @note 不读取实际转速、不控制电流、不发送CAN；目标速度不代表电机实际已达到该速度。
 *       当前底盘WheelVelocityPlanner已在车体层加斜坡，不宜不加分析再逐轮叠加本类。
 */
class MotorSpeedTarget
{
	public:
		/** @brief 立即清零目标；不经过减速斜坡，也不会自动向驱动发停止指令。 */
		void Reset()
		{
			target_dps_ = 0.0F;
		}

		/**
		 * @brief 更新一次速度目标，返回经过速度及加减速约束的指令。
		 * @param requested_dps 希望达到的有符号角速度，°/s，不是编码器测量值。
		 * @param limits 速度/加减速上限，由调用方保存，可在每次调用时传入。
		 * @param dt_s 实际更新间隔，单位秒，必须大于0；任务层另行检查周期是否超时。
		 * @return 最新目标速度，°/s；参数异常时清零目标并返回0，不保留旧速度。
		 */
		float Step(float requested_dps,
			const MotorSpeedTargetLimits& limits,
			float dt_s)
		{
			if (!limits.IsValid() ||
			!std::isfinite(requested_dps) ||
			!std::isfinite(dt_s) || dt_s <= 0.0F)//输入合法性检测
			{
				Reset(); // 输入异常立即撤销旧目标，不沿着旧斜坡继续运动。
				return target_dps_;
			}

			// 正负方向对称限速；这里限制的是最终目标幅值，随后仍要经过斜坡约束。
			if (requested_dps > limits.max_speed_dps)
				requested_dps = limits.max_speed_dps;
			else if (requested_dps < -limits.max_speed_dps)
				requested_dps = -limits.max_speed_dps;

			// 反向时先减速到零，下一周期再向相反方向加速。
			const bool reversing =
				(target_dps_ > 0.0F && requested_dps < 0.0F) ||
				(target_dps_ < 0.0F && requested_dps > 0.0F);

			// 若正负号相反，本周期目标先设0；只有后续周期已到0，才开始向新方向加速。
			const float goal = reversing ? 0.0F : requested_dps;
			// 比较幅值而不是带符号大小：从-10到-20同样是加速，从-20到-10则是减速。
			const bool accelerating =
				std::fabs(goal) > std::fabs(target_dps_);//std::fabs求浮点数绝对值

			// 加速和减速使用不同配置，可独立设置起步柔和程度与制动速度。
			const float rate = accelerating
				? limits.acceleration_dps2
				: limits.dcceleration_dps2;

			const float max_change = rate * dt_s; // °/s²乘s，得到本周期允许变化的速度量，°/s。
			float change = goal - target_dps_;    // 目标相对上次已生成指令的差，不是电机的速度误差。

			// 差值已经在范围内就直接到达goal；超过范围则本周期只推进max_change。
			if (change > max_change)
				change = max_change;
			else if (change < -max_change)
				change = -max_change;

			target_dps_ += change; // 保存斜坡状态，下周期从这里继续，不每次重新从0起步。
			return target_dps_;
		}

		/** @brief 只读出上次生成的目标，不推进时间、不重新计算斜坡。 */
		[[nodiscard]] float Target() const
		{
			return target_dps_;
		}

	private:
		float target_dps_{}; // 斜坡后的目标速度，°/s；默认0，不是编码器反馈。
};


#endif //INFANTRY_H723_SPEED_TARGET_H
