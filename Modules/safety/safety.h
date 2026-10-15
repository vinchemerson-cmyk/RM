/**
 * @file safety.h
 * @brief 急停与输出使能状态管理。
 *
 * @note 触发方式: 左右拨杆同时拨到 DOWN。
 * @note 上电/失联恢复后需摇杆居中、双拨杆 DOWN 至少三个判定周期，然后离开双 DOWN。
 * @note 遥控掉线撤销输出和解锁状态，不会自动续发。
 * @note 双 DOWN 急停立即生效；三周期确认用于解锁手势。
 */

#ifndef INFANTRY_H723_SAFETY_H
#define INFANTRY_H723_SAFETY_H

#include <cstdint>

namespace safety
{
	/** @brief 解锁手势需连续成立的最少周期数。 */
	static constexpr uint8_t DEBOUNCE_COUNT = 1U;

	/** @brief 安全任务轮询周期, 单位毫秒。 */
	static constexpr uint32_t TASK_PERIOD_MS = 1U;

	/** @brief 当前是否允许电机输出; 所有执行器输出必须以此为总开关。 */
	bool ShouldOutput();

	/** @brief 当前是否处于急停状态 (两个拨杆同时在 DOWN)。 */
	bool IsEStopped();

	/** @brief 原始手势状态: 左右拨杆是否同时处于 DOWN, 未经去抖。 */
	bool BothDown();

	/** @brief 创建安全监控任务; 重复调用不重复创建。 */
	void Init();
} // namespace safety

#endif // INFANTRY_H723_SAFETY_H
