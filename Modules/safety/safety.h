/**
 * @file safety.h
 * @brief 急停与输出使能状态管理。
 *
 * @note 触发方式: 左右拨杆同时拨到 DOWN。
 * @note 解除方式: 只要两个拨杆不再同时在 DOWN, 立即解除, 不需要任何额外操作。
 * @note 遥控掉线同样会撤销输出, 恢复收帧后自动恢复判定。
 * @note 上电时若两个拨杆不在 DOWN, 则处于放行状态。
 */

#ifndef INFANTRY_H723_SAFETY_H
#define INFANTRY_H723_SAFETY_H

#include <cstdint>

namespace safety
{
    /** @brief 判定结果需连续成立的最少周期数, 用于滤除拨杆抖动, 单位毫秒。 */
    static constexpr uint8_t DEBOUNCE_COUNT = 3U;

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
