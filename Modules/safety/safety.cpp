/**
 * @file safety.cpp
 * @brief 监控遥控拨杆与掉线状态, 维护输出使能标志。
 */

#include "safety.h"

#include "cmsis_os2.h"
#include "rc/rc.h"

namespace
{
    osThreadId_t taskHandle = nullptr;

    constexpr osThreadAttr_t taskAttrs = {
        .name       = "SafetyTask",
        .stack_size = 1024U,
        // 高于普通应用任务, 保证急停判定不被低的控制任务拖延。
        .priority   = osPriorityAboveNormal,
    };

    /** @brief 判定后的急停状态: 两个拨杆同时在 DOWN。 */
    volatile bool e_stopped = false;

    /** @brief 最近一次判定结果是否为遥控掉线。 */
    volatile bool offline = true;

    /** @brief 当前手势 (原始值, 未经去抖) 已连续成立的周期数。 */
    uint8_t gesture_cycles = 0U;

    /** @brief 去抖时跟踪的上一周期原始手势。 */
    bool last_gesture = false;

    /**
 * @brief 原始急停手势: 左右拨杆是否同时处于 DOWN。
 */
    bool RawBothDown()
    {
        return rc::RC.switch_left == rc::Left_Switch_e::DOWN &&
               rc::RC.switch_right == rc::Right_Switch_e::DOWN;
    }

    /**
 * @brief 对原始手势去抖, 滤除拨杆经过中间档时的抖动。
 * @return 去抖后的手势状态。
 * @note 只在状态跳变时重新计数, 因此去抖引入的延迟固定为 DEBOUNCE_COUNT 毫秒。
 */
    bool DebouncedBothDown()
    {
        const bool raw = RawBothDown();
        if (raw != last_gesture)
        {
            last_gesture   = raw;
            gesture_cycles = 0U;
            return e_stopped; // 状态尚未确认, 保持上一次判定。
        }

        if (gesture_cycles < safety::DEBOUNCE_COUNT)
            ++gesture_cycles;
        if (gesture_cycles < safety::DEBOUNCE_COUNT)
            return e_stopped;

        return raw;
    }

    /**
 * @brief 安全监控任务。
 * @note 急停与解除共用同一个判定结果, 因此只要拨杆离开 DOWN 组合就立即恢复输出。
 * @note 遥控掉线单独作为失效条件, 恢复收帧后自动重新判定。
 */
    void Task(void*)
    {
        for (;;)
        {
            offline = !rc::IsOnline();

            // 掉线时不允许输出; 拨杆手势按去抖后的结果决定是否急停。
            e_stopped = offline || DebouncedBothDown();

            osDelay(safety::TASK_PERIOD_MS);
        }
    }
} // namespace

bool safety::ShouldOutput()
{
    // 安全任务完成首次在线判定前保持关闭；掉线也立即失效。
    return !offline && !e_stopped && rc::IsOnline();
}

bool safety::IsEStopped()
{
    return e_stopped;
}

bool safety::BothDown()
{
    return RawBothDown();
}

void safety::Init()
{
    if (taskHandle != nullptr)
        return;
    taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
}
