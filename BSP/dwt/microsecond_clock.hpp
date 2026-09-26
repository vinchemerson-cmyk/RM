#ifndef MICROSECOND_CLOCK_HPP
#define MICROSECOND_CLOCK_HPP

#include <cstdint>

namespace microsecondClock
{
    /**
     * @brief 启动基于 Cortex-M DWT CYCCNT 的连续微秒时钟。
     * @param coreClockHz Cortex-M7 内核时钟，单位 Hz。
     * @note 不会清零已经运行的 CYCCNT，避免干扰其他 DWT 使用者。
     */
    void Initialize(uint32_t coreClockHz);

    /**
     * @brief 获取自 Initialize() 起累计的 32 位微秒时间戳。
     * @return 微秒时间戳，约 71.6 分钟回卷一次。
     * @note 可在中断中调用；相邻时间戳应使用无符号减法处理回卷。
     */
    [[nodiscard]] uint32_t Now();
} // namespace microsecondClock

#endif /* MICROSECOND_CLOCK_HPP */
