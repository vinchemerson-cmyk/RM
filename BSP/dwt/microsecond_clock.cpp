#include "microsecond_clock.hpp"

extern "C"
{
#include "main.h"
}

namespace
{
    constexpr uint32_t HERTZ_PER_MEGAHERTZ = 1000000U;

    uint32_t cyclesPerMicrosecond{1U};
    uint32_t lastCycleCount{};
    uint32_t remainingCycles{};
    uint32_t accumulatedMicroseconds{};
    bool initialized{};
}

void microsecondClock::Initialize(const uint32_t coreClockHz)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cyclesPerMicrosecond = coreClockHz / HERTZ_PER_MEGAHERTZ;
    if (cyclesPerMicrosecond == 0U)
    {
        cyclesPerMicrosecond = 1U;
    }

    lastCycleCount = DWT->CYCCNT;
    remainingCycles = 0U;
    accumulatedMicroseconds = 0U;
    initialized = true;

    __set_PRIMASK(primask);
}

uint32_t microsecondClock::Now()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (!initialized)
    {
        __set_PRIMASK(primask);
        return 0U;
    }

    const uint32_t currentCycleCount = DWT->CYCCNT;
    const uint32_t elapsedCycles = currentCycleCount - lastCycleCount;
    lastCycleCount = currentCycleCount;

    const uint32_t completeMicroseconds = elapsedCycles / cyclesPerMicrosecond;
    const uint32_t newRemainingCycles = elapsedCycles % cyclesPerMicrosecond;

    accumulatedMicroseconds += completeMicroseconds;
    remainingCycles += newRemainingCycles;
    if (remainingCycles >= cyclesPerMicrosecond)
    {
        accumulatedMicroseconds += remainingCycles / cyclesPerMicrosecond;
        remainingCycles %= cyclesPerMicrosecond;
    }

    const uint32_t timestampUs = accumulatedMicroseconds;
    __set_PRIMASK(primask);
    return timestampUs;
}
