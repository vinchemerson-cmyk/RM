#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include "algorithm/clamp.h"

static_assert(controlMath::Clamp(2.0F, -1.0F, 1.0F) == 1.0F);
static_assert(controlMath::Clamp(-2.0F, -1.0F, 1.0F) == -1.0F);
static_assert(controlMath::Clamp(0.5F, -1.0F, 1.0F) == 0.5F);
static_assert(controlMath::Clamp(1.0F, 0.0F, 0.0F) == 0.0F);

int main()
{
    assert(controlMath::Clamp(-1.0F, -1.0F, 1.0F) == -1.0F);
    assert(controlMath::Clamp(1.0F, -1.0F, 1.0F) == 1.0F);
    assert(std::isnan(controlMath::Clamp(std::numeric_limits<float>::quiet_NaN(), -1.0F, 1.0F)));
    assert(controlMath::Clamp(std::numeric_limits<float>::infinity(), -1.0F, 1.0F) == 1.0F);
    assert(controlMath::Clamp(-std::numeric_limits<float>::infinity(), -1.0F, 1.0F) == -1.0F);
    assert(static_cast<int16_t>(controlMath::Clamp(-12.9F, -100.0F, 100.0F)) == -12);
    assert(static_cast<int16_t>(controlMath::Clamp(20000.0F, -16384.0F, 16384.0F)) == 16384);
}
