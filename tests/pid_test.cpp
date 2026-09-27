#include "algorithm/pid.h"

#include <cassert>
#include <cmath>

static bool Near(float a, float b)
{
    return std::fabs(a - b) < 0.0001F;
}

int main()
{
    PIDInstance motor;
    const PID_Init_Config_s motorConfig{2.0F, 1.0F, 0.5F, -10.0F, 10.0F, -5.0F, 5.0F};
    assert(motor.Configure(motorConfig));
    assert(Near(motor.Update(2.0F, 0.0F, 1.0F), 6.0F));
    assert(Near(motor.Update(2.0F, 1.0F, 1.0F), 4.5F));
    motor.Reset();
    assert(Near(motor.Update(2.0F, 0.0F, 1.0F), 6.0F));

    PIDInstance heater;
    const PID_Init_Config_s heaterConfig{0.1F, 0.2F, 0.0F, 0.0F, 1.0F, 0.0F, 1.0F};
    assert(heater.Configure(heaterConfig));
    assert(Near(heater.Update(50.0F, 20.0F, 1.0F), 1.0F));
    assert(Near(heater.Update(50.0F, 49.0F, 1.0F), 0.3F));
    assert(Near(heater.Update(50.0F, 55.0F, 1.0F), 0.0F));
    heater.Disable();
    assert(!heater.IsConfigured());
    assert(Near(heater.Update(50.0F, 20.0F, 1.0F), 0.0F));

    PIDInstance invalid;
    assert(!invalid.Configure(PID_Init_Config_s{1.0F, 0.0F, 0.0F, 1.0F, 2.0F, 0.0F, 1.0F}));
    assert(invalid.Configure(motorConfig));
    assert(Near(invalid.Update(1.0F, 0.0F, 0.0F), 0.0F));
    assert(Near(invalid.Update(1.0F, 0.0F, 1.0F), 3.0F));
    assert(Near(invalid.Update(NAN, 0.0F, 1.0F), 0.0F));
}
