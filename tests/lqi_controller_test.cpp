/** @file lqi_controller_test.cpp @brief 验证求解器与实时控制器的离散时序一致性。 */
#include <cassert>
#include <cmath>
#include <limits>
#include "algorithm/lqi.h"
#include "algorithm/lqi_controller.h"

int main()
{
    LQI_Model_s model{{{1.0, 1.0}, {0.0, 1.0}}, {0.5, 1.0}, 1.0};
    LQI_Init_Config_s config{};
    config.sample_period_s = 1.0F;
    config.output_min_a = -100.0F; config.output_max_a = 100.0F;
    config.integral_min_deg_s = -100.0F; config.integral_max_deg_s = 100.0F;
    LQI_Solve_Result_s solved{};
    assert(LQI_Solve(model, config, solved) == LQI_Solve_Status::Success);
    config.k_angle = solved.gain[0]; config.k_speed = solved.gain[1];
    config.k_integral = solved.gain[2];
    LQIController control{};
    assert(control.Configure(config));
    double angle = 1.0, speed = 0.0, integral = 0.0;
    for (int i = 0; i < 100; ++i)
    {
        const double expected = -config.k_angle * angle - config.k_speed * speed -
                                config.k_integral * integral;
        const float current = control.Update(0.0F, static_cast<float>(angle),
                                              static_cast<float>(speed), 1.0F);
        assert(std::fabs(current - expected) < 1e-6);
        integral += angle;
        angle += speed + 0.5 * current;
        speed += current;
    }
    assert(std::fabs(angle) < 1e-6 && std::fabs(speed) < 1e-6);
    assert(std::fabs(control.Integral()) < 1e-6);
    assert(control.UpdateRecoverySpeed(10.0F, 0.0F, 1.0F) > 0.0F);
    assert(control.Integral() == 0.0F);
    assert(control.UpdateRecoverySpeed(-10.0F, 0.0F, 1.0F) < 0.0F);
    assert(control.Integral() == 0.0F);
    // 单独验证积分时序、饱和阻止累积，以及反向卸载。
    config.k_angle = config.k_speed = 0.0F; config.k_integral = 1.0F;
    config.output_min_a = -3.0F; config.output_max_a = 3.0F;
    assert(control.Configure(config));
    assert(control.Update(1.0F, 0.0F, 0.0F, 1.0F) == 0.0F);
    for (int i = 0; i < 100; ++i)
        control.Update(1.0F, 0.0F, 0.0F, 1.0F);
    assert(control.Integral() == -3.0F && control.Output() == 3.0F);
    assert(control.Update(0.0F, 1.0F, 0.0F, 1.0F) == 3.0F);
    assert(control.Integral() == -2.0F);
    assert(control.Update(0.0F, 0.0F, 0.0F, 1.0F) == 2.0F);
    control.Reset();
    assert(control.IsConfigured() && control.Output() == 0.0F && control.Integral() == 0.0F);
    assert(control.Update(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 1.0F) == 0.0F);
    assert(control.Update(1.0F, 0.0F, 0.0F, 0.0F) == 0.0F);
    const float large = std::numeric_limits<float>::max();
    assert(control.Update(-large, large, 0.0F, 1.0F) == 0.0F);
    config.k_angle = config.k_speed = config.k_integral = 0.0F;
    assert(!control.Configure(config));
    control.Disable();
    assert(!control.IsConfigured() && control.Update(1.0F, 0.0F, 0.0F, 1.0F) == 0.0F);
    return 0;
}
