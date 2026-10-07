/** @file lqi_controller.cpp @brief LQI 状态反馈、积分限幅与抗饱和。 */
#include "lqi_controller.h"
#include <cmath>

namespace
{
    float Clamp(float value, float minimum, float maximum)
    {
        return value < minimum ? minimum : (value > maximum ? maximum : value);
    }
}

bool LQIController::Configure(const LQI_Init_Config_s& config)
{
    configured_ = false;
    Reset();
    const float values[]{config.k_angle, config.k_speed, config.k_integral,
                         config.sample_period_s, config.output_min_a, config.output_max_a,
                         config.integral_min_deg_s, config.integral_max_deg_s};
    for (const float value : values)
        if (!std::isfinite(value))
            return false;
    if ((config.k_angle == 0.0F && config.k_speed == 0.0F && config.k_integral == 0.0F) ||
        config.sample_period_s <= 0.0F || config.output_min_a >= config.output_max_a ||
        config.output_min_a > 0.0F || config.output_max_a < 0.0F ||
        config.integral_min_deg_s > config.integral_max_deg_s ||
        config.integral_min_deg_s > 0.0F || config.integral_max_deg_s < 0.0F)
        return false;
    config_ = config;
    configured_ = true;
    return true;
}

void LQIController::Reset()
{
    integral_deg_s_ = output_a_ = 0.0F;
    position_current_a_ = speed_current_a_ = 0.0F;
}

void LQIController::Disable()
{
    configured_ = false;
    Reset();
}

float LQIController::Update(float target_deg, float actual_deg, float actual_speed_dps, float dt_s)
{
    if (!configured_ || !std::isfinite(target_deg) || !std::isfinite(actual_deg) ||
        !std::isfinite(actual_speed_dps) || !std::isfinite(dt_s) || dt_s <= 0.0F)
    {
        Reset();
        return 0.0F;
    }
    const float error = actual_deg - target_deg;
    const float feedback = -config_.k_angle * error - config_.k_speed * actual_speed_dps;
    const float nextIntegral = integral_deg_s_ + error * dt_s;
    if (!std::isfinite(error) || !std::isfinite(feedback) || !std::isfinite(nextIntegral))
    {
        Reset();
        return 0.0F;
    }
    const float candidate = Clamp(nextIntegral, config_.integral_min_deg_s, config_.integral_max_deg_s);
    // DARE 使用 z[k+1]=z[k]+Ts*e[k]：本周期电流须使用更新前的 z[k]。
    const float output = feedback - config_.k_integral * integral_deg_s_;
    const float candidateOutput = feedback - config_.k_integral * candidate;
    const float change = -config_.k_integral * (candidate - integral_deg_s_);
    if (!std::isfinite(output) || !std::isfinite(candidateOutput) || !std::isfinite(change))
    {
        Reset();
        return 0.0F;
    }
    if (!((candidateOutput > config_.output_max_a && change > 0.0F) ||
          (candidateOutput < config_.output_min_a && change < 0.0F)))
        integral_deg_s_ = candidate;
    output_a_ = Clamp(output, config_.output_min_a, config_.output_max_a);
    position_current_a_ = -config_.k_angle * error;
    speed_current_a_ = -config_.k_speed * actual_speed_dps;
    return output_a_;
}

float LQIController::UpdateRecoverySpeed(float target_dps, float actual_dps, float dt_s)
{
    Reset();
    return Update(0.0F, 0.0F, actual_dps - target_dps, dt_s);
}
