/** @file lqi_controller.h @brief 使用已求解 K 的单轴 LQI 实时电流计算。 */
#ifndef INFANTRY_H723_LQI_CONTROLLER_H
#define INFANTRY_H723_LQI_CONTROLLER_H

#include "lqi_config.h"

/** @brief 每轴独立维护误差积分；不在实时控制周期内求解 K。 */
class LQIController
{
public:
    /** @brief 装载求解后的增益并清状态；非法配置或全零 K 返回 false。 */
    bool Configure(const LQI_Init_Config_s& config);
    void Reset();
    void Disable();

    /**
     * @brief 按 I=-K*[实际角度-目标角度,实际速度,误差积分] 计算电流 A。
     * @note 角度必须采用同一连续坐标，dt_s 使用实际控制周期。
     *       本周期使用 z[k] 计算输出，再更新 z[k+1]，与 DARE 积分模型一致。
     *       包含积分限幅、条件积分抗饱和；非法输入或溢出清状态并返回零。
     */
    float Update(float target_deg, float actual_deg, float actual_speed_dps, float dt_s);

    /** @brief 端点退回使用 K_speed 对速度误差反馈；不保持位置，清除积分，输出 A。 */
    float UpdateRecoverySpeed(float target_dps, float actual_dps, float dt_s);

    [[nodiscard]] bool IsConfigured() const { return configured_; }
    [[nodiscard]] float Output() const { return output_a_; }
    [[nodiscard]] float Integral() const { return integral_deg_s_; }
    [[nodiscard]] float PositionCurrent() const { return position_current_a_; }
    [[nodiscard]] float SpeedCurrent() const { return speed_current_a_; }

private:
    LQI_Init_Config_s config_{};
    float integral_deg_s_{};
    float output_a_{};
    float position_current_a_{};
    float speed_current_a_{};
    bool configured_{};
};

#endif // INFANTRY_H723_LQI_CONTROLLER_H
