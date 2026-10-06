/**
 * @file lqi_config.h
 * @brief 单轴角度 LQI 的离线设计参数与状态反馈增益配置。
 */

#ifndef INFANTRY_H723_LQI_CONFIG_H
#define INFANTRY_H723_LQI_CONFIG_H

/**
 * @brief LQI 参数；角度使用度，速度使用度每秒，输出为电流安培。
 * @note 约定 e = 实际角度 - 目标角度，积分状态 z = integral(e * dt)。
 *       定点保持时控制律为 I = -k_angle*e - k_speed*实际速度 - k_integral*z。
 *       Q = diag(q_angle, q_speed, q_integral)，R = r_current，交叉权重 N = 0。
 *       Q/R 仅记录离线设计权重，不能代替增益；需根据机构模型求解 K。
 *       求解模型的积分状态也必须使用实际角度减目标角度，不能混用相反符号。
 *       本文件仅定义配置，不实现求解器、积分更新、抗饱和或控制器切换。
 */
struct LQI_Init_Config_s
{
    float k_angle{};    ///< 角度误差反馈增益，单位 A/度。
    float k_speed{};    ///< 实际速度反馈增益，单位 A/(度/秒)。
    float k_integral{}; ///< 角度误差积分反馈增益，单位 A/(度*秒)。

    float q_angle{1.0F};    ///< 角度误差权重，非负；占位值需按机构整定。
    float q_speed{1.0F};    ///< 实际速度权重，非负。
    float q_integral{1.0F}; ///< 角度误差积分权重，非负。
    float r_current{1.0F};  ///< 电流输出权重，必须大于零。

    float sample_period_s{};      ///< 离线离散模型与控制周期，单位秒。
    float output_min_a{};         ///< 电流输出下限，单位 A。
    float output_max_a{};         ///< 电流输出上限，单位 A。
    float integral_min_deg_s{};   ///< 积分状态下限，单位度*秒。
    float integral_max_deg_s{};   ///< 积分状态上限，单位度*秒。
};

#endif // INFANTRY_H723_LQI_CONFIG_H
