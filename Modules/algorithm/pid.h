/**
 * @file pid.h
 * @brief 与 HAL 和具体设备无关的离散 PID 控制器。
 */

#ifndef INFANTRY_H723_PID_H
#define INFANTRY_H723_PID_H

/**
 * @brief PID 参数；时间单位统一为秒，输出范围由具体执行器决定。
 * @note 电机可配置双向输出，单向加热器应配置为 0 到 1。
 */
struct PID_Init_Config_s
{
    float kp{};
    float ki{};
    float kd{};
    float output_min{};
    float output_max{};
    float integral_min{};
    float integral_max{};
};

/**
 * @brief 独立保存一个控制环的积分、历史反馈和输出。
 * @note 微分作用于测量值，避免目标值突变造成微分冲击。每个电机环和加热环各用一个实例。
 */
class PIDInstance
{
public:
    /**
     * @brief 验证参数并清空控制器历史状态。
     * @return 参数合法时返回 true；非法配置不会启用控制器。
     */
    bool Configure(const PID_Init_Config_s& config);

    /** @brief 清空积分和微分历史；停机、掉线或切换模式时调用。 */
    void Reset();

    /** @brief 停用控制器并清空状态；再次使用前必须重新配置。 */
    void Disable();

    /**
     * @brief 计算一次 PID 输出，结果限制在配置范围内。
     * @param target 目标值，与 measurement 使用相同单位。
     * @param measurement 本次反馈值。
     * @param dt_s 距离上次调用的时间，单位秒且必须大于零。
     * @return 执行器输出；配置或输入无效时返回零并清空状态。
     */
    float Update(float target, float measurement, float dt_s);

    /** @brief 返回上一次计算的输出。 */
    float Output() const { return output_; }

    /** @brief 查询是否已载入合法参数。 */
    bool IsConfigured() const { return configured_; }

private:
    PID_Init_Config_s config_{};
    float integral_{};
    float last_measurement_{};
    float output_{};
    bool configured_{};
    bool has_measurement_{};
};

#endif // INFANTRY_H723_PID_H
