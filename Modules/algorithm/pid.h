/**
 * @file pid.h
 * @brief 与 HAL 和具体设备无关的离散 PID 控制器。
 *
 * 算法: u = Kp*e + Ki*∫e*dt + Kd*d(measurement)/dt
 *   - 微分作用于测量值而非误差, 目标值阶跃不会产生微分冲击, 适合电机角度环;
 *   - 积分带独立限幅 (integral_min/integral_max), 并采用条件积分抗饱和:
 *     输出饱和且误差与饱和同向时暂停积分, 退出饱和后自动恢复;
 *   - 单位由调用方约定: 电机角度环为 度/度每秒, 电流环为指令码,
 *     加热环为 摄氏度/占空比 (0~1)。
 *
 * 生命周期: 每个控制环一个实例。
 *   Configure() 校验参数并清空历史 (非法参数如 NaN、输出区间不含 0
 *   会配置失败, 由调用方决定是否停机); Reset() 清积分与微分历史;
 *   Disable() 停用并清状态, 再次使用前必须重新 Configure()。
 *
 * 失败安全: Update() 的 dt_s 必须为正 (单位秒); 任何非法输入
 * (未配置、NaN、dt<=0) 返回 0 并清空状态, 使控制量异常时输出收敛到零。
 * 输出与积分始终限幅在配置区间内, 输出区间必须包含 0。
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
	float output_min{};//输出最小值
	float output_max{};//输出最大值
	float integral_min{};//积分最小值
	float integral_max{};//积分最大值
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
		[[nodiscard]] float Output() const { return output_; }

		/** @brief 查询是否已载入合法参数。 */
		[[nodiscard]] bool IsConfigured() const { return configured_; }

	private:
		PID_Init_Config_s config_{};
		float             integral_{};
		float             last_measurement_{};
		float             output_{};
		bool              configured_{};
		bool              has_measurement_{};
};

#endif // INFANTRY_H723_PID_H
