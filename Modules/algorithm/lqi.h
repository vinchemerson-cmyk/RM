/** @file lqi.h @brief 由两状态离散机构模型与 Q/R 求解单轴 LQI 增益。 */
#ifndef INFANTRY_H723_LQI_H
#define INFANTRY_H723_LQI_H

#include <cstdint>
#include "lqi_config.h"

/**
 * @brief 离散机构模型 x[k+1] = A*x[k] + B*I[k]。
 * @note x=[角度(度), 速度(度/秒)]，输入 I 为 A；必须填入已辨识且离散化的模型。
 *       求解器增加 z[k+1]=z[k]+Ts*(实际角度-目标角度)，不做连续模型离散化。
 */
struct LQI_Model_s
{
	double a[2][2]{};
	double b[2]{};
	double sample_period_s{};
};

/** @brief Riccati 迭代参数；小采样周期的积分状态可能需要较多迭代。 */
struct LQI_Solver_Options_s
{
	uint32_t max_iterations{50000U};  //最大迭代次数
	double   relative_tolerance{1e-10};//相对收敛容差
	double   absolute_tolerance{1e-12};//绝对收敛容差
};

enum class LQI_Solve_Status
{
	Success,          //成功
	InvalidInput,     //无效输入
	NumericalFailure, //数值故障
	NoConvergence,    //不收敛
	NonStabilizing    //非稳定化
};

/** @brief 成功时的 K 与 Riccati 解；失败时 K/P 保持全零，保留迭代诊断。 */
struct LQI_Solve_Result_s
{
	float    gain[3]{}; ///< [k_angle, k_speed, k_integral]，I=-K*[角度误差,速度,积分]。
	double   p[3][3]{};
	uint32_t iterations{};        //迭代次数
	double   relative_residual{}; //相对残差
};

/**
 * @brief 求解离散 LQI，Q 为配置的三项对角权重，R 为 r_current，N=0。
 *
 * @param model 离散机构模型 x[k+1] = A*x[k] + B*I[k]
 * @param config 角度使用度，速度使用度每秒，输出为电流安培
 * @param result 成功时的 K 与 Riccati 解；失败时 K/P 保持全零，保留迭代诊断
 * @param options Riccati 迭代参数；小采样周期的积分状态可能需要较多迭代。
 *
 * @note 不使用配置中已有的 K 或输出/积分限幅；限幅不参与无约束 Riccati 方程。
 *       模型和配置的采样周期必须相同。内部使用 double，K 转为 float 后再检查
 *       闭环是否严格位于单位圆内；不收敛或不稳定时不返回可用增益。
 *       属于初始化/离线计算，不能放进 1kHz 电机控制任务；不自动启用 LQI。
 */
LQI_Solve_Status LQI_Solve(const LQI_Model_s&          model, const LQI_Init_Config_s& config,
						   LQI_Solve_Result_s&         result,
						   const LQI_Solver_Options_s& options = {});

#endif // INFANTRY_H723_LQI_H
