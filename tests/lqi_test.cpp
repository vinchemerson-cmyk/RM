/** @file lqi_test.cpp @brief 与独立 DARE 解对照，验证 LQI 增益和失败路径。 */
#include <cassert>
#include <cmath>
#include <limits>
#include "algorithm/lqi.h"

int main()
{
	LQI_Init_Config_s config{};
	config.sample_period_s = 1.0F;
	LQI_Model_s        model{{{1.0, 1.0}, {0.0, 1.0}}, {0.5, 1.0}, 1.0};
	LQI_Solve_Result_s result{};
	assert(LQI_Solve(model, config, result) == LQI_Solve_Status::Success);
	// SciPy solve_discrete_are 的独立基准：Q=I，R=1，Ts=1 的双积分机构。
	const double expected[3]{1.11803398874990, 1.30901699437495, 0.30901699437495};
	for (int i = 0; i < 3; ++i)
		assert(std::fabs(result.gain[i] - expected[i]) < 1e-6);
	assert(result.relative_residual < 1e-9);
	// 独立验证返回 P 的 DARE 残差，不复用求解器中的矩阵运算函数。
	const double a[3][3]{{1, 1, 0}, {0, 1, 0}, {1, 0, 1}};
	const double b[3]{0.5, 1, 0};
	double       denominator = 1.0;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			denominator += b[i] * result.p[i][j] * b[j];
	double numerator[3]{};
	for (int j = 0; j < 3; ++j)
		for (int i = 0; i < 3; ++i)
			for (int k = 0; k < 3; ++k)
				numerator[j] += b[i] * result.p[i][k] * a[k][j];
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
		{
			double residual = (i == j ? 1.0 : 0.0) - result.p[i][j] -
							  numerator[i] * numerator[j] / denominator;
			for (int k = 0; k < 3; ++k)
				for (int l = 0; l < 3; ++l)
					residual += a[k][i] * result.p[k][l] * a[l][j];
			assert(std::fabs(residual) < 1e-8);
		}
	// 1ms 周期、100度/s²/A 的纯模拟模型；不是实车模型。
	model                  = {{{1.0, 0.001}, {0.0, 1.0}}, {0.00005, 0.1}, 0.001};
	config.sample_period_s = 0.001F;
	assert(LQI_Solve(model, config, result) == LQI_Solve_Status::Success);
	const double fastExpected[3]{1.65660206947948, 0.96769141462869, 0.95042813569472};
	for (int i = 0; i < 3; ++i)
		assert(std::fabs(result.gain[i] - fastExpected[i]) < 1e-5);
	// 带阻尼和恢复力的离散模型，非等权 Q/R；基准来自 SciPy DARE。
	const auto fastModel   = model;
	const auto fastConfig  = config;
	model                  = {{{1.0, 0.02}, {-0.1, 0.98}}, {0.0002, 0.02}, 0.02};
	config.sample_period_s = 0.02F;
	config.q_angle         = 4.0F;
	config.q_speed         = 0.3F;
	config.q_integral      = 2.0F;
	config.r_current       = 0.7F;
	assert(LQI_Solve(model, config, result) == LQI_Solve_Status::Success);
	const double weightedExpected[3]{1.10561776, 0.93827180, 1.67425396};
	for (int i = 0; i < 3; ++i)
		assert(std::fabs(result.gain[i] - weightedExpected[i]) < 1e-6);
	model.b[0] = -model.b[0];
	model.b[1] = -model.b[1];
	assert(LQI_Solve(model, config, result) == LQI_Solve_Status::Success);
	for (int i = 0; i < 3; ++i)
		assert(std::fabs(result.gain[i] + weightedExpected[i]) < 1e-6);
	model  = fastModel;
	config = fastConfig;
	LQI_Solver_Options_s options{};
	options.max_iterations = 1U;
	assert(LQI_Solve(model, config, result, options) == LQI_Solve_Status::NoConvergence);
	assert(result.gain[0] == 0.0F && result.gain[1] == 0.0F && result.gain[2] == 0.0F);
	auto invalid      = config;
	invalid.r_current = 0.0F;
	assert(LQI_Solve(model, invalid, result) == LQI_Solve_Status::InvalidInput);
	invalid         = config;
	invalid.q_speed = -1.0F;
	assert(LQI_Solve(model, invalid, result) == LQI_Solve_Status::InvalidInput);
	invalid                 = config;
	invalid.sample_period_s = 0.01F;
	assert(LQI_Solve(model, invalid, result) == LQI_Solve_Status::InvalidInput);
	auto badModel    = model;
	badModel.a[0][0] = std::numeric_limits<double>::quiet_NaN();
	assert(LQI_Solve(badModel, config, result) == LQI_Solve_Status::InvalidInput);
	badModel      = model;
	badModel.b[0] = badModel.b[1] = 0.0;
	assert(LQI_Solve(badModel, config, result) == LQI_Solve_Status::InvalidInput);
	// 无状态权重会遗留单位圆上的积分极点，不能作为稳定 LQI 返回。
	invalid         = config;
	invalid.q_angle = invalid.q_speed = invalid.q_integral = 0.0F;
	assert(LQI_Solve(model, invalid, result) == LQI_Solve_Status::NonStabilizing);
	assert(result.gain[0] == 0.0F && result.p[0][0] == 0.0);
	invalid            = config;
	invalid.q_integral = 0.0F;
	assert(LQI_Solve(model, invalid, result) == LQI_Solve_Status::NonStabilizing);
	// 速度状态不可控且不稳定：达到迭代上限时不得返回可用 K。
	badModel               = {{{1.0, 0.0}, {0.0, 1.1}}, {0.1, 0.0}, 0.001};
	options.max_iterations = 100U;
	assert(LQI_Solve(badModel, config, result, options) == LQI_Solve_Status::NoConvergence);
	badModel.a[1][1] = 1e200;
	assert(LQI_Solve(badModel, config, result) == LQI_Solve_Status::NumericalFailure);
	return 0;
}
