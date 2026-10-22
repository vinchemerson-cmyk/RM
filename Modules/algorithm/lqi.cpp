/** @file lqi.cpp @brief 三状态单输入 LQI 的离散 Riccati 迭代与稳定性检查。 */
#include "lqi.h"

#include <algorithm>
#include <cmath>

namespace
{
	using Matrix = double[3][3]; //矩阵

	//行列式
	double Determinant(const Matrix& m)
	{
		return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
			   m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
			   m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
	}

	// 特征多项式经 Schur 递推判定根是否严格位于单位圆内，不依赖矩阵库。
	bool IsStable(const Matrix& m)
	{
		double c[4]{
			1.0, -(m[0][0] + m[1][1] + m[2][2]),
			m[0][0] * m[1][1] - m[0][1] * m[1][0] +
			m[0][0] * m[2][2] - m[0][2] * m[2][0] +
			m[1][1] * m[2][2] - m[1][2] * m[2][1],
			-Determinant(m)
		};
		for (int degree = 3; degree > 0; --degree)
		{
			if (!std::isfinite(c[degree]) || std::fabs(c[degree]) >= 1.0)
				return false;
			double       next[4]{};
			const double denominator = 1.0 - c[degree] * c[degree];
			for (int i = 0; i < degree; ++i)
			{
				next[i] = (c[i] - c[degree] * c[degree - i]) / denominator;
				if (!std::isfinite(next[i]))
					return false;
			}
			for (int i = 0; i < degree; ++i)
				c[i] = next[i];
		}
		return true;
	}

	// Pnext=Q+A'PA-A'PB*(R+B'PB)^-1*B'PA；输入只有一个电流通道。
	bool RiccatiStep(const Matrix& a, const double (&b)[3], const double (&q)[3],
					 double        r, const Matrix&  p, Matrix&            next, double (&gain)[3])
	{
		Matrix pa{};
		double pb[3]{};
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
			{
				pb[i] += p[i][j] * b[j];
				for (int k = 0; k < 3; ++k)
					pa[i][j] += p[i][k] * a[k][j];
			}
		double denominator = r;
		for (int i = 0; i < 3; ++i)
			denominator += b[i] * pb[i];
		if (!std::isfinite(denominator) || denominator <= 0.0)
			return false;
		double atpb[3]{};
		for (int i = 0; i < 3; ++i)
		{
			for (int k = 0; k < 3; ++k)
				atpb[i] += a[k][i] * pb[k];
			gain[i] = atpb[i] / denominator;
			if (!std::isfinite(gain[i]))
				return false;
		}
		for (int i = 0; i < 3; ++i)
			for (int j = i; j < 3; ++j)
			{
				double value = (i == j ? q[i] : 0.0) - atpb[i] * gain[j];
				for (int k = 0; k < 3; ++k)
					value += a[k][i] * pa[k][j];
				if (!std::isfinite(value))
					return false;
				next[i][j] = next[j][i] = value;
			}
		return true;
	}
}

LQI_Solve_Status LQI_Solve(const LQI_Model_s&  model, const LQI_Init_Config_s&     config,
						   LQI_Solve_Result_s& result, const LQI_Solver_Options_s& options)
{
	result = {};
	const double q[3]{config.q_angle, config.q_speed, config.q_integral};
	const double r = config.r_current;

	//满足以下任一条件则为false:
	//1:模型采样周期非有限
	//2:模型采样周期非正
	//3:配置采样周期非有限
	//4:配置采样周期非正
	//5:模型和配置采样周期不一致
	//6:r_current 非有限
	//7:r_current 非正
	//8:最大迭代次数为0
	//9:相对容差非有限
	//10:相对容差非正
	//11:绝对容差非有限
	//12:绝对容差为负
	//
	if (!std::isfinite(model.sample_period_s) || model.sample_period_s <= 0.0 ||
		!std::isfinite(config.sample_period_s) || config.sample_period_s <= 0.0F ||
		std::fabs(model.sample_period_s - config.sample_period_s) >
		1e-6 * model.sample_period_s ||
		!std::isfinite(r) || r <= 0.0 || options.max_iterations == 0U ||
		!std::isfinite(options.relative_tolerance) || options.relative_tolerance <= 0.0 ||
		!std::isfinite(options.absolute_tolerance) || options.absolute_tolerance < 0.0)
		return LQI_Solve_Status::InvalidInput;

	for (double i : q)
		if (!std::isfinite(i) || i < 0.0)
			return LQI_Solve_Status::InvalidInput;

	for (int i = 0; i < 2; ++i)
	{
		if (!std::isfinite(model.b[i]))
			return LQI_Solve_Status::InvalidInput;
		for (int j = 0; j < 2; ++j)
			if (!std::isfinite(model.a[i][j]))
				return LQI_Solve_Status::InvalidInput;
	}

	if (model.b[0] == 0.0 && model.b[1] == 0.0)
		return LQI_Solve_Status::InvalidInput;

	const Matrix a{
		{model.a[0][0], model.a[0][1], 0.0},
		{model.a[1][0], model.a[1][1], 0.0},
		{model.sample_period_s, 0.0, 1.0}
	};

	const double b[3]{model.b[0], model.b[1], 0.0};
	Matrix       p{{q[0], 0.0, 0.0}, {0.0, q[1], 0.0}, {0.0, 0.0, q[2]}};
	Matrix       next{};
	double       gain[3]{};

	for (uint32_t iteration = 0U; iteration < options.max_iterations; ++iteration)
	{
		result.iterations = iteration + 1U;
		if (!RiccatiStep(a, b, q, r, p, next, gain))
			return LQI_Solve_Status::NumericalFailure;
		double difference = 0.0, scale = 1.0;
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
			{
				difference = std::max(difference, std::fabs(next[i][j] - p[i][j]));
				scale      = std::max(scale, std::fabs(p[i][j]));
			}
		result.relative_residual = difference / scale;
		if (difference <= options.absolute_tolerance + options.relative_tolerance * scale)
		{
			// gain 对应当前 P；先按目标 float 精度量化，再检验闭环 A-BK。
			float  storedGain[3]{};
			Matrix closedLoop{};
			for (int j = 0; j < 3; ++j)
			{
				storedGain[j] = static_cast<float>(gain[j]);
				if (!std::isfinite(storedGain[j]))
					return LQI_Solve_Status::NumericalFailure;
				for (int i = 0; i < 3; ++i)
					closedLoop[i][j] = a[i][j] - b[i] * storedGain[j];
			}
			if (!IsStable(closedLoop))
				return LQI_Solve_Status::NonStabilizing;
			for (int i = 0; i < 3; ++i)
			{
				result.gain[i] = storedGain[i];
				for (int j = 0; j < 3; ++j)
					result.p[i][j] = p[i][j];
			}
			return LQI_Solve_Status::Success;
		}
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				p[i][j] = next[i][j];
	}

	return LQI_Solve_Status::NoConvergence;
}
