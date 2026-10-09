#include <array>
#include <cassert>
#include <cmath>
#include <limits>

#include "imu/AttitudeEstimator.h"

namespace
{
	constexpr float kGravity          = 9.80665F;
	constexpr float kDegreesToRadians = 0.017453292519943295F;

	bool Near(float actual, float expected, float tolerance = 0.3F)
	{
		return std::fabs(actual - expected) < tolerance;
	}
}

int main()
{
	imu::AttitudeEstimator     estimator{};
	const std::array<float, 3> level{0.0F, 0.0F, kGravity};
	const std::array<float, 3> still{};

	auto attitude = estimator.Update(level, still, 0.0F);
	assert(attitude.valid);
	assert(Near(attitude.roll_deg, 0.0F));
	assert(Near(attitude.pitch_deg, 0.0F));

	estimator.Reset();
	const float roll = 30.0F * kDegreesToRadians;
	attitude         = estimator.Update({
											0.0F, kGravity * std::sin(roll),
											kGravity * std::cos(roll)
										}, still, 0.0F);
	assert(attitude.valid && Near(attitude.roll_deg, 30.0F));

	estimator.Reset();
	const float pitch = 20.0F * kDegreesToRadians;
	attitude          = estimator.Update({
											 -kGravity * std::sin(pitch), 0.0F,
											 kGravity * std::cos(pitch)
										 }, still, 0.0F);
	assert(attitude.valid && Near(attitude.pitch_deg, 20.0F));

	// 初始水平后缓慢倾斜，加速度修正应让 Roll 逐步追上实测重力方向。
	estimator.Reset();
	(void)estimator.Update(level, still, 0.0F);
	for (int index = 0; index < 1500; ++index)
		attitude = estimator.Update({
										0.0F, kGravity * std::sin(roll),
										kGravity * std::cos(roll)
									}, still, 0.002F);
	assert(attitude.valid && attitude.roll_deg > 28.0F && attitude.roll_deg < 31.0F);

	estimator.Reset();
	(void)estimator.Update(level, still, 0.0F);
	const std::array<float, 3> yawRate{0.0F, 0.0F, 90.0F * kDegreesToRadians};
	for (int index = 0; index < 500; ++index)
		attitude = estimator.Update(level, yawRate, 0.002F);
	assert(attitude.valid && Near(attitude.yaw_deg, 90.0F));

	// 机体线性加速时，不能把加速度方向直接当作重力方向修正姿态。
	estimator.Reset();
	(void)estimator.Update(level, still, 0.0F);
	for (int index = 0; index < 500; ++index)
		attitude = estimator.Update({2.0F * kGravity, 0.0F, kGravity},
									still, 0.002F);
	assert(attitude.valid && Near(attitude.pitch_deg, 0.0F));

	attitude = estimator.Update({std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F},
								still, 0.002F);
	assert(!attitude.valid);
	attitude = estimator.Update(level, still, 0.0F);
	assert(attitude.valid && Near(attitude.yaw_deg, 0.0F));
	return 0;
}
