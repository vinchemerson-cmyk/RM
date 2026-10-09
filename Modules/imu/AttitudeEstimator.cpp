/**
 * @file AttitudeEstimator.cpp
 * @brief BMI088 六轴姿态的四元数互补估计实现。
 */

#include "AttitudeEstimator.h"

#include <cmath>

namespace
{
	constexpr float kGravity             = 9.79140F;
	constexpr float kRadToDeg            = 57.29577951308232F;
	constexpr float kAccelCorrectionGain = 2.0F;
	constexpr float kMinimumGravity      = 0.7F * kGravity;
	constexpr float kMaximumGravity      = 1.3F * kGravity;

	bool FiniteAxes(const std::array<float, 3>& axes)
	{
		return std::isfinite(axes[0]) && std::isfinite(axes[1]) &&
			   std::isfinite(axes[2]);
	}
}

void imu::AttitudeEstimator::Reset()
{
	quaternion_  = {1.0F, 0.0F, 0.0F, 0.0F};
	initialized_ = false;
}

void imu::AttitudeEstimator::Initialize(const std::array<float, 3>& acceleration)
{
	const float roll  = std::atan2(acceleration[1], acceleration[2]);
	const float pitch = std::atan2(-acceleration[0],
								   std::hypot(acceleration[1], acceleration[2]));
	const float cr = std::cos(roll * 0.5F);
	const float sr = std::sin(roll * 0.5F);
	const float cp = std::cos(pitch * 0.5F);
	const float sp = std::sin(pitch * 0.5F);
	// 仅靠重力无法确定绝对航向，因此初始 yaw 设为零。
	quaternion_    = {cr * cp, sr * cp, cr * sp, -sr * sp};
	initialized_   = true;
}

imu::AttitudeSample imu::AttitudeEstimator::EulerAngles() const
{
	const float w            = quaternion_[0];
	const float x            = quaternion_[1];
	const float y            = quaternion_[2];
	const float z            = quaternion_[3];
	const float sinPitch     = 2.0F * (w * y - z * x);
	const float clampedPitch = sinPitch < -1.0F ? -1.0F : (sinPitch > 1.0F ? 1.0F : sinPitch);

	AttitudeSample result{};
	result.roll_deg = std::atan2(2.0F * (w * x + y * z),
								 1.0F - 2.0F * (x * x + y * y)) * kRadToDeg;
	result.pitch_deg = std::asin(clampedPitch) * kRadToDeg;
	result.yaw_deg   = std::atan2(2.0F * (w * z + x * y),
								1.0F - 2.0F * (y * y + z * z)) * kRadToDeg;
	result.valid = true;
	return result;
}

imu::AttitudeSample imu::AttitudeEstimator::Update(
	const std::array<float, 3>& acceleration,
	const std::array<float, 3>& angularVelocity, float dt_s)
{
	if (!FiniteAxes(acceleration) || !FiniteAxes(angularVelocity))
	{
		Reset();
		return {};
	}

	const float normSquared = acceleration[0] * acceleration[0] +
							  acceleration[1] * acceleration[1] +
							  acceleration[2] * acceleration[2];
	const float norm = std::sqrt(normSquared);
	if (!initialized_)
	{
		if (norm < kMinimumGravity || norm > kMaximumGravity)
			return {};
		Initialize(acceleration);
		return EulerAngles();
	}

	if (!std::isfinite(dt_s) || dt_s <= 0.0F || dt_s > 0.05F)
	{
		Reset();
		return {};
	}

	float gx = angularVelocity[0];
	float gy = angularVelocity[1];
	float gz = angularVelocity[2];
	// 线性加速度较大时只积分陀螺仪，避免把加速运动误判为倾斜。
	if (norm >= kMinimumGravity && norm <= kMaximumGravity)
	{
		const float ax         = acceleration[0] / norm;
		const float ay         = acceleration[1] / norm;
		const float az         = acceleration[2] / norm;
		const float w          = quaternion_[0];
		const float x          = quaternion_[1];
		const float y          = quaternion_[2];
		const float z          = quaternion_[3];
		const float predictedX = 2.0F * (x * z - w * y);
		const float predictedY = 2.0F * (w * x + y * z);
		const float predictedZ = w * w - x * x - y * y + z * z;
		gx                     += kAccelCorrectionGain * (ay * predictedZ - az * predictedY);
		gy                     += kAccelCorrectionGain * (az * predictedX - ax * predictedZ);
		gz                     += kAccelCorrectionGain * (ax * predictedY - ay * predictedX);
	}

	const float w      = quaternion_[0];
	const float x      = quaternion_[1];
	const float y      = quaternion_[2];
	const float z      = quaternion_[3];
	const float halfDt = dt_s * 0.5F;
	quaternion_[0]     += (-x * gx - y * gy - z * gz) * halfDt;
	quaternion_[1]     += (w * gx + y * gz - z * gy) * halfDt;
	quaternion_[2]     += (w * gy - x * gz + z * gx) * halfDt;
	quaternion_[3]     += (w * gz + x * gy - y * gx) * halfDt;

	const float lengthSquared = quaternion_[0] * quaternion_[0] +
								quaternion_[1] * quaternion_[1] +
								quaternion_[2] * quaternion_[2] +
								quaternion_[3] * quaternion_[3];
	if (!std::isfinite(lengthSquared) || lengthSquared < 0.0001F)
	{
		Reset();
		return {};
	}
	const float inverseLength = 1.0F / std::sqrt(lengthSquared);
	for (float& component : quaternion_)
		component *= inverseLength;
	return EulerAngles();
}
