/**
 * @file AttitudeEstimator.h
 * @brief 用陀螺仪积分和重力方向修正估计六轴姿态。
 */

#ifndef INFANTRY_H723_ATTITUDE_ESTIMATOR_H
#define INFANTRY_H723_ATTITUDE_ESTIMATOR_H

#include <array>
#include <cstdint>

namespace imu
{
	/** @brief 机体坐标系的欧拉角，单位度；Yaw 以上电时朝向为零。 */
	struct AttitudeSample
	{
		float    roll_deg{};
		float    pitch_deg{};
		float    yaw_deg{};
		uint32_t tick{};
		bool     valid{};
	};

	/** @brief 六轴四元数互补估计器，不依赖 HAL 或 FreeRTOS。 */
	class AttitudeEstimator
	{
		public:
			/** @brief 清除四元数和时间基准。 */
			void Reset();

		/**
         * @brief 用一次六轴采样更新姿态。
         * @param acceleration_mps2 三轴加速度，单位 m/s²。
         * @param angular_velocity_rad_s 三轴角速度，单位 rad/s。
         * @param dt_s 与上一次有效采样的间隔，单位秒；首次调用忽略。
         * @return 最新姿态；无效采样时 valid 为 false。
         */
			AttitudeSample Update(const std::array<float, 3>& acceleration_mps2,
								  const std::array<float, 3>& angular_velocity_rad_s,
								  float                       dt_s);

		private:
			void                         Initialize(const std::array<float, 3>& acceleration_mps2);
			[[nodiscard]] AttitudeSample EulerAngles() const;

			std::array<float, 4> quaternion_{1.0F, 0.0F, 0.0F, 0.0F};
			bool                 initialized_{};
	};
}

#endif // INFANTRY_H723_ATTITUDE_ESTIMATOR_H
