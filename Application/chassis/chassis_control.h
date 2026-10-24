//
// Created by 19923 on 2026/10/21.
//

#ifndef INFANTRY_H723_CHASSIS_CONTROL_H
#define INFANTRY_H723_CHASSIS_CONTROL_H
#include <cstdint>
#include <array>
#include <cmath>

namespace chassis
{
	// 明确采用右手系：x前、y左、z上，逆时针yaw为正；与参考代码y右约定不同。
	enum class Mode : uint8_t
	{
		Disabled,
		NoFollow,
		FollowGimbal,
		Spin,
		Direct
	};

	enum class chassis_mode : uint8_t
	{
		CHASSIS_NO_FOLLOW,         // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
		CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随云台,不单独设置pid,以误差角度平方为速度输出
		CHASSIS_ROTATE,            //自旋,同时保持全向机动;当前wz维持定值,todo后续增加不规则的变速策略
		poweroff
	};

	enum class CommandFrame : uint8_t
	{
		Chassis,
		Gimbal
	};

    inline constexpr unsigned kWheelCount = 4U; // FL、FR、BL、BR。
    struct WheelGeometry
    {
        float x_m{}, y_m{};            // x前、y左，轮心相对底盘中心。
        float drive_direction_deg{};  // 主动滚动正方向，从+x向+y为正，不是轮轴方向。
        float radius_m{};
        float max_output_speed_dps{}; // 轮轴，不是转子。
    };
    struct MotionLimits
    {
        float max_translation_mps{}, max_yaw_dps{};
        float translation_acceleration_mps2{}, yaw_acceleration_dps2{};
    };

    inline bool ControlPeriodTicks(float seconds, uint32_t frequency, uint32_t& ticks)
    {
        ticks = 0U;
        const double value = static_cast<double>(seconds) * frequency;
        if (!std::isfinite(value) || value < 0.5 || value >= 2147483648.0) return false;
        const double rounded = std::round(value);
        if (std::fabs(value - rounded) > 0.001) return false;
        ticks = static_cast<uint32_t>(rounded);
        return true;
    }

    /** @brief 纯全向轮速度规划：车体斜坡一次、四轮统一缩放，输出轴°/s。 */
    class WheelVelocityPlanner
    {
    public:
        bool Configure(const std::array<WheelGeometry, kWheelCount>& geometry, const MotionLimits& limits)
        {
            configured_ = false;
            Reset();
            for (float value : {limits.max_translation_mps, limits.max_yaw_dps,
                                limits.translation_acceleration_mps2, limits.yaw_acceleration_dps2})
                if (!std::isfinite(value) || value <= 0.0F) return false;
            float reference = 0.0F;
            for (const auto& w : geometry)
            {
                if (!std::isfinite(w.x_m) || !std::isfinite(w.y_m) ||
                    !std::isfinite(w.drive_direction_deg) || !std::isfinite(w.radius_m) ||
                    !std::isfinite(w.max_output_speed_dps) || w.radius_m <= 0.0F ||
                    w.max_output_speed_dps <= 0.0F) return false;
                reference = std::fmax(reference, std::hypot(w.x_m, w.y_m));
            }
            if (!std::isfinite(reference) || reference <= 0.0F) return false;
            float gram[3][3]{};
            for (unsigned i = 0; i < kWheelCount; ++i)
            {
                const auto& w = geometry[i];
                const float theta = std::remainder(w.drive_direction_deg, 360.0F) * kDegToRad;
                const float tx = std::cos(theta), ty = std::sin(theta);
                const float moment = -w.y_m * tx + w.x_m * ty;
                if (!std::isfinite(moment)) return false;
                rows_[i] = {tx, ty, moment};
                const float normalized[]{tx, ty, moment / reference};
                for (unsigned r = 0; r < 3; ++r)
                    for (unsigned c = 0; c < 3; ++c) gram[r][c] += normalized[r] * normalized[c];
            }
            // 三自由度必须满秩；避免空方向配置或不支持平移/旋转的轮组进入运行。
            const float det = gram[0][0] * (gram[1][1] * gram[2][2] - gram[1][2] * gram[2][1]) -
                              gram[0][1] * (gram[1][0] * gram[2][2] - gram[1][2] * gram[2][0]) +
                              gram[0][2] * (gram[1][0] * gram[2][1] - gram[1][1] * gram[2][0]);
            if (!std::isfinite(det) || det <= 1e-5F) return false;
            geometry_ = geometry;
            limits_ = limits;
            configured_ = true;
            return true;
        }

        void Reset() { vx_ = vy_ = yaw_ = 0.0F; }

        bool Update(float vx, float vy, float yaw, float dt, std::array<float, kWheelCount>& output)
        {
            output = {};
            if (!configured_ || !std::isfinite(vx) || !std::isfinite(vy) ||
                !std::isfinite(yaw) || !std::isfinite(dt) || dt <= 0.0F) return Fail(output);
            const float norm = std::hypot(vx, vy);
            if (!std::isfinite(norm)) return Fail(output);
            if (norm > limits_.max_translation_mps)
            { vx *= limits_.max_translation_mps / norm; vy *= limits_.max_translation_mps / norm; }
            yaw = Limit(yaw, limits_.max_yaw_dps);
            const float delta = std::hypot(vx - vx_, vy - vy_);
            const float step = limits_.translation_acceleration_mps2 * dt;
            const float yawStep = limits_.yaw_acceleration_dps2 * dt;
            if (!std::isfinite(delta) || !std::isfinite(step) || !std::isfinite(yawStep)) return Fail(output);
            if (delta > step)
            { vx = vx_ + (vx - vx_) * step / delta; vy = vy_ + (vy - vy_) * step / delta; }
            yaw = yaw_ + Limit(yaw - yaw_, yawStep);
            float scale = 1.0F;
            for (unsigned i = 0; i < kWheelCount; ++i)
            {
                const auto& row = rows_[i];
                output[i] = (row[0] * vx + row[1] * vy + row[2] * yaw * kDegToRad) /
                            (geometry_[i].radius_m * kDegToRad);
                if (!std::isfinite(output[i])) return Fail(output);
                const float magnitude = std::fabs(output[i]);
                if (magnitude > geometry_[i].max_output_speed_dps)
                    scale = std::fmin(scale, geometry_[i].max_output_speed_dps / magnitude);
            }
            for (float& speed : output) speed *= scale;
            // 保存实际可执行目标，避免超限状态在斜坡内部累积。
            vx_ = vx * scale; vy_ = vy * scale; yaw_ = yaw * scale;
            return true;
        }

    private:
        static constexpr float kDegToRad = 0.017453292519943295F;
        static float Limit(float v, float limit) { return std::fmax(-limit, std::fmin(limit, v)); }
        bool Fail(std::array<float, kWheelCount>& output) { output = {}; Reset(); return false; }
        std::array<WheelGeometry, kWheelCount> geometry_{};
        std::array<std::array<float, 3>, kWheelCount> rows_{};
        MotionLimits limits_{};
        float vx_{}, vy_{}, yaw_{};
        bool configured_{};
    };


}

#endif //INFANTRY_H723_CHASSIS_CONTROL_H
