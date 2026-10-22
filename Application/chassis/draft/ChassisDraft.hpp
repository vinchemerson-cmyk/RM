/** @file ChassisDraft.hpp @brief 四全向轮底盘纯逻辑草稿；不注册电机、不创建任务。 */
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace chassis::draft
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

	enum class CommandFrame : uint8_t
	{
		Chassis,
		Gimbal
	};

	enum class StopReason : uint8_t
	{
		None,
		Unconfigured,
		Safety,
		CommandStale,
		InvalidCommand,
		InvalidTiming
	};

	enum Wheel : unsigned
	{
		FrontLeft,
		FrontRight,
		RearLeft,
		RearRight,
		WheelCount
	};

	struct Config
	{
		bool                          commissioned{false};       // 机构、方向和安全策略确认前不得设为true。
		float                         wheel_radius_m{};          //车轮半径
		float                         wheelbase_m{};             // 前后轮中心距离；假定四轮在矩形四角。
		float                         track_width_m{};           // 左右轮中心距离。
		float                         max_translation_mps{};     //底盘最大平移速度
		float                         max_yaw_dps{};             //最大底盘yaw轴转速(小陀螺用?
		float                         max_wheel_output_dps{};    // 实际车轮轴，不是电机转子。
		float                         follow_gain_per_s{};       // yaw误差deg -> yaw速度deg/s；不照搬参考平方增益。
		float                         spin_yaw_dps{};            //小陀螺转速
		uint32_t                      command_timeout_ms{100U};  // 草稿值，接入时与指令发布周期核对。
		uint32_t                      feedback_timeout_ms{100U}; //反馈超时
		float                         max_dt_s{0.001F};
		// 各轮正向滚动的驱动方向角，从底盘+x向+y为正；不是轮轴指向。
		// 必须按实物填写，不能套用麦轮公式。轮序FL/FR/RL/RR。
		std::array<float, WheelCount> drive_direction_deg{};
		float                         max_translation_accel_mps2{3.0F}; // 草稿起点，实车需按轮胎抓地/负载整定。
		float                         max_yaw_accel_dps2{720.0F};
		float                         follow_deadband_deg{1.0F};    // 跟随死区连续减去，避免边界速度跳变。
		bool                          require_wheel_feedback{true}; // false仅用于离线仿真，不建议实车关闭。
	};

	struct Command
	{
		Mode     mode{Mode::Disabled};
		CommandFrame    frame{CommandFrame::Chassis};
		float    vx_mps{}, vy_mps{}, yaw_rate_dps{};
		float    gimbal_yaw_relative_deg{}; // 云台相对底盘，左转为正；须与上述速度同步有效。
		uint32_t source_tick_ms{};          // 原始有效指令时间，重复转发不能刷新。
		uint32_t sequence{};//序列
	};

	struct WheelFeedback
	{
		std::array<float, WheelCount> output_speed_dps{};
		bool                          valid{};
		uint32_t                      oldest_sample_tick_ms{}; // 四轮中最老的有效采样时间。
	};

	struct Result
	{
		std::array<float, WheelCount> wheel_target_output_dps{};
		bool                          output_requested{}; // 不是硬件输出许可；最终安全和驱动有效性仍需校验。
		bool                          estimate_valid{};
		StopReason                    stop_reason{StopReason::Unconfigured};
		float                         requested_vx_mps{}, requested_vy_mps{}, requested_yaw_dps{};
		float                         wheel_scale{1.0F}; // 四轮按相同比例缩放，保持运动比例；不是功率控制。
		float                         measured_vx_mps{}, measured_vy_mps{}, measured_yaw_dps{};
		uint32_t                      tick_ms{},         command_sequence{};
	};

	class Controller
	{
		public:
			explicit Controller(const Config& config) : config_(config)
			{
				configured_ = ValidateConfig();
				if (!configured_) return;
				for (unsigned i = 0U; i < WheelCount; ++i) geometry_[i] = BuildGeometryRow(i);
				// 机构不变，只在初始化求伪逆；周期控制不再三角计算轮组几何或消元。
				for (unsigned i = 0U; i < WheelCount; ++i)
				{
					std::array<float, WheelCount> unit{};
					unit[i] = 1.0F;
					std::array<float, 3> column{};
					if (!BuildSolve(unit, column)) { configured_ = false; return; }
					for (unsigned r = 0U; r < 3U; ++r) inverse_[r][i] = column[r];
				}
			}

			void Reset() { vx_ = vy_ = yaw_ = 0.0F; }

			Result Update(const Command&       command, bool      have_command, bool safety_allowed,
						  const WheelFeedback& feedback, uint32_t now_ms, float      dt_s,
						  float speed_scale = 1.0F)
			{
				Result result{};
				result.tick_ms          = now_ms;
				result.command_sequence = command.sequence;
				if (!configured_) return Stop(result);
				Estimate(feedback, now_ms, result);
				if (!safety_allowed)
				{
					result.stop_reason = StopReason::Safety;
					return Stop(result);
				}
				if (!have_command || now_ms - command.source_tick_ms > config_.command_timeout_ms)
				{
					result.stop_reason = StopReason::CommandStale;
					return Stop(result);
				}
				if (!std::isfinite(dt_s) || dt_s <= 0.0F || dt_s > config_.max_dt_s)
				{
					result.stop_reason = StopReason::InvalidTiming;
					return Stop(result);
				}
				if (!Valid(command))
				{
					result.stop_reason = StopReason::InvalidCommand;
					return Stop(result);
				}
				result.stop_reason = StopReason::None;
				if (command.mode == Mode::Disabled) return Stop(result);
				// 失效四轮反馈时撤销请求；不是只把速度估计标为无效后继续驱动。
				if (config_.require_wheel_feedback && !result.estimate_valid)
				{ result.stop_reason = StopReason::Safety; return Stop(result); }
				// 外部安全/比赛策略可限制速度能力；不是电功率或电流限制器。
				if (!std::isfinite(speed_scale) || speed_scale < 0.0F || speed_scale > 1.0F)
				{ result.stop_reason = StopReason::InvalidCommand; return Stop(result); }
				if (speed_scale == 0.0F)
				{ result.stop_reason = StopReason::Safety; return Stop(result); }

				float       vx   = command.vx_mps, vy = command.vy_mps;
				const float norm = std::hypot(vx, vy);
				if (!std::isfinite(norm))
				{ result.stop_reason = StopReason::InvalidCommand; return Stop(result); }
				if (norm > config_.max_translation_mps)
				{
					const float scale = config_.max_translation_mps / norm;
					vx                *= scale;
					vy                *= scale;
				}
				const float offset = std::remainder(command.gimbal_yaw_relative_deg, 360.0F);
				if (command.frame == CommandFrame::Gimbal)
				{
					const float theta = offset * kDegToRad;
					const float x     = vx * std::cos(theta) - vy * std::sin(theta);
					vy                = vx * std::sin(theta) + vy * std::cos(theta);
					vx                = x;
				}
				float yaw = command.yaw_rate_dps;
				switch (command.mode)
				{
					case Mode::NoFollow: yaw = 0.0F;
						break;
					case Mode::FollowGimbal:
						yaw = config_.follow_gain_per_s * std::copysign(
							std::fmax(0.0F, std::fabs(offset) - config_.follow_deadband_deg), offset);
						break;
					case Mode::Spin: yaw = config_.spin_yaw_dps;
						break;
					default: break;
				}
				yaw                      = Limit(yaw, config_.max_yaw_dps);
				vx *= speed_scale; vy *= speed_scale; yaw *= speed_scale;
				const float delta = std::hypot(vx - vx_, vy - vy_);
				const float max_delta = config_.max_translation_accel_mps2 * dt_s;
				if (delta > max_delta)
				{
					const float scale = max_delta / delta;
					vx = vx_ + (vx - vx_) * scale;
					vy = vy_ + (vy - vy_) * scale;
				}
				yaw = yaw_ + Limit(yaw - yaw_, config_.max_yaw_accel_dps2 * dt_s);
				// 能力收紧时优先满足新速度上限，而非为了平滑继续超限。
				const float limited_norm = std::hypot(vx, vy);
				const float max_norm = config_.max_translation_mps * speed_scale;
				if (limited_norm > max_norm)
				{ vx *= max_norm / limited_norm; vy *= max_norm / limited_norm; }
				yaw = Limit(yaw, config_.max_yaw_dps * speed_scale);
				result.requested_vx_mps  = vx;
				result.requested_vy_mps  = vy;
				result.requested_yaw_dps = yaw;
				const float factor       = 1.0F / (config_.wheel_radius_m * kDegToRad);
				// 全向轮：将轮心速度投影到该轮的主动滚动方向；自由滚子方向不施加约束。
				auto&       wheels       = result.wheel_target_output_dps;
				for (unsigned i = 0U; i < WheelCount; ++i)
				{
					const auto  row = GeometryRow(i);
					const float reference_length = (config_.wheelbase_m + config_.track_width_m) * 0.5F;
					wheels[i] = (row[0] * vx + row[1] * vy + row[2] * reference_length * yaw * kDegToRad) * factor;
				}
				float peak = 0.0F;
				for (float value : wheels)
				{
					if (!std::isfinite(value))
					{
						result.wheel_target_output_dps = {};
						result.stop_reason             = StopReason::InvalidCommand;
						return Stop(result);
					}
					peak = std::fmax(peak, std::fabs(value));
				}
				if (peak > config_.max_wheel_output_dps * speed_scale)
				{
					result.wheel_scale = config_.max_wheel_output_dps * speed_scale / peak;
					for (float& value : wheels) value *= result.wheel_scale;
				}
				result.output_requested = true;
				// 记录实际下发目标，避免轮速饱和时继续积压爬升状态。
				vx_ = vx * result.wheel_scale;
				vy_ = vy * result.wheel_scale;
				yaw_ = yaw * result.wheel_scale;
				return result;
			}

		private:
			static constexpr float kDegToRad = 0.017453292519943295F;
			Config                 config_;
			bool configured_{};
			float vx_{}, vy_{}, yaw_{};
			std::array<std::array<float, 3>, WheelCount> geometry_{};
			std::array<std::array<float, WheelCount>, 3> inverse_{};

			Result Stop(Result result)
			{
				Reset();
				result.wheel_target_output_dps = {};
				result.output_requested = false;
				return result;
			}

			static float Limit(float value, float limit)
			{
				return std::fmax(-limit, std::fmin(limit, value));
			}

			bool ValidateConfig() const
			{
				const float positive[]{
					config_.wheel_radius_m, config_.wheelbase_m, config_.track_width_m,
					config_.max_translation_mps, config_.max_yaw_dps, config_.max_wheel_output_dps, config_.max_dt_s,
					config_.max_translation_accel_mps2, config_.max_yaw_accel_dps2
				};
				for (float value : positive) if (!std::isfinite(value) || value <= 0.0F) return false;
				for (float angle : config_.drive_direction_deg) if (!std::isfinite(angle)) return false;
				return config_.commissioned && std::isfinite(config_.follow_gain_per_s) &&
					   std::isfinite(config_.follow_deadband_deg) && config_.follow_deadband_deg >= 0.0F &&
					   config_.follow_deadband_deg < 180.0F &&
					   config_.follow_gain_per_s >= 0.0F && std::isfinite(config_.spin_yaw_dps) &&
					   config_.command_timeout_ms > 0U && config_.command_timeout_ms < 0x80000000U &&
					   config_.feedback_timeout_ms > 0U && config_.feedback_timeout_ms < 0x80000000U;
			}

			static bool Valid(const Command& command)
			{
				switch (command.mode)
				{
					case Mode::Disabled:
					case Mode::NoFollow:
					case Mode::FollowGimbal:
					case Mode::Spin:
					case Mode::Direct: break;
					default: return false;
				}
				return (command.frame == CommandFrame::Chassis || command.frame == CommandFrame::Gimbal) &&
					   std::isfinite(command.vx_mps) && std::isfinite(command.vy_mps) &&
					   std::isfinite(command.yaw_rate_dps) && std::isfinite(command.gimbal_yaw_relative_deg);
			}

			const std::array<float, 3>& GeometryRow(unsigned i) const { return geometry_[i]; }

			std::array<float, 3> BuildGeometryRow(unsigned i) const
			{
				const float theta = std::remainder(config_.drive_direction_deg[i], 360.0F) * kDegToRad;
				const float c = std::cos(theta), s = std::sin(theta);
				const float x = (i < 2U ? 1.0F : -1.0F) * config_.wheelbase_m * 0.5F;
				const float y = (i == FrontLeft || i == RearLeft ? 1.0F : -1.0F) * config_.track_width_m * 0.5F;
				// 第三列使用归一化转动变量q = yaw_rad_s * reference_length，改善矩阵尺度。
				const float reference_length = (config_.wheelbase_m + config_.track_width_m) * 0.5F;
				return {c, s, (-y * c + x * s) / reference_length};
			}

			bool BuildSolve(const std::array<float, WheelCount>& linear_speed,
							   std::array<float, 3>&                velocity) const
			{
				// 最小二乘(A^T A)^-1 A^T v，4轮估计平面3自由度；不处理打滑。
				float a[3][4]{};
				for (unsigned i = 0U; i < WheelCount; ++i)
				{
					const auto row = GeometryRow(i);
					for (unsigned r = 0U; r < 3U; ++r)
					{
						for (unsigned c = 0U; c < 3U; ++c) a[r][c] += row[r] * row[c];
						a[r][3] += row[r] * linear_speed[i];
					}
				}
				for (unsigned column = 0U; column < 3U; ++column)
				{
					unsigned pivot = column;
					for (unsigned r = column + 1U; r < 3U; ++r)
						if (std::fabs(a[r][column]) > std::fabs(a[pivot][column])) pivot = r;
					if (!std::isfinite(a[pivot][column]) || std::fabs(a[pivot][column]) < 1e-5F) return false;
					if (pivot != column)
						for (unsigned c = 0U; c < 4U; ++c)
						{
							const float temp = a[column][c];
							a[column][c]     = a[pivot][c];
							a[pivot][c]      = temp;
						}
					const float divisor = a[column][column];
					for (unsigned c = column; c < 4U; ++c) a[column][c] /= divisor;
					for (unsigned r = 0U; r < 3U; ++r)
					{
						if (r == column) continue;
						const float scale = a[r][column];
						for (unsigned c = column; c < 4U; ++c) a[r][c] -= scale * a[column][c];
					}
				}
				velocity = {a[0][3], a[1][3], a[2][3]};
				return std::isfinite(velocity[0]) && std::isfinite(velocity[1]) && std::isfinite(velocity[2]);
			}

			bool SolveVelocity(const std::array<float, WheelCount>& linear_speed,
							   std::array<float, 3>& velocity) const
			{
				velocity = {};
				for (unsigned r = 0U; r < 3U; ++r)
					for (unsigned i = 0U; i < WheelCount; ++i)
						velocity[r] += inverse_[r][i] * linear_speed[i];
				return std::isfinite(velocity[0]) && std::isfinite(velocity[1]) && std::isfinite(velocity[2]);
			}

			void Estimate(const WheelFeedback& feedback, uint32_t now_ms, Result& result) const
			{
				if (!feedback.valid || now_ms - feedback.oldest_sample_tick_ms > config_.feedback_timeout_ms) return;
				for (float value : feedback.output_speed_dps) if (!std::isfinite(value)) return;
				std::array<float, WheelCount> speed{};
				for (unsigned i = 0U; i < WheelCount; ++i)
					speed[i] = feedback.output_speed_dps[i] * config_.wheel_radius_m * kDegToRad;
				std::array<float, 3> velocity{};
				if (!SolveVelocity(speed, velocity)) return;
				result.measured_vx_mps  = velocity[0];
				result.measured_vy_mps  = velocity[1];
				result.measured_yaw_dps = velocity[2] /
										  ((config_.wheelbase_m + config_.track_width_m) * 0.5F * kDegToRad);
				result.estimate_valid = std::isfinite(result.measured_vx_mps) &&
										std::isfinite(result.measured_vy_mps) && std::isfinite(result.measured_yaw_dps);
			}
	};
}
