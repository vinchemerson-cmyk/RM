/** @file shoot_control.h @brief 摩擦轮爬升/就绪、单发/连发及有限次数堵转反转状态机。 */
#pragma once
#include <cmath>
#include <cstdint>

namespace shoot
{
	enum class Trigger : uint8_t { Stop, Single, Continuous };

	enum class FeederState : uint8_t { Idle, Single, Continuous, Reversing, Fault };

	/** @brief 所有速度、角度均为电机转子轴单位，反馈方向在电机驱动中归一化。 */
	struct ControlConfig
	{
		bool     enable_feeder{true};       // 测试双摩擦轮时可单独禁止拨弹。
		float    wheel_speed_dps{36000.0F}; // 转子 6000 RPM
		float    wheel_acceleration_dps2{12000.0F};
		float    wheel_ready_tolerance_dps{1500.0F};
		uint32_t wheel_ready_hold_ms{200U};
		float    feeder_speed_dps{12960.0F};     // 36:1 减速后拨盘 60 RPM
		float    single_shot_angle_deg{2160.0F}; // 36:1，六槽拨盘一次 60 度
		float    single_done_tolerance_deg{30.0F};
		float    single_done_speed_dps{360.0F};
		float    stall_speed_dps{600.0F};
		int16_t  stall_current_raw{5000};
		uint32_t stall_hold_ms{250U};
		float    reverse_speed_dps{6480.0F};
		uint32_t reverse_duration_ms{180U};
		uint8_t  max_reverse_attempts{3U};
	};

	struct ControlInput
	{
		bool     allowed{};
		bool     wheels_requested{};
		Trigger  trigger{Trigger::Stop};
		float    left_speed_dps{}, right_speed_dps{}, feeder_speed_dps{}, feeder_angle_deg{};
		int16_t  feeder_current_raw{};
		uint32_t feeder_epoch{};
	};

	struct ControlOutput
	{
		bool        wheels_enabled{};
		float       wheel_speed_dps{};
		bool        wheels_ready{};
		bool        feeder_enabled{};
		bool        feeder_position_mode{};
		float       feeder_speed_dps{}, feeder_angle_deg{};
		FeederState feeder_state{FeederState::Idle};
	};

	class Controller
	{
		public:
			explicit Controller(const ControlConfig& config = {}) : config_(config)
			{
			}

			/** @brief 急停、摩擦轮关闭或反馈异常时，丢弃原有发射目标与触发状态。 */
			void Reset()
			{
				wheel_speed_    = 0.0F;
				ready_tracking_ = false;
				epoch_valid_    = false;
				CancelFeeder(Trigger::Stop);
			}

			/** @brief 每个控制周期更新，只有稳定转速且见过 Stop 档后才允许触发拨弹。 */
			ControlOutput Update(const ControlInput& input, uint32_t now, float dt_s)
			{
				ControlOutput output{};
				if (!input.allowed || !input.wheels_requested ||
					!std::isfinite(dt_s) || dt_s <= 0.0F || dt_s > 0.05F ||
					!std::isfinite(input.left_speed_dps) || !std::isfinite(input.right_speed_dps) ||
					(config_.enable_feeder &&
					 (!std::isfinite(input.feeder_speed_dps) || !std::isfinite(input.feeder_angle_deg))))
				{
					Reset();
					return output;
				}
				output.wheels_enabled = true;
				wheel_speed_          = std::fmin(config_.wheel_speed_dps,
												  wheel_speed_ + config_.wheel_acceleration_dps2 * dt_s);
				output.wheel_speed_dps  = wheel_speed_;
				const bool speedReached = wheel_speed_ >= config_.wheel_speed_dps &&
										  std::fabs(input.left_speed_dps - config_.wheel_speed_dps) <= config_.
										  wheel_ready_tolerance_dps &&
										  std::fabs(input.right_speed_dps - config_.wheel_speed_dps) <= config_.
										  wheel_ready_tolerance_dps;
				if (!speedReached)
					ready_tracking_ = false;
				else if (!ready_tracking_)
				{
					ready_tracking_ = true;
					ready_since_    = now;
				}
				output.wheels_ready = ready_tracking_ && now - ready_since_ >= config_.wheel_ready_hold_ms;
				if (!config_.enable_feeder || !output.wheels_ready)
				{
					CancelFeeder(input.trigger);
					return output;
				}
				if (!epoch_valid_ || input.feeder_epoch != epoch_)
				{
					CancelFeeder(input.trigger);
					epoch_       = input.feeder_epoch;
					epoch_valid_ = true;
				}
				if (input.trigger == Trigger::Stop)
				{
					fire_armed_ = true;
					if (state_ != FeederState::Single)
					{
						state_            = FeederState::Idle;
						reverse_attempts_ = 0U;
						stall_tracking_   = false;
					}
				}
				if (state_ == FeederState::Fault)
				{
					previous_trigger_   = input.trigger;
					output.feeder_state = state_;
					return output;
				}
				if (state_ == FeederState::Reversing && now - reverse_since_ >= config_.reverse_duration_ms)
				{
					state_          = resume_state_;
					stall_tracking_ = false;
				}
				if (state_ != FeederState::Reversing && fire_armed_)
				{
					if (input.trigger == Trigger::Continuous)
						state_ = FeederState::Continuous;
					else if (input.trigger == Trigger::Single && previous_trigger_ != Trigger::Single &&
							 state_ != FeederState::Single)
					{
						target_angle_   = input.feeder_angle_deg + config_.single_shot_angle_deg;
						state_          = FeederState::Single;
						stall_tracking_ = false;
					}
				}
				previous_trigger_ = input.trigger;
				if (state_ == FeederState::Single &&
					std::fabs(target_angle_ - input.feeder_angle_deg) <= config_.single_done_tolerance_deg &&
					std::fabs(input.feeder_speed_dps) <= config_.single_done_speed_dps)
				{
					state_            = FeederState::Idle;
					reverse_attempts_ = 0U;
					stall_tracking_   = false;
				}
				const bool moving = state_ == FeederState::Continuous ||
									(state_ == FeederState::Single &&
									 std::fabs(target_angle_ - input.feeder_angle_deg) > config_.
									 single_done_tolerance_deg);
				const bool stalled = moving && std::fabs(input.feeder_speed_dps) < config_.stall_speed_dps &&
									 std::abs(static_cast<int32_t>(input.feeder_current_raw)) >= config_.
									 stall_current_raw;
				if (!stalled)
					stall_tracking_ = false;
				else if (!stall_tracking_)
				{
					stall_tracking_ = true;
					stall_since_    = now;
				}
				if (stalled && now - stall_since_ >= config_.stall_hold_ms)
				{
					stall_tracking_ = false;
					if (reverse_attempts_ >= config_.max_reverse_attempts)
						state_ = FeederState::Fault;
					else
					{
						++reverse_attempts_;
						resume_state_  = state_;
						reverse_since_ = now;
						state_         = FeederState::Reversing;
					}
				}
				output.feeder_state   = state_;
				output.feeder_enabled = state_ == FeederState::Single || state_ == FeederState::Continuous ||
										state_ == FeederState::Reversing;
				output.feeder_position_mode = state_ == FeederState::Single;
				output.feeder_angle_deg     = target_angle_;
				output.feeder_speed_dps     = state_ == FeederState::Reversing
												  ? -config_.reverse_speed_dps
												  : state_ == FeederState::Continuous
														? config_.feeder_speed_dps
														: 0.0F;
				return output;
			}

		private:
			void CancelFeeder(Trigger trigger)
			{
				state_            = FeederState::Idle;
				fire_armed_       = false;
				previous_trigger_ = trigger;
				reverse_attempts_ = 0U;
				stall_tracking_   = false;
				target_angle_     = 0.0F;
			}

			ControlConfig config_{};
			float         wheel_speed_{},    target_angle_{};
			bool          ready_tracking_{}, fire_armed_{},  stall_tracking_{}, epoch_valid_{};
			uint32_t      ready_since_{},    stall_since_{}, reverse_since_{},  epoch_{};
			uint8_t       reverse_attempts_{};
			Trigger       previous_trigger_{Trigger::Stop};
			FeederState   state_{FeederState::Idle}, resume_state_{FeederState::Idle};
	};
}
