/** @file shoot_control_test.cpp @brief 发射状态机的就绪、触发、反馈及堵转回归测试。 */
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "shoot/shoot_control.h"

static shoot::ControlConfig Config()
{
	shoot::ControlConfig c{};
	c.wheel_speed_dps           = 1000.0F;
	c.wheel_acceleration_dps2   = 100000.0F;
	c.wheel_ready_tolerance_dps = 20.0F;
	c.wheel_ready_hold_ms       = 20U;
	c.feeder_speed_dps          = 600.0F;
	c.single_shot_angle_deg     = 360.0F;
	c.single_done_tolerance_deg = 5.0F;
	c.single_done_speed_dps     = 10.0F;
	c.stall_speed_dps           = 20.0F;
	c.stall_current_raw         = 100;
	c.stall_hold_ms             = 30U;
	c.reverse_speed_dps         = 300.0F;
	c.reverse_duration_ms       = 20U;
	c.max_reverse_attempts      = 2U;
	return c;
}

static shoot::ControlInput Input()
{
	shoot::ControlInput i{};
	i.allowed        = i.wheels_requested = true;
	i.left_speed_dps = i.right_speed_dps  = 1000.0F;
	i.feeder_epoch   = 1U;
	return i;
}

static void Ready(shoot::Controller& c, const shoot::ControlInput& i)
{
	assert(!c.Update(i, 0U, 0.01F).wheels_ready);
	assert(!c.Update(i, 10U, 0.01F).wheels_ready);
	assert(c.Update(i, 20U, 0.01F).wheels_ready);
}

int main()
{
	auto              cfg   = Config();
	auto              input = Input();
	shoot::Controller controller{cfg};
	input.trigger = shoot::Trigger::Continuous;
	Ready(controller, input);
	assert(!controller.Update(input, 30U, 0.01F).feeder_enabled); // 启动时不能沿用 held trigger
	input.trigger = shoot::Trigger::Stop;
	controller.Update(input, 40U, 0.01F);
	input.trigger = shoot::Trigger::Continuous;
	auto out      = controller.Update(input, 50U, 0.01F);
	assert(out.feeder_enabled && !out.feeder_position_mode && out.feeder_speed_dps == 600.0F);
	input.trigger = shoot::Trigger::Stop;
	assert(!controller.Update(input, 60U, 0.01F).feeder_enabled);
	input.trigger          = shoot::Trigger::Single;
	input.feeder_angle_deg = 100.0F;
	out                    = controller.Update(input, 70U, 0.01F);
	assert(out.feeder_position_mode && out.feeder_angle_deg == 460.0F);
	assert(controller.Update(input, 80U, 0.01F).feeder_angle_deg == 460.0F); // held single 不重复加目标
	input.feeder_angle_deg = 460.0F;
	assert(!controller.Update(input, 90U, 0.01F).feeder_enabled);
	assert(!controller.Update(input, 100U, 0.01F).feeder_enabled);
	input.trigger = shoot::Trigger::Stop;
	controller.Update(input, 110U, 0.01F);
	input.trigger = shoot::Trigger::Single;
	assert(controller.Update(input, 120U, 0.01F).feeder_angle_deg == 820.0F);
	input.allowed = false;
	out           = controller.Update(input, 130U, 0.01F);
	assert(!out.wheels_enabled && !out.feeder_enabled);
	input         = Input();
	input.trigger = shoot::Trigger::Continuous;
	controller.Update(input, 140U, 0.01F);
	controller.Update(input, 150U, 0.01F);
	assert(!controller.Update(input, 160U, 0.01F).feeder_enabled); // 恢复不自动续发
	input.trigger = shoot::Trigger::Stop;
	controller.Update(input, 170U, 0.01F);
	input.trigger = shoot::Trigger::Continuous;
	assert(controller.Update(input, 180U, 0.01F).feeder_enabled);
	++input.feeder_epoch;
	assert(!controller.Update(input, 190U, 0.01F).feeder_enabled); // 重锚丢弃旧目标
	input.trigger = shoot::Trigger::Stop;
	controller.Update(input, 200U, 0.01F);
	input.trigger = shoot::Trigger::Continuous;
	out           = controller.Update(input, 1000U, 0.01F); // 相同反馈长时间不更新不触发掉线停机
	assert(out.wheels_enabled && out.feeder_enabled);

	auto wheelsOnlyConfig          = cfg;
	wheelsOnlyConfig.enable_feeder = false;
	shoot::Controller wheelsOnly{wheelsOnlyConfig};
	auto              wheelsInput = Input();
	wheelsInput.right_speed_dps   = 0.0F;
	out                           = wheelsOnly.Update(wheelsInput, 0U, 0.01F);
	assert(out.wheels_enabled && !out.wheels_ready && !out.feeder_enabled);
	wheelsInput.right_speed_dps = wheelsInput.left_speed_dps;
	wheelsOnly.Update(wheelsInput, 10U, 0.01F);
	out = wheelsOnly.Update(wheelsInput, 30U, 0.01F);
	assert(out.wheels_enabled && out.wheels_ready && !out.feeder_enabled);
	for (auto trigger : {shoot::Trigger::Stop, shoot::Trigger::Single, shoot::Trigger::Continuous})
	{
		wheelsInput.trigger = trigger;
		out                 = wheelsOnly.Update(wheelsInput, 10000U, 0.01F);
		assert(out.wheels_enabled && out.wheels_ready && !out.feeder_enabled);
	}
	wheelsInput.allowed = false;
	out                 = wheelsOnly.Update(wheelsInput, 10010U, 0.01F);
	assert(!out.wheels_enabled && !out.feeder_enabled); // 遥控失联/急停仍同时停两轮
	wheelsInput                  = Input();
	wheelsInput.wheels_requested = false;
	assert(!wheelsOnly.Update(wheelsInput, 10020U, 0.01F).wheels_enabled);

	controller.Reset();
	input = Input();
	Ready(controller, input);
	input.trigger            = shoot::Trigger::Continuous;
	input.feeder_current_raw = 150;
	controller.Update(input, 30U, 0.01F);
	out = controller.Update(input, 60U, 0.01F);
	assert(out.feeder_state == shoot::FeederState::Reversing && out.feeder_speed_dps == -300.0F);
	out = controller.Update(input, 80U, 0.01F);
	assert(out.feeder_state == shoot::FeederState::Continuous);
	assert(controller.Update(input, 110U, 0.01F).feeder_state == shoot::FeederState::Reversing);
	controller.Update(input, 130U, 0.01F);
	out = controller.Update(input, 160U, 0.01F);
	assert(out.feeder_state == shoot::FeederState::Fault && !out.feeder_enabled);
	assert(!controller.Update(input, 170U, 0.01F).feeder_enabled);
	input.trigger = shoot::Trigger::Stop;
	assert(controller.Update(input, 180U, 0.01F).feeder_state == shoot::FeederState::Idle);
	input.trigger = shoot::Trigger::Continuous;
	assert(controller.Update(input, 190U, 0.01F).feeder_enabled);
	input.feeder_angle_deg = std::numeric_limits<float>::quiet_NaN();
	out                    = controller.Update(input, 200U, 0.01F);
	assert(!out.wheels_enabled && !out.feeder_enabled);
	input                  = Input();
	input.wheels_requested = false;
	assert(!controller.Update(input, 210U, 0.01F).wheels_enabled);
	input.wheels_requested = true;
	assert(!controller.Update(input, 220U, 0.1F).wheels_enabled);
	std::cout << "Shooter: dual-wheel coupling, wheels-only mode, remote stop, triggers and jam recovery: PASS\n";
}
