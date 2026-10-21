/** 独立宿主机草稿自检；不加入正式CMake，不涉及电机。 */
#include "ChassisDraftBus.hpp"
#include <cassert>
#include <limits>

using namespace chassis::draft;
int main()
{
    // 以下尺寸仅为数学验证样例，不是实车配置。
    Config config{true, 0.1F, 0.4F, 0.4F, 3.0F, 180.0F, 10000.0F, 2.0F, 90.0F};
    // 四角切向全向轮数学样例，不是实车确认方向。
    config.drive_direction_deg = {135.0F, 45.0F, -135.0F, -45.0F};
    Controller control(config);
    Command command{Mode::Direct, Frame::Chassis, 1.0F, 0.0F, 0.0F, 0.0F, 10U, 1U};
    WheelFeedback feedback{};
    auto result = control.Update(command, true, true, feedback, 10U, 0.001F);
    assert(result.output_requested && !result.estimate_valid);
    const std::array<float, 4> forward_sign{-1.0F, 1.0F, -1.0F, 1.0F};
    for (unsigned i = 0U; i < WheelCount; ++i)
        assert(std::fabs(result.wheel_target_output_dps[i] - forward_sign[i] * 405.1423F) < 0.01F);
    feedback = {result.wheel_target_output_dps, true, 10U};
    result = control.Update(command, true, true, feedback, 10U, 0.001F);
    assert(result.estimate_valid && std::fabs(result.measured_vx_mps - 1.0F) < 1e-5F);
    command.vy_mps = 0.5F; command.yaw_rate_dps = 50.0F;
    result = control.Update(command, true, true, {}, 10U, 0.001F);
    feedback = {result.wheel_target_output_dps, true, 10U};
    result = control.Update(command, true, true, feedback, 10U, 0.001F);
    assert(std::fabs(result.measured_vx_mps - 1.0F) < 1e-5F);
    assert(std::fabs(result.measured_vy_mps - 0.5F) < 1e-5F);
    assert(std::fabs(result.measured_yaw_dps - 50.0F) < 1e-3F);
    command.frame = Frame::Gimbal; command.gimbal_yaw_relative_deg = 90.0F;
    command.vy_mps = command.yaw_rate_dps = 0.0F;
    result = control.Update(command, true, true, {}, 10U, 0.001F);
    assert(std::fabs(result.requested_vx_mps) < 1e-5F && std::fabs(result.requested_vy_mps - 1.0F) < 1e-5F);
    command.mode = Mode::FollowGimbal;
    assert(control.Update(command, true, true, {}, 10U, 0.001F).requested_yaw_dps == 180.0F);
    command.mode = Mode::Spin;
    assert(control.Update(command, true, true, {}, 10U, 0.001F).requested_yaw_dps == 90.0F);
    command.mode = Mode::NoFollow;
    assert(control.Update(command, true, true, {}, 10U, 0.001F).requested_yaw_dps == 0.0F);
    command.mode = Mode::Disabled;
    assert(!control.Update(command, true, true, {}, 10U, 0.001F).output_requested);
    assert(control.Update(command, true, true, {}, 111U, 0.001F).stop_reason == StopReason::CommandStale);
    assert(control.Update(command, false, true, {}, 10U, 0.001F).stop_reason == StopReason::CommandStale);
    assert(control.Update(command, true, false, {}, 10U, 0.001F).stop_reason == StopReason::Safety);
    assert(control.Update(command, true, true, {}, 10U, 0.1F).stop_reason == StopReason::InvalidTiming);
    command.mode = static_cast<Mode>(255U);
    assert(control.Update(command, true, true, {}, 10U, 0.001F).stop_reason == StopReason::InvalidCommand);
    command.mode = Mode::Direct; command.source_tick_ms = UINT32_MAX - 9U;
    assert(control.Update(command, true, true, {}, 5U, 0.001F).output_requested);
    command.vx_mps = std::numeric_limits<float>::quiet_NaN();
    assert(!control.Update(command, true, true, {}, 5U, 0.001F).output_requested);
    assert(!Controller(Config{}).Update({}, true, true, {}, 0U, 0.001F).output_requested);
    auto singular = config;
    singular.drive_direction_deg = {}; // 四轮均沿x驱动，不能控制横向。
    assert(!Controller(singular).Update({}, true, true, {}, 0U, 0.001F).output_requested);
    config.max_wheel_output_dps = 100.0F;
    command = {Mode::Direct, Frame::Chassis, 1.0F, 1.0F, 90.0F, 0.0F, 10U, 1U};
    result = Controller(config).Update(command, true, true, {}, 10U, 0.001F);
    assert(result.output_requested && result.wheel_scale < 1.0F);
    for (float w : result.wheel_target_output_dps) assert(std::fabs(w) <= 100.001F);
}
