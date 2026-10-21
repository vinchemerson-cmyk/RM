/** 独立离线检查，不接入正式工程，不覆盖原有自检文件。 */
#include "ChassisDraftBus.hpp"
#include <cassert>
#include <cstring>
#include <limits>

// 最小消息接口替身，验证Endpoint接线，不代表真实消息中心并发验证。
static Subscriber_t sub{};
static Publisher_t pub{};
static chassis::draft::Command pending{};
static bool has_pending{};
static unsigned publish_count{}, registration_count{};
extern "C" Subscriber_t* SubRegister(char*, uint8_t) { ++registration_count; return &sub; }
extern "C" Publisher_t* PubRegister(char*, uint8_t) { ++registration_count; return &pub; }
extern "C" uint8_t SubGetMessage(Subscriber_t*, void* out)
{
    if (!has_pending) return 0U;
    std::memcpy(out, &pending, sizeof(pending)); has_pending = false; return 1U;
}
extern "C" uint8_t PubPushMessage(Publisher_t*, void*) { ++publish_count; return 1U; }

using namespace chassis::draft;
static Config Fixture()
{
    Config c{}; // 数学样例，不是实车参数。
    c.commissioned = true;
    c.wheel_radius_m = 0.075F; c.wheelbase_m = c.track_width_m = 0.4F;
    c.max_translation_mps = 3.0F; c.max_yaw_dps = 180.0F;
    c.max_wheel_output_dps = 3000.0F;
    c.follow_gain_per_s = 2.0F; c.spin_yaw_dps = 90.0F;
    c.drive_direction_deg = {135.0F, 45.0F, -135.0F, -45.0F};
    return c;
}

int main()
{
    auto c = Fixture();
    Controller controller(c);
    Command command{Mode::Direct, CommandFrame::Chassis, 1.0F, 0.0F, 0.0F, 0.0F, 10U, 1U};
    WheelFeedback feedback{{}, true, 10U};
    auto r = controller.Update(command, true, true, feedback, 10U, 0.001F);
    assert(r.output_requested && r.estimate_valid);
    assert(std::fabs(r.requested_vx_mps - 0.003F) < 1e-6F);
    assert(std::fabs(r.wheel_target_output_dps[FrontLeft] + 1.62057F) < 1e-4F);
    assert(std::fabs(r.wheel_target_output_dps[FrontRight] - 1.62057F) < 1e-4F);
    // 稳态目标正解还原，以及任意混合平移/自转正反解一致。
    for (unsigned i = 0U; i < 500U; ++i)
        r = controller.Update(command, true, true, feedback, 10U, 0.001F);
    assert(std::fabs(r.requested_vx_mps - 1.0F) < 1e-5F);
    feedback.output_speed_dps = r.wheel_target_output_dps;
    r = controller.Update(command, true, true, feedback, 10U, 0.001F);
    assert(std::fabs(r.measured_vx_mps - 1.0F) < 1e-5F);
    command.vy_mps = 0.5F; command.yaw_rate_dps = 50.0F;
    for (unsigned i = 0U; i < 500U; ++i)
        r = controller.Update(command, true, true, feedback, 10U, 0.001F);
    feedback.output_speed_dps = r.wheel_target_output_dps;
    r = controller.Update(command, true, true, feedback, 10U, 0.001F);
    assert(std::fabs(r.measured_vy_mps - 0.5F) < 1e-5F);
    assert(std::fabs(r.measured_yaw_dps - 50.0F) < 1e-3F);
    // 正常反向斜坡：每步平移向量变化不超过a*dt，角速度不超过alpha*dt。
    const float old_x = r.requested_vx_mps, old_y = r.requested_vy_mps, old_w = r.requested_yaw_dps;
    command.vx_mps = -2.0F; command.vy_mps = -1.0F; command.yaw_rate_dps = -180.0F;
    r = controller.Update(command, true, true, feedback, 10U, 0.001F);
    assert(std::hypot(r.requested_vx_mps - old_x, r.requested_vy_mps - old_y) <= 0.00301F);
    assert(std::fabs(r.requested_yaw_dps - old_w) <= 0.72001F);
    // 急停立即零输出，不按斜坡慢慢停；恢复从零起步。
    r = controller.Update(command, true, false, feedback, 10U, 0.001F);
    assert(!r.output_requested);
    for (float w : r.wheel_target_output_dps) assert(w == 0.0F);
    r = controller.Update(command, true, true, feedback, 10U, 0.001F);
    assert(std::hypot(r.requested_vx_mps, r.requested_vy_mps) <= 0.00301F);
    assert(!controller.Update(command, true, true, {}, 10U, 0.001F).output_requested);
    assert(!controller.Update(command, true, true, feedback, 111U, 0.001F).output_requested);
    command.source_tick_ms = 111U;
    assert(!controller.Update(command, true, true, feedback, 111U, 0.001F).output_requested); // 旧反馈。
    feedback.oldest_sample_tick_ms = 111U;
    assert(!controller.Update(command, true, true, feedback, 111U, 0.002F).output_requested); // 保留用户dt门限。
    assert(!controller.Update(command, true, true, feedback, 111U, 0.001F, 0.0F).output_requested);
    assert(!controller.Update(command, true, true, feedback, 111U, 0.001F, -1.0F).output_requested);
    command.vx_mps = std::numeric_limits<float>::quiet_NaN();
    assert(!controller.Update(command, true, true, feedback, 111U, 0.001F).output_requested);
    command = {Mode::FollowGimbal, CommandFrame::Gimbal, 1.0F, 0.0F, 0.0F, 0.5F, 111U, 2U};
    controller.Reset();
    r = controller.Update(command, true, true, feedback, 111U, 0.001F);
    assert(r.requested_yaw_dps == 0.0F); // 死区不来回跟随。
    command.gimbal_yaw_relative_deg = 90.0F;
    controller.Reset();
    r = controller.Update(command, true, true, feedback, 111U, 0.001F);
    assert(std::fabs(r.requested_vx_mps) < 1e-6F && r.requested_vy_mps > 0.0F);
    command.mode = Mode::NoFollow;
    controller.Reset();
    assert(controller.Update(command, true, true, feedback, 111U, 0.001F).requested_yaw_dps == 0.0F);
    command.mode = static_cast<Mode>(255U);
    assert(!controller.Update(command, true, true, feedback, 111U, 0.001F).output_requested);
    command.mode = Mode::Direct;
    command.source_tick_ms = UINT32_MAX - 9U; feedback.oldest_sample_tick_ms = command.source_tick_ms;
    assert(controller.Update(command, true, true, feedback, 5U, 0.001F).output_requested);
    assert(!Controller(Config{}).Update(command, true, true, feedback, 5U, 0.001F).output_requested);
    auto singular = c; singular.drive_direction_deg = {};
    assert(!Controller(singular).Update(command, true, true, feedback, 5U, 0.001F).output_requested);
    c.max_wheel_output_dps = 0.5F;
    Controller saturated(c);
    command = {Mode::Spin, CommandFrame::Chassis, 2.0F, 2.0F, 0.0F, 0.0F, 10U, 3U};
    feedback = {{}, true, 10U};
    r = saturated.Update(command, true, true, feedback, 10U, 0.001F);
    assert(r.wheel_scale < 1.0F);
    for (float w : r.wheel_target_output_dps) assert(std::fabs(w) <= 0.50001F);
    r = saturated.Update(command, true, true, feedback, 10U, 0.001F, 0.1F);
    for (float w : r.wheel_target_output_dps) assert(std::fabs(w) <= 0.05001F);
    // 总线端点：初始化幂等，无消息不输出，安全/反馈恢复不能复用故障前指令。
    BusEndpoint endpoint(Fixture()); endpoint.Init(); endpoint.Init();
    assert(registration_count == 2U);
    assert(!endpoint.Tick(true, feedback, 10U, 0.001F).output_requested);
    pending = command; has_pending = true;
    assert(endpoint.Tick(true, feedback, 10U, 0.001F).output_requested);
    assert(!endpoint.Tick(false, feedback, 10U, 0.001F).output_requested);
    assert(!endpoint.Tick(true, feedback, 10U, 0.001F).output_requested);
    has_pending = true;
    assert(endpoint.Tick(true, feedback, 10U, 0.001F).output_requested);
    assert(!endpoint.Tick(true, {}, 10U, 0.001F).output_requested);
    assert(!endpoint.Tick(true, feedback, 10U, 0.001F).output_requested);
    assert(publish_count == 7U);
}
