/** @file ChassisDraftBus.hpp @brief 板内发布订阅草稿，不创建任务或调用电机。 */
#pragma once
#include <type_traits>
#include "ChassisDraft.hpp"
#include "message_center/message_center.h"

namespace chassis::draft
{
inline char kCommandTopic[] = "chassis_draft_cmd";
inline char kStateTopic[] = "chassis_draft_state";
static_assert(std::is_trivially_copyable_v<Command> && sizeof(Command) <= 255U);
static_assert(std::is_trivially_copyable_v<Result> && sizeof(Result) <= 255U);

class BusEndpoint
{
public:
    explicit BusEndpoint(const Config& config) : controller_(config) {}
    // 调度器启动前调用一次，message_center注册不用于中断或运行时并发修改。
    void Init()
    {
        if (subscriber_ != nullptr) return;
        subscriber_ = SubRegister(kCommandTopic, static_cast<uint8_t>(sizeof(Command)));
        publisher_ = PubRegister(kStateTopic, static_cast<uint8_t>(sizeof(Result)));
    }

    // 周期任务提供实时安全许可、统一输出轴反馈及实际dt。
    // 调用方必须将每次返回的停止结果落实到电机，不能仅在有新消息时发指令。
    Result Tick(bool safety_allowed, const WheelFeedback& feedback, uint32_t now_ms, float dt_s,
                float speed_scale = 1.0F)
    {
        if (subscriber_ == nullptr || publisher_ == nullptr) return {};
        if (SubGetMessage(subscriber_, &latest_) != 0U) have_command_ = true;
        Result result = controller_.Update(latest_, have_command_, safety_allowed, feedback, now_ms, dt_s, speed_scale);
        // 安全撤销、超时、反馈或时序异常后废弃旧指令；恢复须有新消息和外部安全解锁。
        if (!result.output_requested) have_command_ = false;
        (void)PubPushMessage(publisher_, &result);
        return result;
    }
private:
    Controller controller_;
    Subscriber_t* subscriber_{};
    Publisher_t* publisher_{};
    Command latest_{};
    bool have_command_{};
};
}
