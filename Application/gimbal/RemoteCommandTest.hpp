#pragma once

#include <cstdint>

namespace gimbal
{
    struct RemoteCommandTestConfig
    {
        float yaw_stick = 0.5F;
        uint32_t initial_hold_ms = 1000U;
        uint32_t move_ms = 200U;
        uint32_t hold_ms = 2000U;
    };

    /** @brief 模拟解析后的归一化 yaw 摇杆，不伪造 DBUS 帧或在线状态。 */
    inline float GenerateRemoteYawCommand(uint32_t elapsed_ms,
                                          const RemoteCommandTestConfig& config)
    {
        uint64_t t = elapsed_ms;
        if (t < config.initial_hold_ms) return 0.0F;
        t -= config.initial_hold_ms;
        if (t < config.move_ms) return config.yaw_stick;
        t -= config.move_ms;
        if (t < config.hold_ms) return 0.0F;
        t -= config.hold_ms;
        if (t < config.move_ms) return -config.yaw_stick;
        return 0.0F;
    }

    /** @brief 一次触发一轮；取消/失联后须重新释放触发才可运行，不自动循环。 */
    class RemoteCommandTest
    {
    public:
        float Update(uint32_t now_ms, bool allowed, bool requested,
                     const RemoteCommandTestConfig& config)
        {
            if (!allowed)
            {
                active_ = armed_ = false;
                return 0.0F;
            }
            if (!requested)
            {
                active_ = false;
                armed_ = true;
                return 0.0F;
            }
            if (armed_)
            {
                armed_ = false;
                active_ = true;
                start_ms_ = now_ms;
            }
            if (!active_) return 0.0F;
            const uint32_t elapsed = now_ms - start_ms_;
            const uint64_t duration = static_cast<uint64_t>(config.initial_hold_ms) +
                2ULL * config.move_ms + 2ULL * config.hold_ms;
            if (elapsed >= duration)
            {
                active_ = false;
                return 0.0F;
            }
            return GenerateRemoteYawCommand(elapsed, config);
        }

    private:
        bool armed_ = false;
        bool active_ = false;
        uint32_t start_ms_ = 0U;
    };
}
