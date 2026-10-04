/** @file output_interlock.h @brief 上电/失联后用双拨杆 DOWN、摇杆居中解锁输出。 */
#pragma once
#include <cstdint>
namespace safety
{
class OutputInterlock
{
public:
    /** @brief 急停立即生效；在线、摇杆居中且双 DOWN 连续三次后允许解除急停。 */
    bool Update(bool online, bool both_down, bool sticks_neutral)
    {
        if (!online)
        {
            armed_ = false;
            down_cycles_ = 0U;
            return false;
        }
        if (both_down)
        {
            if (!sticks_neutral)
                down_cycles_ = 0U;
            else if (down_cycles_ < 3U)
                ++down_cycles_;
            if (down_cycles_ >= 3U)
                armed_ = true;
            return false;
        }
        down_cycles_ = 0U;
        return armed_;
    }
    [[nodiscard]] bool Armed() const { return armed_; }
private:
    bool armed_{};
    uint8_t down_cycles_{};
};
}
