/**
 * @file angle_target_test.cpp
 * @brief MotorEncoderDelta / MotorAngleTarget 的宿主机 (host) 单元测试。
 *
 * 编译运行 (无 CMake 目标, 直接用 g++ 手动编译):
 *   g++ -std=c++17 -I Modules tests/angle_target_test.cpp -o build/angle_target_test
 *   && ./build/angle_target_test
 *
 * 覆盖场景:
 *   1. 13 位编码器跨 0/8191 边界的最短增量 (正反两个方向);
 *   2. 未标定行程时 Sync() 拒绝同步;
 *   3. 零点靠近编码器回绕处: 相对角度、行程内限幅、越界检测;
 *   4. 反馈方向反转 (feedback_reverse) 时相对零位方向随之反转;
 *   5. 反馈中断重锚 (epoch 变化) 后旧目标作废, 须重新 Sync()。
 */
#include <cassert>
#include <cmath>

#include "motor/angle_target.h"

int main()
{
    assert(MotorEncoderDelta(8190U, 2U) == 4);
    assert(MotorEncoderDelta(2U, 8190U) == -4);

    constexpr MotorAngleLimits uncalibrated{};
    MotorAngleTarget target{};
    assert(!target.Sync(uncalibrated, 360.0F, 2U, 1U, false));

    // 零点靠近编码器回绕处，限位仍应按零点的相对角度判断。
    constexpr MotorAngleLimits limits{true, false, 8190U, -10.0F, 20.0F};
    assert(target.Sync(limits, 360.09F, 2U, 1U, false));
    assert(target.Contains(limits, 360.09F));
    assert(std::fabs(target.Step(limits, 1.0F, 100.0F, 1.0F) -
                     (360.09F - 4.0F * 360.0F / 8192.0F + 20.0F)) < 0.001F);
    assert(!target.Contains(limits, 400.0F));
    assert(!target.Sync(limits, 400.0F, 3000U, 1U, false));

    // 反馈方向反转后，相对零位的方向也随之反转。
    constexpr MotorAngleLimits reverseLimits{true, false, 1000U, -5.0F, 5.0F};
    assert(target.Sync(reverseLimits, -44.0F, 1002U, 1U, true));
    assert(target.Contains(reverseLimits, -44.0F));

    // 反馈中断后由调用方按新的 epoch 重新同步，旧目标不能继续使用。
    target.Reset();
    assert(!target.Ready());
    assert(target.Sync(limits, 360.09F, 2U, 2U, false));
    assert(target.Epoch() == 2U);
    assert(std::fabs(target.Target() - 360.09F) < 0.001F);
    return 0;
}
