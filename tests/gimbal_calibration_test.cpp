/**
 * @file gimbal_calibration_test.cpp
 * @brief 验证实测云台标定、软限位及启用范围；不替代实车 PID/方向验证。
 */
#include <cassert>
#include <cmath>
#include <initializer_list>
#include <limits>

#include "gimbal/GimbalConfig.hpp"

FDCAN_HandleTypeDef hfdcan1{}, hfdcan2{}, hfdcan3{};

int main()
{
    assert(!kEncoderCalibrationOnly);
    assert(kEnablePitch && kPitchPositionMode);
    assert(kEnableYaw && kYawPositionMode && kReadYawEncoder);
    assert(kGimbalCanHandle == &hfdcan1);
    assert(kPitchAngleLimits.zero_raw == 893U);
    assert(kYawAngleLimits.zero_raw == 3741U);
    assert(kYawAngleLimits.IsConfigured() && kYawAngleLimits.continuous);
    assert(kPitchAngleLimits.IsConfigured() && !kPitchAngleLimits.continuous);
    // PID 由实车独立整定；此测试只检查合法性，不改动或固定某次参数。
    assert(std::isfinite(kPitchSpeedKp) && kPitchSpeedKp > 0.0F);
    assert(std::isfinite(kPitchSpeedKi) && kPitchSpeedKi >= 0.0F);
    assert(std::isfinite(kPitchSpeedKd) && kPitchSpeedKd >= 0.0F);

    const float sign = kPitchMotorReverse ? -1.0F : 1.0F;
    const float physicalTop = MotorEncoderDelta(kPitchZeroRaw, kPitchTopRaw) *
                              (360.0F / 8192.0F) * sign;
    const float physicalBottom = MotorEncoderDelta(kPitchZeroRaw, kPitchBottomRaw) *
                                 (360.0F / 8192.0F) * sign;
    assert(kPitchLimitMarginDeg == 10.0F);
    assert(std::fabs(kPitchAngleLimits.min_deg - physicalTop - kPitchLimitMarginDeg) < 0.001F);
    assert(std::fabs(kPitchAngleLimits.max_deg - physicalBottom + kPitchLimitMarginDeg) < 0.001F);
    assert(kPitchAngleLimits.min_deg < 0.0F && kPitchAngleLimits.max_deg > 0.0F);

    // 10°约为228格：上顶点120附近从347开始只能向零位，下顶点1490附近从1263开始只能向零位。
    assert(!kPitchMotorReverse);
    constexpr uint16_t topInside = 348U, topGuard = 347U;
    constexpr uint16_t bottomInside = 1262U, bottomGuard = 1263U;
    for (const uint16_t raw : {topInside, bottomInside})
    {
        assert(kPitchAngleLimits.AllowsSpeed(raw, kPitchMotorReverse, 1.0F));
        assert(kPitchAngleLimits.AllowsSpeed(raw, kPitchMotorReverse, -1.0F));
    }
    for (const uint16_t raw : {topGuard, kPitchTopRaw})
    {
        assert(kPitchAngleLimits.AllowsSpeed(raw, kPitchMotorReverse, 1.0F));
        assert(!kPitchAngleLimits.AllowsSpeed(raw, kPitchMotorReverse, -1.0F));
        assert(!kPitchAngleLimits.AllowsSpeed(raw, kPitchMotorReverse, 0.0F));
    }
    for (const uint16_t raw : {bottomGuard, kPitchBottomRaw})
    {
        assert(kPitchAngleLimits.AllowsSpeed(raw, kPitchMotorReverse, -1.0F));
        assert(!kPitchAngleLimits.AllowsSpeed(raw, kPitchMotorReverse, 1.0F));
        assert(!kPitchAngleLimits.AllowsSpeed(raw, kPitchMotorReverse, 0.0F));
    }
    assert(kPitchAngleLimits.AllowsSpeed(kPitchZeroRaw, kPitchMotorReverse, 1.0F));
    assert(kPitchAngleLimits.AllowsSpeed(kPitchZeroRaw, kPitchMotorReverse, -1.0F));
    assert(kPitchAngleLimits.AllowsSpeed(kPitchZeroRaw, kPitchMotorReverse, 0.0F));
    assert(!kPitchAngleLimits.AllowsSpeed(8192U, kPitchMotorReverse, 1.0F));
    assert(!kPitchAngleLimits.AllowsSpeed(kPitchZeroRaw, kPitchMotorReverse,
                                        std::numeric_limits<float>::quiet_NaN()));
    constexpr MotorAngleLimits uncalibrated{};
    assert(!uncalibrated.AllowsSpeed(893U, false, 1.0F));
    constexpr MotorAngleLimits reversedLimits{true, false, 893U, -25.7958984375F, 33.57421875F};
    assert(reversedLimits.AllowsSpeed(129U, true, -1.0F));
    assert(!reversedLimits.AllowsSpeed(129U, true, 1.0F));
    assert(reversedLimits.AllowsSpeed(1480U, true, 1.0F));
    assert(!reversedLimits.AllowsSpeed(1480U, true, -1.0F));
    assert(kYawAngleLimits.AllowsSpeed(2U, kYawMotorReverse, -180.0F));

    MotorAngleTarget target{};
    const float zeroTotal = 720.0F; // 连续角度坐标可有任意整圈偏移。
    assert(target.Sync(kPitchAngleLimits, zeroTotal, kPitchZeroRaw, 1U, kPitchMotorReverse));
    assert(target.Target() == zeroTotal);
    assert(target.Contains(kPitchAngleLimits, zeroTotal + kPitchAngleLimits.min_deg));
    assert(target.Contains(kPitchAngleLimits, zeroTotal + kPitchAngleLimits.max_deg));
    assert(!target.Contains(kPitchAngleLimits, zeroTotal + physicalTop));
    assert(!target.Contains(kPitchAngleLimits, zeroTotal + physicalBottom));
    assert(std::fabs(target.Step(kPitchAngleLimits, 1.0F, kPitchTargetRateDps, 10.0F) -
                     (zeroTotal + kPitchAngleLimits.max_deg)) < 0.001F);
    assert(std::fabs(target.Step(kPitchAngleLimits, -1.0F, kPitchTargetRateDps, 10.0F) -
                     (zeroTotal + kPitchAngleLimits.min_deg)) < 0.001F);

    // 解锁时保持当前实测位置；软件限位之外的物理端点不能作为位置目标同步。
    constexpr uint16_t interiorRaw = 1000U;
    const float interiorTotal = zeroTotal + MotorEncoderDelta(kPitchZeroRaw, interiorRaw) *
                               (360.0F / 8192.0F) * sign;
    assert(target.Sync(kPitchAngleLimits, interiorTotal, interiorRaw, 2U, kPitchMotorReverse));
    assert(target.Target() == interiorTotal);
    assert(!target.Sync(kPitchAngleLimits, zeroTotal + physicalTop, kPitchTopRaw,
                        3U, kPitchMotorReverse));
    assert(!target.Sync(kPitchAngleLimits, zeroTotal + physicalBottom, kPitchBottomRaw,
                        3U, kPitchMotorReverse));
    assert(target.Sync(kPitchAngleLimits, zeroTotal, kPitchZeroRaw, 4U, kPitchMotorReverse));

    // yaw 连续旋转不做±180°裁剪，也不因原始编码器跨零而自动回正前方。
    assert(target.Sync(kYawAngleLimits, 720.0F, 8190U, 4U, kYawMotorReverse));
    assert(target.Target() == 720.0F);
    assert(std::fabs(target.Step(kYawAngleLimits, 1.0F, kYawTargetRateDps, 10.0F) -
                     2520.0F) < 0.001F);
    assert(target.Contains(kYawAngleLimits, 2520.0F));
    assert(std::fabs(target.Step(kYawAngleLimits, -1.0F, kYawTargetRateDps, 20.0F) +
                     1080.0F) < 0.001F);
    return 0;
}
