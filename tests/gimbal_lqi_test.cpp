/** @file gimbal_lqi_test.cpp @brief 对真实云台更新函数验证 LQI、启动求解和端点保护。 */
#include <cassert>
#include <cmath>
#include <initializer_list>
#include "../Application/gimbal/gimbal.cpp"

FDCAN_HandleTypeDef hfdcan1{}, hfdcan2{}, hfdcan3{};
static DJI_Motor_s registered[2]{};
static unsigned motorCount{}, threadCount{}, logCount{};

void TestLog(const char*, ...) { ++logCount; }
void Error_Handler() { assert(false); }
uint32_t osKernelGetTickCount() { return 0U; }
osThreadId_t osThreadNew(void (*)(void*), void*, const osThreadAttr_t*)
{
    ++threadCount;
    return &threadCount;
}
void osDelay(uint32_t) {}
void osDelayUntil(uint32_t) {}
namespace rc
{
    RC_Data_s Snapshot() { return {}; }
    bool IsOnline() { return false; }
    float Normalize(uint16_t) { return 0.0F; }
}
namespace safety { bool ShouldOutput() { return false; } }

DJI_Motor_s* DJI_Motor_Init(const Motor_Init_Config_s& config)
{
    assert(motorCount < 2U);
    assert(config.controller_setting_init_config.close_loop_type == CURRENT_LOOP);
    assert(config.controller_param_init_config.speed_PID.kp == 0.0F);
    assert(config.controller_param_init_config.angle_PID.kp == 0.0F);
    auto& motor = registered[motorCount++];
    motor.motor_settings = config.controller_setting_init_config;
    motor.motor_type = config.motor_type;
    return &motor;
}
void DJI_Motor_Enable(DJI_Motor_s* motor) { motor->enabled = true; }
void DJI_Motor_Disable(DJI_Motor_s* motor)
{
    if (motor == nullptr) return;
    motor->enabled = false;
    motor->motor_settings.current = motor->motor_settings.angle = motor->motor_settings.speed = 0.0F;
}
void DJI_Motor_Control(DJI_Motor_s**, uint8_t, float) {}

int main()
{
    // 对照真实 PID 算法的未限幅比例项，检验 A->CAN 指令码换算等效。
    const auto checkLegacyProportional = [](const LQI_Init_Config_s& lqiConfig,
                                           const PID_Init_Config_s& angleConfig, float speedKp)
    {
        PIDInstance anglePid{}, speedPid{};
        assert(anglePid.Configure(angleConfig));
        assert(speedPid.Configure({speedKp, 0.0F, 0.0F,
            -kGimbalOutputLimit, kGimbalOutputLimit, 0.0F, 0.0F}));
        LQIController lqi{};
        assert(lqi.Configure(lqiConfig));
        for (const float target : {1.0F, -1.0F})
        {
            anglePid.Reset(); speedPid.Reset(); lqi.Reset();
            const float legacy = speedPid.Update(anglePid.Update(target, 0.0F, 0.001F),
                                                   2.0F, 0.001F);
            const float currentCode = lqi.Update(target, 0.0F, 2.0F, 0.001F) *
                                      kDjiGm6020CommandUnitsPerAmp;
            assert(std::fabs(currentCode - legacy) < 1e-5F * (1.0F + std::fabs(legacy)));
        }
    };
    checkLegacyProportional(kPitchLqiConfig, kPitchAnglePid, kPitchSpeedKp);
    // yaw 仅速度反馈，位置与积分不参与；静止打满输出 0.18A。
    LQIController yawCheck{};
    assert(yawCheck.Configure(kYawLqiConfig));
    for (const float speed : {0.0F, 2.0F, -2.0F})
    {
        const float yawCurrent = yawCheck.UpdateRecoverySpeed(60.0F, speed, 0.001F);
        assert(std::fabs(yawCurrent - 0.003F * (60.0F - speed)) < 1e-6F);
        assert(yawCheck.Integral() == 0.0F);
    }
    // 当前手填模式忽略占位模型；启动装载两轴 K，但未解锁前不使能电机。
    gimbal::Init();
    assert(motorCount == 2U && threadCount == 1U && logCount == 2U);
    assert(yawLqi.IsConfigured() && pitchLqi.IsConfigured());
    assert(!yawMotor->enabled && !pitchMotor->enabled);
    gimbal::Init();
    assert(motorCount == 2U && threadCount == 1U && logCount == 2U);

    // 手填 K 确实作用于电流，而不是被零模型或 Q/R 覆盖。
    LQIController manual{};
    InitializeLqi("manual", kPitchLqiModel, kPitchLqiConfig, manual, GimbalLqiGainSource::Manual);
    assert(manual.IsConfigured());
    const float expectedManual = std::fmin(kPitchLqiConfig.output_max_a,
        std::fmax(kPitchLqiConfig.output_min_a, 2.0F * kPitchLqiConfig.k_angle));
    assert(std::fabs(manual.Update(2.0F, 0.0F, 0.0F, 0.001F) - expectedManual) < 1e-6F);
    auto invalidManual = kPitchLqiConfig;
    invalidManual.k_angle = invalidManual.k_speed = invalidManual.k_integral = 0.0F;
    InitializeLqi("manual", kPitchLqiModel, invalidManual, manual, GimbalLqiGainSource::Manual);
    assert(!manual.IsConfigured());
    // 切回求解模式时，零模型仍须拒绝；不会自动回退到手填 K。
    InitializeLqi("solve", kPitchLqiModel, kPitchLqiConfig, manual, GimbalLqiGainSource::BootSolve);
    assert(!manual.IsConfigured());

    // 合法模型只用作宿主机验证，不写入实车配置。
    const LQI_Model_s model{{{1, 0.001}, {0, 1}}, {0.00005, 0.1}, 0.001};
    auto config = kPitchLqiConfig;
    config.integral_min_deg_s = -10.0F; config.integral_max_deg_s = 10.0F;
    InitializeLqi("pitch", model, config, pitchLqi, GimbalLqiGainSource::BootSolve);
    assert(pitchLqi.IsConfigured());
    auto& motor = *pitchMotor;
    motor.feedback.angle_initialized = true;
    motor.feedback.angle_epoch = 1U;
    motor.feedback.raw_angle = kPitchZeroRaw;
    motor.feedback.total_angle = 0.0F;
    UpdateAxis(&motor, pitchTarget, pitchLqi, true, kPitchAngleLimits, 0.0F, 60.0F, 0.001F);
    assert(motor.enabled && motor.motor_settings.current == 0.0F && pitchTarget.Ready());
    UpdateAxis(&motor, pitchTarget, pitchLqi, true, kPitchAngleLimits, 1.0F, 60.0F, 0.001F);
    assert(motor.motor_settings.close_loop_type == CURRENT_LOOP);
    assert(motor.motor_settings.current > 0.0F);
    assert(std::fabs(gimbal::GetEncoderSnapshot().pitch_position_target_deg - 0.06F) < 1e-6F);
    UpdateAxis(&motor, pitchTarget, pitchLqi, false, kPitchAngleLimits, 1.0F, 60.0F, 0.001F);
    assert(!motor.enabled && motor.motor_settings.current == 0.0F);
    assert(!pitchTarget.Ready() && pitchLqi.Integral() == 0.0F);

    // 上端点：向外/松杆零输出，向内用 LQI K_speed，电流码正确限幅。
    motor.feedback.raw_angle = kPitchTopRaw;
    motor.feedback.total_angle = kPitchTopDeg;
    UpdateAxis(&motor, pitchTarget, pitchLqi, true, kPitchAngleLimits, -1.0F, 60.0F, 0.001F);
    assert(!motor.enabled && motor.motor_settings.current == 0.0F);
    UpdateAxis(&motor, pitchTarget, pitchLqi, true, kPitchAngleLimits, 1.0F, 60.0F, 0.001F);
    assert(motor.enabled && motor.motor_settings.current > 0.0F);
    assert(motor.motor_settings.current <= kGimbalOutputLimit && !pitchTarget.Ready());
    assert(pitchLqi.Integral() == 0.0F);
    UpdateAxis(&motor, pitchTarget, pitchLqi, true, kPitchAngleLimits, 0.0F, 60.0F, 0.001F);
    assert(!motor.enabled && motor.motor_settings.current == 0.0F);

    // 下端点方向相反；回到安全区后同步当前位置，而不是追逐旧目标。
    motor.feedback.raw_angle = kPitchBottomRaw;
    motor.feedback.total_angle = kPitchBottomDeg;
    UpdateAxis(&motor, pitchTarget, pitchLqi, true, kPitchAngleLimits, 1.0F, 60.0F, 0.001F);
    assert(!motor.enabled);
    UpdateAxis(&motor, pitchTarget, pitchLqi, true, kPitchAngleLimits, -1.0F, 60.0F, 0.001F);
    assert(motor.enabled && motor.motor_settings.current < 0.0F);
    motor.feedback.raw_angle = 1000;
    motor.feedback.total_angle = (1000 - kPitchZeroRaw) * 360.0F / 8192.0F;
    UpdateAxis(&motor, pitchTarget, pitchLqi, true, kPitchAngleLimits, 0.0F, 60.0F, 0.001F);
    assert(pitchTarget.Ready() && pitchTarget.Target() == motor.feedback.total_angle);
    assert(motor.motor_settings.current == 0.0F);
    ++motor.feedback.angle_epoch;
    motor.feedback.total_angle += 360.0F;
    UpdateAxis(&motor, pitchTarget, pitchLqi, true, kPitchAngleLimits, 0.0F, 60.0F, 0.001F);
    assert(motor.motor_settings.current == 0.0F && pitchLqi.Integral() == 0.0F);

    InitializeLqi("yaw", model, config, yawLqi, GimbalLqiGainSource::BootSolve);
    yawMotor->feedback.angle_initialized = true;
    yawMotor->feedback.angle_epoch = 1;
    yawMotor->feedback.total_angle = 3600.0F;
    UpdateAxis(yawMotor, yawTarget, yawLqi, true, kYawAngleLimits, 1.0F, 180.0F, 0.001F);
    assert(yawMotor->enabled && yawTarget.Target() > 3600.0F);
    assert(yawMotor->motor_settings.current > 0.0F);
    const auto logs = logCount;
    for (int i = 0; i < 10; ++i)
        UpdateAxis(yawMotor, yawTarget, yawLqi, true, kYawAngleLimits, 0.0F, 180.0F, 0.001F);
    assert(logCount == logs); // 实时更新不重复进行启动求解和初始化日志。
    const auto yawSnapshot = gimbal::GetEncoderSnapshot();
    assert(yawSnapshot.yaw_position_target_deg == yawMotor->motor_settings.angle);
    assert(yawSnapshot.yaw_position_actual_deg == yawMotor->feedback.total_angle);
    assert(yawSnapshot.yaw_speed_dps == yawMotor->feedback.speed);
    assert(std::fabs(yawSnapshot.yaw_command_current_a -
        yawMotor->motor_settings.current / kDjiGm6020CommandUnitsPerAmp) < 1e-6F);
    UpdateAxis(yawMotor, yawTarget, yawLqi, false, kYawAngleLimits, 0.0F, 180.0F, 0.001F);
    assert(gimbal::GetEncoderSnapshot().yaw_command_current_a == 0.0F);
    assert(!kYawPositionMode && yawLqi.Configure(kYawLqiConfig));
    yawMotor->feedback.speed = 0.0F;
    for (int i = 0; i < 1000; ++i)
        UpdateAxis(yawMotor, yawTarget, yawLqi, true, kYawAngleLimits,
                   1.0F, kYawSpeedTestMaxDps, 0.001F, false);
    assert(!yawTarget.Ready()); // 长时间打杆不会积压角度目标。
    assert(std::fabs(gimbal::GetEncoderSnapshot().yaw_target_speed_dps - 60.0F) < 1e-6F);
    assert(std::fabs(gimbal::GetEncoderSnapshot().yaw_command_current_a - 0.18F) < 1e-6F);
    yawMotor->feedback.total_angle += 360.0F;
    UpdateAxis(yawMotor, yawTarget, yawLqi, true, kYawAngleLimits, 0.0F, 60.0F, 0.001F, false);
    assert(yawMotor->motor_settings.current == 0.0F); // 松杆静止后不追逐旧角度。
    for (const float speed : {-630.0F, 630.0F})
    {
        yawMotor->feedback.speed = speed;
        UpdateAxis(yawMotor, yawTarget, yawLqi, true, kYawAngleLimits, 0.0F, 60.0F, 0.001F, false);
        const float current = gimbal::GetEncoderSnapshot().yaw_command_current_a;
        assert(std::fabs(std::fabs(current) - 0.5F) < 1e-6F && current * speed < 0.0F);
        assert(yawLqi.Integral() == 0.0F);
    }
    UpdateAxis(yawMotor, yawTarget, yawLqi, false, kYawAngleLimits, 1.0F, 60.0F, 0.001F, false);
    assert(!yawMotor->enabled && yawMotor->motor_settings.current == 0.0F);
    assert(gimbal::GetEncoderSnapshot().yaw_target_speed_dps == 0.0F);
    yawMotor->feedback.angle_initialized = false;
    UpdateAxis(yawMotor, yawTarget, yawLqi, true, kYawAngleLimits, 1.0F, 60.0F, 0.001F, false);
    assert(!yawMotor->enabled);
    return 0;
}
