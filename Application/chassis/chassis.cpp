/** @file chassis.cpp @brief 四全向轮独立控制模块；硬件留空时禁止注册，尚未接入Robot。 */
#include "chassis.h"
#include "ChassisConfig.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_log.h"
#include "safety/safety.h"
#include <limits>

namespace
{
	struct ChassisSingleMotor
    {
        DJI_Motor_s* motor{};
        chassis::WheelControlSnapshot snapshot{};
    };
    std::array<ChassisSingleMotor, chassis::kWheelCount> wheels{};
    chassis::WheelVelocityPlanner planner{};
    osThreadId_t taskHandle{};
    bool registrationAttempted{};
    uint32_t periodTicks{};
    constexpr osThreadAttr_t taskAttrs{
        .name = "ChassisTask", .stack_size = 2048U, .priority = osPriorityHigh,
    };

    float Deadband(float stick)
    {
        if (std::fabs(stick) <= kChassisStickDeadband) return 0.0F;
        return std::copysign((std::fabs(stick) - kChassisStickDeadband) /
                            (1.0F - kChassisStickDeadband), stick);
    }

    bool Fresh(uint32_t now, uint32_t sample, uint32_t timeoutMs)
    {
        return static_cast<uint64_t>(now - sample) * 1000U <
               static_cast<uint64_t>(timeoutMs) * osKernelGetTickFreq();
    }

    DJI::Motor_Init_Config_s MakeChassisMotorConfig(const ChassisWheelHardware& hardware)
    {
        DJI::Motor_Init_Config_s config{};
        config.motor_type = hardware.type;
        config.controller_param_init_config.speed_PID = kChassisWheelSpeedPi;
        auto& setting = config.controller_setting_init_config;
        setting.close_loop_type = SPEED_AND_CURRENT_LOOP;
        setting.outer_loop_type = SPEED_LOOP;
        setting.motor_reverse_flag = hardware.reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
        setting.angle_feedback_source = MOTOR_FEED;
        setting.speed_feedback_source = MOTOR_FEED;
        setting.feedforward_flag = FEEDFORWARD_NONE;
        config.can_init_config.can_handle = kChassisCanHandle;
        config.can_init_config.can_id = hardware.feedback_id;
        return config;
    }

    void StopWheels()
    {
        planner.Reset();
        for (auto& wheel : wheels) DJI_Motor_Disable(wheel.motor); // 驱动允许空指针。
    }

    void PublishWheelControlSnapshot(unsigned index, uint32_t tick, float dt)
    {
        auto& wheel = wheels[index];
        if (wheel.motor == nullptr) return;
        chassis::WheelControlSnapshot snapshot{};
        const auto& motor = *wheel.motor;
        taskENTER_CRITICAL();
        const auto feedback = motor.feedback;
        snapshot.control_sequence = wheel.snapshot.control_sequence + 1U;
        taskEXIT_CRITICAL();
        snapshot.control_tick = tick;
        snapshot.control_dt_s = dt;
        snapshot.target_speed_dps = motor.enabled
            ? motor.motor_settings.speed / kChassisWheels[index].gear_ratio : 0.0F;
        snapshot.actual_speed_dps = feedback.speed / kChassisWheels[index].gear_ratio;
        snapshot.command_current_raw = motor.enabled ? motor.controller.speed_PID.Output() : 0.0F;
        snapshot.feedback_current_a = feedback.current;
        snapshot.output_enabled = motor.enabled;
        snapshot.feedback_seen = feedback.angle_initialized;
        snapshot.feedback_tick = feedback.last_update_tick;
        snapshot.tx_attempts = motor.tx_enqueue_attempts;
        snapshot.tx_failures = motor.tx_enqueue_failures;
        taskENTER_CRITICAL();
        wheel.snapshot = snapshot;
        taskEXIT_CRITICAL();
    }

    bool FeedbackReady(uint32_t tick)
    {
        bool ready = true;
        taskENTER_CRITICAL();
        for (const auto& wheel : wheels)
        {
            if (wheel.motor == nullptr) { ready = false; break; }
            const auto& feedback = wheel.motor->feedback;
            if (!feedback.angle_initialized || !std::isfinite(feedback.speed) ||
                !Fresh(tick, feedback.last_update_tick, kChassisFeedbackTimeoutMs))
            { ready = false; break; }
        }
        taskEXIT_CRITICAL();
        return ready;
    }

    void Task(void*)
    {
        uint32_t lastTick = osKernelGetTickCount();
        uint32_t wakeTick = lastTick;
        bool txFault = false;
        DJI_Motor_s* motors[chassis::kWheelCount]{};
        for (unsigned i = 0; i < chassis::kWheelCount; ++i) motors[i] = wheels[i].motor;

        for (;;)
        {
            // 绝对调度；若工作已跨过截止时间，重新对齐，避免连续补跑。
            wakeTick += periodTicks;
            const uint32_t beforeWait = osKernelGetTickCount();
            if (static_cast<int32_t>(wakeTick - beforeWait) <= 0) wakeTick = beforeWait + periodTicks;
            const bool waited = osDelayUntil(wakeTick) == osOK;
            const uint32_t now = osKernelGetTickCount();
            const float dt = static_cast<float>(now - lastTick) / osKernelGetTickFreq();
            lastTick = now;
            const bool timingValid = waited && std::isfinite(dt) && dt > 0.0F && dt <= kChassisMaxDtS;
            const auto remote = rc::Snapshot();
            const bool remoteValid = remote.received && Fresh(now, remote.last_update_tick, rc::OFFLINE_TIMEOUT_MS);
            const bool runRequested = remote.switch_right == kChassisRunSwitch;
            // 发送故障须先离开运行拨杆位置再进入，不在故障后自动续转。
            if (remoteValid && !runRequested) txFault = false;
            bool allowed = timingValid && remoteValid && runRequested && !txFault &&
                           safety::ShouldOutput() && FeedbackReady(now);
            std::array<float, chassis::kWheelCount> target{};
            if (allowed)
            {
                const float vx = Deadband(rc::Normalize(remote.channels[kChassisForwardChannel])) *
                                 kChassisForwardSign * kChassisMotionLimits.max_translation_mps;
                const float vy = Deadband(rc::Normalize(remote.channels[kChassisLeftChannel])) *
                                 kChassisLeftSign * kChassisMotionLimits.max_translation_mps;
                const float yaw = Deadband(rc::Normalize(remote.channels[kChassisYawChannel])) *
                                  kChassisYawSign * kChassisMotionLimits.max_yaw_dps;
                allowed = planner.Update(vx, vy, yaw, dt, target);
            }
            if (!allowed) StopWheels();
            else
            {
                for (unsigned i = 0; i < chassis::kWheelCount; ++i)
                {
                    auto& motor = *motors[i];
                    if (!motor.enabled) DJI_Motor_Enable(&motor); // 只在状态切换时清PI。
                    motor.motor_settings.speed = target[i] * kChassisWheels[i].gear_ratio;
                    motor.motor_settings.current = 0.0F;
                }
            }
            uint32_t failures[chassis::kWheelCount]{};
            for (unsigned i = 0; i < chassis::kWheelCount; ++i) failures[i] = motors[i]->tx_enqueue_failures;
            // 停止也发送零指令；所有同组电机在一次调用中提交。
            DJI_Motor_Control(motors, chassis::kWheelCount, timingValid ? dt : kChassisControlPeriodS);
            for (unsigned i = 0; i < chassis::kWheelCount; ++i)
                if (motors[i]->tx_enqueue_failures != failures[i]) txFault = true;
            if (txFault)
            {
                StopWheels();
                // 尽力撤销可能已入队的其他分组输出；最多补发一次，不阻塞周期。
                DJI_Motor_Control(motors, chassis::kWheelCount, kChassisControlPeriodS);
            }
            for (unsigned i = 0; i < chassis::kWheelCount; ++i) PublishWheelControlSnapshot(i, now, dt);
            if (!waited) osDelay(periodTicks); // API异常时也不允许高优先级空转。
        }
    }

    bool ValidateConfiguration()
    {
        if (!kChassisHardwareConfirmed || kChassisCanHandle == nullptr ||
            kChassisCanHandle->Init.FrameFormat != FDCAN_FRAME_CLASSIC ||
            !chassis::ControlPeriodTicks(kChassisControlPeriodS, osKernelGetTickFreq(), periodTicks) ||
            !std::isfinite(kChassisMaxDtS) || kChassisMaxDtS < kChassisControlPeriodS ||
            kChassisFeedbackTimeoutMs == 0U || kChassisFeedbackTimeoutMs >= 0x80000000U ||
            kChassisForwardChannel >= rc::CHANNEL_COUNT || kChassisLeftChannel >= rc::CHANNEL_COUNT ||
            kChassisYawChannel >= rc::CHANNEL_COUNT ||
            !std::isfinite(kChassisStickDeadband) || kChassisStickDeadband < 0.0F || kChassisStickDeadband >= 1.0F)
            return false;
        for (float sign : {kChassisForwardSign, kChassisLeftSign, kChassisYawSign})
            if (sign != 1.0F && sign != -1.0F) return false;
        if (kChassisRunSwitch != rc::Right_Switch_e::UP && kChassisRunSwitch != rc::Right_Switch_e::MIDDLE)
            return false;
        PIDInstance pi;
        if (!pi.Configure(kChassisWheelSpeedPi) || kChassisWheelSpeedPi.kd != 0.0F ||
            kChassisWheelSpeedPi.kp <= 0.0F || kChassisWheelSpeedPi.ki < 0.0F) return false;
        for (unsigned i = 0; i < chassis::kWheelCount; ++i)
        {
            const auto& hardware = kChassisWheels[i];
            if ((hardware.type != DJI::M3508 && hardware.type != DJI::M2006) ||
                hardware.feedback_id < 0x201U || hardware.feedback_id > 0x208U ||
                !std::isfinite(hardware.gear_ratio) || hardware.gear_ratio <= 0.0F) return false;
            const float peakRotorSpeed = hardware.gear_ratio * kChassisGeometry[i].max_output_speed_dps;
            if (!std::isfinite(peakRotorSpeed) || peakRotorSpeed > std::numeric_limits<int16_t>::max() * 6.0F)
                return false;
            const float commandLimit = hardware.type == DJI::M3508 ? 16384.0F : 10000.0F;
            if (kChassisWheelSpeedPi.output_max > commandLimit || kChassisWheelSpeedPi.output_min < -commandLimit)
                return false;
            for (unsigned j = 0; j < i; ++j)
                if (hardware.feedback_id == kChassisWheels[j].feedback_id) return false;
        }
        return planner.Configure(kChassisGeometry, kChassisMotionLimits);
    }
}

bool chassis::Init()
{
    if (taskHandle != nullptr) return true;
    if (registrationAttempted || osKernelGetState() != osKernelReady)
        return false;
    if (!ValidateConfiguration())
    {
        LOGERROR("[chassis] hardware/configuration incomplete; no motors registered");
        return false;
    }
    registrationAttempted = true;
    for (unsigned i = 0; i < kWheelCount; ++i)
    {
        wheels[i].motor = DJI_Motor_Init(MakeChassisMotorConfig(kChassisWheels[i]));
        if (wheels[i].motor == nullptr)
        {
            StopWheels();
            LOGERROR("[chassis] wheel %u registration failed; task not created", i);
            return false; // 驱动池无注销接口，不重试以免重复占用。
        }
    }
    taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
    if (taskHandle == nullptr)
    {
        StopWheels();
        LOGERROR("[chassis] task creation failed; motors remain disabled");
        return false;
    }
    return true;
}

chassis::WheelControlSnapshot chassis::GetWheelControlSnapshot(Wheel wheel)
{
    const unsigned index = static_cast<unsigned>(wheel);
    if (index >= kWheelCount) return {};
    taskENTER_CRITICAL();
    const auto snapshot = wheels[index].snapshot;
    taskEXIT_CRITICAL();
    return snapshot;
}
