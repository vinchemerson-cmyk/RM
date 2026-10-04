/**
 * @file gimbal.cpp
 * @brief 用右摇杆控制云台 yaw/pitch 两轴 GM6020 的速度或位置目标。
 *
 * 控制管线 (GimbalTask, 周期约 1ms):
 *   1. 计算 dt: 调度抖动被钳位到 (0, kGimbalMaxDtS], 异常大的 dt 不进入
 *      PID 积分/微分, 避免积分尖峰把电机打出极限。
 *   2. outputAllowed = rc::IsOnline() && safety::ShouldOutput(); 任一条件
 *      不满足则两轴 Disable、目标复位 (安全侧默认, 重新使能后重新同步)。
 *   3. UpdateAxis(): 每轴按当前模式改写 motor_settings:
 *        - 速度模式: close_loop_type = SPEED_AND_CURRENT_LOOP,
 *          speed = 摇杆值 * kXxxMaxSpeedDps (度每秒);
 *        - 位置模式: close_loop_type = ANGLE_AND_SPEED_LOOP,
 *          MotorAngleTarget 以 kXxxTargetRateDps 速率积分角度目标并做
 *          行程限幅; 反馈重锚 (angle_epoch 变化, 见 dji 驱动) 时目标
 *          自动同步到当前实测角度, 防止重新追逐失联前的旧目标。
 *   4. DJI_Motor_Control(): 由 dji 驱动计算 PID, 把两轴打包进同一帧
 *      0x1FE 电流指令发出；电机自身电流环必须开启。
 *
 * 线程模型:
 *   - 本任务是 motor_settings 的唯一写者, 无需加锁;
 *   - 电机反馈由 FDCAN 中断填充, 读取时用临界区做快照
 *     (FeedbackSnapshot), 避免读到"角度已更新而转速未更新"的混合值;
 *   - 指令帧在任务上下文发送, CAN 发送 FIFO 满时由 dji 驱动自然丢弃,
 *     下个周期重发。
 */

#include "gimbal.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "motor/dji/dji.h"
#include "rc/rc.h"
#include "safety/safety.h"

#include "GimbalConfig.hpp"

namespace
{
    osThreadId_t taskHandle = nullptr;

    DJI_Motor_s* yawMotor   = nullptr;
    DJI_Motor_s* pitchMotor = nullptr;

    /** @brief 两轴电机句柄数组, 交给 DJI 驱动逐周期打包下发。 */
    DJI_Motor_s* gimbalMotors[2] = {nullptr, nullptr};

    /** @brief 角度目标在失联或重新使能时会同步到当前反馈。 */
    MotorAngleTarget yawTarget{};
    MotorAngleTarget pitchTarget{};

    constexpr osThreadAttr_t taskAttrs = {
        .name       = "GimbalTask",
        .stack_size = 2048U,
        .priority   = osPriorityHigh,
    };

    /**
 * @brief 构造一个云台电机的初始化配置。
 * @param motor_type 仅用于标记型号, 必须为 GM6020。
 * @param feedback_id 该电机的反馈 ID, 决定它在指令帧中的槽位。
 * @param kp/ki/kd 速度环参数。
 * @param anglePid 角度环参数，输出单位为度每秒。
 * @param reverse 输出方向是否取反。
 * @note 位置模式可使用预置角度环；MCU 电流 PID 保持停用，电机内部闭合电流环。
 */
    Motor_Init_Config_s MakeMotorConfig(const Motor_Type_e motor_type, const uint32_t feedback_id,
                                        const float        kp, const float            ki, const float kd,
                                        const PID_Init_Config_s& anglePid, const bool reverse)
    {
        Motor_Init_Config_s config{};
        config.motor_type = motor_type;

        // 使用电机自身反馈，不使用外部反馈与前馈通道。
        config.controller_param_init_config.other_angle_feedback_ptr = nullptr;//其他角度反馈
        config.controller_param_init_config.other_speed_feedback_ptr = nullptr;//其他速度反馈
        config.controller_param_init_config.speed_feedforward_ptr    = nullptr;//速度前馈
        config.controller_param_init_config.current_feedforward_ptr  = nullptr;//电流前馈

        config.controller_param_init_config.speed_PID.kp           = kp;
        config.controller_param_init_config.speed_PID.ki           = ki;
        config.controller_param_init_config.speed_PID.kd           = kd;
        config.controller_param_init_config.speed_PID.output_min   = -kGimbalOutputLimit;
        config.controller_param_init_config.speed_PID.output_max   = kGimbalOutputLimit;
        config.controller_param_init_config.speed_PID.integral_min = -kGimbalIntegralLimit;
        config.controller_param_init_config.speed_PID.integral_max = kGimbalIntegralLimit;
        config.controller_param_init_config.angle_PID = anglePid;

        config.controller_setting_init_config.close_loop_type    = SPEED_AND_CURRENT_LOOP;
        config.controller_setting_init_config.outer_loop_type    = SPEED_AND_CURRENT_LOOP;
        config.controller_setting_init_config.motor_reverse_flag =
            reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
        config.controller_setting_init_config.feedback_reverse_flag =
            reverse ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL;
        config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
        config.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
        config.controller_setting_init_config.feedforward_flag      = FEEDFORWARD_NONE;

        config.can_init_config.can_handle = kGimbalCanHandle;
        config.can_init_config.rx_id      = feedback_id;
        return config;
    }

    /** @brief 在任务中读取一次完整反馈，避免中断更新期间读到混合值。 */
    Motor_Feedback_s FeedbackSnapshot(const DJI_Motor_s& motor)
    {
        taskENTER_CRITICAL();
        const Motor_Feedback_s feedback = motor.feedback;
        taskEXIT_CRITICAL();
        return feedback;
    }

    /**
 * @brief 更新单轴目标；急停或遥控失联后发送零输出，测试模式不检查电机反馈超时。
 * @note 位置模式重新使能时先将目标同步到当前角度，避免重新追逐旧目标。
 * todo read
 */
    void UpdateAxis(DJI_Motor_s* motor, MotorAngleTarget& target, bool outputAllowed,
                    bool positionMode, const MotorAngleLimits& limits,
                    float stick, float maxSpeedDps, float targetRateDps,
                    float dt_s)
    {
        if (motor == nullptr)
            return;

        const Motor_Feedback_s feedback = FeedbackSnapshot(*motor);
        if (!outputAllowed)
        {
            if (motor->enabled)
                DJI_Motor_Disable(motor);
            target.Reset();
            return;
        }

        if (positionMode)
        {
            // 位置模式需要首帧定位；速度测试模式不等待电机反馈。
            if (!feedback.angle_initialized)
                return;
            if (!limits.continuous)
            {
                const bool reverse = motor->motor_settings.feedback_reverse_flag ==
                                     FEEDBACK_DIRECTION_REVERSE;
                const auto raw = static_cast<uint16_t>(feedback.raw_angle);
                const bool allowPositive = limits.AllowsSpeed(raw, reverse, 1.0F);
                const bool allowNegative = limits.AllowsSpeed(raw, reverse, -1.0F);
                if (!allowPositive && !allowNegative)
                {
                    if (motor->enabled)
                        DJI_Motor_Disable(motor);
                    target.Reset();
                    return;
                }
                if (!allowPositive || !allowNegative)
                {
                    // 进入任一顶点内侧保护区后撤销位置保持，只接受手动向零位退回。
                    const float recoverySpeed = stick * targetRateDps;
                    if (!limits.AllowsSpeed(raw, reverse, recoverySpeed))
                    {
                        if (motor->enabled)
                            DJI_Motor_Disable(motor);
                        target.Reset();
                        return;
                    }
                    target.Reset();
                    if (motor->enabled && motor->motor_settings.close_loop_type != SPEED_AND_CURRENT_LOOP)
                        DJI_Motor_Disable(motor);
                    motor->motor_settings.close_loop_type = SPEED_AND_CURRENT_LOOP;
                    motor->motor_settings.speed = recoverySpeed;
                    if (!motor->enabled)
                        DJI_Motor_Enable(motor);
                    return;
                }
            }
            if (!target.Ready() || target.Epoch() != feedback.angle_epoch)
            {
                const bool reverse = motor->motor_settings.feedback_reverse_flag ==
                                     FEEDBACK_DIRECTION_REVERSE;
                if (!target.Sync(limits, feedback.total_angle,
                                 static_cast<uint16_t>(feedback.raw_angle),
                                 feedback.angle_epoch, reverse))
                {
                    if (motor->enabled)
                        DJI_Motor_Disable(motor);
                    return;
                }
                if (motor->enabled)
                    DJI_Motor_Disable(motor);
            }

            if (!target.Contains(limits, feedback.total_angle))
            {
                DJI_Motor_Disable(motor);
                target.Reset();
                return;
            }

            motor->motor_settings.close_loop_type = ANGLE_AND_SPEED_LOOP;
            motor->motor_settings.angle = target.Step(limits, stick, targetRateDps, dt_s);
            motor->motor_settings.angle_epoch = feedback.angle_epoch;
        }
        else
        {
            // 绕开角度环单调速度时仍保留有界轴的端点方向保护，不检查反馈年龄。
            const float speed = stick * maxSpeedDps;
            if (!limits.continuous)
            {
                const bool reverse = motor->motor_settings.feedback_reverse_flag ==
                                     FEEDBACK_DIRECTION_REVERSE;
                if (!feedback.angle_initialized ||
                    !limits.AllowsSpeed(static_cast<uint16_t>(feedback.raw_angle), reverse, speed))
                {
                    if (motor->enabled)
                        DJI_Motor_Disable(motor);
                    target.Reset();
                    return;
                }
            }
            target.Reset();
            motor->motor_settings.close_loop_type = SPEED_AND_CURRENT_LOOP;
            motor->motor_settings.speed = speed;
        }

        if (!motor->enabled)
            DJI_Motor_Enable(motor);
    }

    /** @brief 云台控制任务：读取遥控器、更新两轴目标，并发送统一 CAN 指令帧。 */
    void Task(void*)
    {
        uint32_t lastTick = osKernelGetTickCount();

        for (;;)
        {
            const uint32_t now  = osKernelGetTickCount();
            float          dt_s = static_cast<float>(now - lastTick) / 1000.0F;
            lastTick            = now;

            // 调度被长时间占用时不使用异常 dt, 避免积分项和微分项出现尖峰。
            if (dt_s <= 0.0F)
                dt_s = kGimbalControlPeriodS;
            const bool timingValid = dt_s <= kGimbalMaxDtS;
            if (!timingValid)
                dt_s = kGimbalMaxDtS;

            const auto remote = rc::Snapshot();
            const bool outputAllowed = !kEncoderCalibrationOnly && timingValid &&
                                       rc::IsOnline() && safety::ShouldOutput();
            UpdateAxis(yawMotor, yawTarget, outputAllowed && kEnableYaw, kYawPositionMode,
                       kYawAngleLimits, rc::Normalize(remote.channels[0]), kYawMaxSpeedDps,
                       kYawTargetRateDps, dt_s);
            UpdateAxis(pitchMotor, pitchTarget, outputAllowed && kEnablePitch, kPitchPositionMode,
                       kPitchAngleLimits, rc::Normalize(remote.channels[1]), kPitchMaxSpeedDps,
                       kPitchTargetRateDps, dt_s);

            DJI_Motor_Control(gimbalMotors, 2U, dt_s);

            const uint32_t next = now + 1U;
            if (static_cast<int32_t>(next - osKernelGetTickCount()) > 0)
                osDelayUntil(next);
            else
                osDelay(1U);
        }
    }
} // namespace

void gimbal::Update()
{
    // 控制逻辑集中在云台任务中执行; 保留此入口供后续由统一调度器调用。
}

serialTasks::EncoderSnapshot gimbal::GetEncoderSnapshot()
{
    serialTasks::EncoderSnapshot snapshot{};
    taskENTER_CRITICAL();
    if (pitchMotor != nullptr)
    {
        snapshot.pitch = {static_cast<uint16_t>(pitchMotor->feedback.raw_angle),
                          pitchMotor->feedback.angle_initialized,
                          pitchMotor->feedback.last_update_tick};
        snapshot.pitch_current_a = pitchMotor->feedback.current;
        if (pitchMotor->feedback.angle_initialized)
        {
            const bool reverse = pitchMotor->motor_settings.feedback_reverse_flag ==
                                 FEEDBACK_DIRECTION_REVERSE;
            const float position_deg = static_cast<float>(MotorEncoderDelta(
                kPitchZeroRaw, static_cast<uint16_t>(pitchMotor->feedback.raw_angle))) *
                (360.0F / 8192.0F) * (reverse ? -1.0F : 1.0F);
            snapshot.pitch_position_actual_deg = position_deg;
            if (pitchMotor->enabled &&
                pitchMotor->motor_settings.close_loop_type == ANGLE_AND_SPEED_LOOP &&
                pitchMotor->motor_settings.angle_epoch == pitchMotor->feedback.angle_epoch)
                snapshot.pitch_position_target_deg = pitchMotor->motor_settings.angle -
                                                     pitchMotor->feedback.total_angle + position_deg;
        }
    }
    if (yawMotor != nullptr)
    {
        snapshot.yaw = {static_cast<uint16_t>(yawMotor->feedback.raw_angle),
                        yawMotor->feedback.angle_initialized,
                        yawMotor->feedback.last_update_tick};
    }
    taskEXIT_CRITICAL();
    return snapshot;
}

void gimbal::Init()
{
    if (taskHandle != nullptr)
        return;

    // 反馈 ID 必须在 0x205..0x208 范围内, 对应 GM6020 的指令帧槽位。
    const Motor_Init_Config_s yawConfig = MakeMotorConfig(
                                                          GM6020, kYawMotorFeedbackId, kYawSpeedKp, kYawSpeedKi,
                                                          kYawSpeedKd, kYawAnglePid, kYawMotorReverse);
    const Motor_Init_Config_s pitchConfig = MakeMotorConfig(
                                                            GM6020, kPitchMotorFeedbackId, kPitchSpeedKp, kPitchSpeedKi,
                                                            kPitchSpeedKd, kPitchAnglePid, kPitchMotorReverse);

    if constexpr (kEnableYaw || kEncoderCalibrationOnly || kReadYawEncoder)
        yawMotor = DJI_Motor_Init(yawConfig);
    if constexpr (kEnablePitch || kEncoderCalibrationOnly)
        pitchMotor = DJI_Motor_Init(pitchConfig);
    if (((kEnableYaw || kEncoderCalibrationOnly || kReadYawEncoder) && yawMotor == nullptr) ||
        ((kEnablePitch || kEncoderCalibrationOnly) && pitchMotor == nullptr))
        Error_Handler(); // 只校验当前启用的轴。

    gimbalMotors[0] = yawMotor;
    gimbalMotors[1] = pitchMotor;

    taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
    if (taskHandle == nullptr)
        Error_Handler();
}
