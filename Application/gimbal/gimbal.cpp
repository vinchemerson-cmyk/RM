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
 *      0x1FF 电压指令发出。
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
 * @note 位置模式可使用预置角度环；电流环以全零参数保持停用。
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
        config.controller_param_init_config.speed_PID.integral_min = -kGimbalOutputLimit;
        config.controller_param_init_config.speed_PID.integral_max = kGimbalOutputLimit;
        config.controller_param_init_config.angle_PID = anglePid;

        config.controller_setting_init_config.close_loop_type    = SPEED_AND_CURRENT_LOOP;
        config.controller_setting_init_config.outer_loop_type    = SPEED_AND_CURRENT_LOOP;
        config.controller_setting_init_config.motor_reverse_flag =
            reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
        config.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
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
 * @brief 更新单轴目标；急停、遥控失联或电机反馈超时后发送零输出。
 * @note 位置模式重新使能时先将目标同步到当前角度，避免重新追逐旧目标。
 * todo read
 */
    void UpdateAxis(DJI_Motor_s* motor, MotorAngleTarget& target, bool outputAllowed,
                    bool positionMode, const MotorAngleLimits& limits,
                    float stick, float maxSpeedDps, float targetRateDps,
                    float dt_s, uint32_t now)
    {
        if (motor == nullptr)
            return;

        const Motor_Feedback_s feedback = FeedbackSnapshot(*motor);
        const bool fresh = feedback.angle_initialized &&
                           now - feedback.last_update_tick <= kGimbalFeedbackTimeoutMs;
        if (!outputAllowed || !fresh)
        {
            if (motor->enabled)
                DJI_Motor_Disable(motor);
            target.Reset();
            return;
        }

        if (positionMode)
        {
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
        }
        else
        {
            motor->motor_settings.close_loop_type = SPEED_AND_CURRENT_LOOP;
            motor->motor_settings.speed = stick * maxSpeedDps;
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
            if (dt_s > kGimbalMaxDtS)
                dt_s = kGimbalMaxDtS;

            const bool outputAllowed = rc::IsOnline() && safety::ShouldOutput();
            UpdateAxis(yawMotor, yawTarget, outputAllowed, kYawPositionMode,
                       kYawAngleLimits, rc::RightHorizontal(), kYawMaxSpeedDps,
                       kYawTargetRateDps, dt_s, now);
            UpdateAxis(pitchMotor, pitchTarget, outputAllowed, kPitchPositionMode,
                       kPitchAngleLimits, rc::RightVertical(), kPitchMaxSpeedDps,
                       kPitchTargetRateDps, dt_s, now);

            DJI_Motor_Control(gimbalMotors, 2U, dt_s);

            osDelay(1);
        }
    }
} // namespace

void gimbal::Update()
{
    // 控制逻辑集中在云台任务中执行; 保留此入口供后续由统一调度器调用。
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

    yawMotor   = DJI_Motor_Init(yawConfig);
    pitchMotor = DJI_Motor_Init(pitchConfig);
    //note:yawpitch耦合
    if (yawMotor == nullptr || pitchMotor == nullptr)
        Error_Handler(); // 配置非法属于开发期错误, 不应带着半个云台继续运行。

    gimbalMotors[0] = yawMotor;
    gimbalMotors[1] = pitchMotor;

    taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
    if (taskHandle == nullptr)
        Error_Handler();
}
