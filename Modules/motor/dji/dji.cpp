/**
 * @file dji.cpp
 * @brief DJI M3508、GM6020 与 M2006 的反馈解析、闭环计算和指令打包。
 *
 * 职责划分:
 *   - 反馈路径: FDCAN 中断 -> CAN_ 模板按 FilterIndex 投递 ->
 *     MotorFeedbackCallback() 解析 8 字节反馈帧, 维护单圈角度、
 *     转速、连续角度 total_angle 与掉线时间戳。回调只做解析与标量
 *     赋值, 绝不阻塞。
 *   - 控制路径: 上层任务每个周期调 DJI_Motor_Control(), 按物理总线和指令帧分组
 *     计算各电机 16 位输出并打包进对应电流指令帧 (GM6020: 0x1FE/0x2FE，C610/C620: 0x200/0x1FF)。
 *     同帧最多 4 台电机, 输出槽位由反馈 ID 决定。
 *
 * 连续角度与重锚 (angle_epoch):
 *   相邻两次反馈间用 13 位编码器增量解卷绕累计 total_angle; 仅首帧
 *   初始化单圈基准并递增 angle_epoch。测试阶段移除反馈超时重锚和停机；
 *   中断期间若电机转过半圈以上，恢复后的累计角度可能不准确。
 *
 * 实例管理: 电机实例来自 16 槽静态池 (无动态分配), CAN 回调经
 * can_config.module_handle 直接回指电机实例, 无需额外映射表。
 */

//
// Created by 19923 on 2026/9/26.
//

#include "dji.h"

#include <cstring>

#include "cmsis_os2.h"
#include "main.h"
#include "motor/encoder_angle.h"
#include "dji_protocol.h"
#include "FreeRTOS.h"
#include "task.h"

namespace
{
    // ===== 修改开始：按 DJI 协议校验完整的 8 字节反馈帧 =====
    /** @brief GM6020、C620 和 C610 的反馈帧 DLC 均为 8 字节。 */
    constexpr uint8_t kFeedbackLength = 8U;
    // ===== 修改结束：反馈帧长度 =====

    /** @brief M3508 与 M2006 (C620/C610 电调) 的电流指令范围. */
    constexpr float kC620CurrentLimit = 16384.0F;
    constexpr float kC610CurrentLimit = 10000.0F;

    /** @brief 每个型号的指令帧 ID 起点、反馈 ID 起点和输出上限. */
    struct MotorTypeSpec
    {
        Motor_Type_e type;
        float        output_limit;      // 16 位输出的绝对值上限.
    };

    /**
 * @brief DJI 两类电调的指令帧与反馈 ID 分配.
 * @note GM6020 使用 0x1FE/0x2FE 电流指令与反馈 0x205 起，其余型号的电流协议不变。
 */
    constexpr MotorTypeSpec kMotorSpecs[] = {

        {
            .type = GM6020, .output_limit = kDjiGm6020CurrentCommandLimit
        },

        {
            .type = M3508, .output_limit = kC620CurrentLimit
        },

        {
            .type = M2006, .output_limit = kC610CurrentLimit
        },
    };

    /**
 * @brief 取出型号对应的帧与反馈分配.
 * @return 找到时返回对应表项; 型号非法时返回 nullptr.
 */
    const MotorTypeSpec* FindSpec(const Motor_Type_e motor_type)
    {
        for (const MotorTypeSpec& spec : kMotorSpecs)
        {
            if (spec.type == motor_type)
                return &spec;
        }
        return nullptr;
    }

    /**
 * @brief 把 16 位输出限制在型号允许范围内.
 * @note 超出范围按边界截断, 不调用 Error_Handler, 避免控制量异常导致整机停机.
 */
    int16_t ClampOutput(const float output, const float limit)
    {
        if (output > limit)
            return static_cast<int16_t>(limit);
        if (output < -limit)
            return static_cast<int16_t>(-limit);
        return static_cast<int16_t>(output);
    }

    /**
 * @brief 接收回调: 把反馈帧解析进引用该过滤器的电机实例.
 * @param attribute_instance BSP 注册表内的本实例属性.
     * @note 注册属性通过 module_handle 指向电机实例。
 * @note 本函数在 FDCAN 中断内执行, 只做解析和标量赋值, 不做任何阻塞操作.
 */
    void MotorFeedbackCallback(classicCanType* attribute_instance)
    {
        if (attribute_instance == nullptr || attribute_instance->module_handle == nullptr)
            return;
        auto* motor    = static_cast<DJI_Motor_s*>(attribute_instance->module_handle);
        // 首次注册启动 CAN 后可能立即进中断，此时返回句柄尚未写回 motor。
        if (motor->can_instance == nullptr)
            return;
        auto& instance = *motor->can_instance;

        const uint8_t* data = instance.output.rx_buffer.data();
        // ===== 修改开始：丢弃 DLC 不等于 8 的反馈帧 =====
        if (instance.output.rx_len != kFeedbackLength)
            return;
        // ===== 修改结束：接收长度检查 =====

        // 完整反馈帧为 8 字节；DATA[0..5] 为大端角度、转速及电流。
        const auto read_be16 = [](const uint8_t* bytes) -> int16_t
        {
            return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8U) | bytes[1]);
        };

        const int16_t raw_angle   = read_be16(data);
        const int16_t raw_speed   = read_be16(data + 2);
        const int16_t raw_current = read_be16(data + 4);

        const bool reverse = motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE;

        // 单圈机械角: 取 13 位计数值换算到 [0, 360) 度; GM6020 的完整 16 位原值保留在 raw 字段.
        const float angle_deg = static_cast<float>(static_cast<uint16_t>(raw_angle) & 0x1FFFU) / 8192.0F * 360.0F;
        // RPM 换算为度每秒.
        const float speed_dps = static_cast<float>(raw_speed) * 6.0F;
        // GM6020 按电流模式 ±16384 对应 ±3A；C610/C620 保留原有换算。
        const float current_a = static_cast<float>(raw_current) *
            (motor->motor_type == GM6020 ? 3.0F / kDjiGm6020CurrentCommandLimit : 20.0F / 32768.0F);

        const uint32_t    tick     = osKernelGetTickCount();
        Motor_Feedback_s& feedback = motor->feedback;
        //收到反馈,长度校验通过
        const bool        reanchor = !feedback.angle_initialized;
        if (reanchor)
        {
            // 首帧建立角度基准；反馈中断后不自动停机或重锚。
            feedback.total_angle       = reverse ? -angle_deg : angle_deg;
            feedback.angle_initialized = true;
            ++feedback.angle_epoch;
        } else
        {
            //正常:按最短增量累加
            const int16_t delta = MotorEncoderDelta(static_cast<uint16_t>(feedback.raw_angle),
                                                    static_cast<uint16_t>(raw_angle));
            const float delta_deg = static_cast<float>(delta) * (360.0F / 8192.0F);
            feedback.total_angle  += reverse ? -delta_deg : delta_deg;
        }
        feedback.raw_angle        = raw_angle;
        feedback.raw_speed        = raw_speed;
        feedback.raw_current      = raw_current;
        feedback.angle            = reverse ? -angle_deg : angle_deg;
        feedback.speed            = reverse ? -speed_dps : speed_dps;
        feedback.current          = reverse ? -current_a : current_a;
        // ===== 修改开始：C610 的 DATA[6..7] 为保留字节，不是温度 =====
        // 当前反馈结构以 0 表示“未提供温度”，不能把它当作实测 0°C。
        feedback.temperature      = motor->motor_type == M2006
                                        ? 0.0F : static_cast<float>(data[6]);
        // DATA[7] 不参与解析；C620/GM6020 的温度仍从 DATA[6] 读取。
        // ===== 修改结束：M2006 温度字段 =====
        feedback.last_update_tick = tick;

        instance.feedback_tick = tick;
    }

    /**
 * @brief 按闭环设置计算单个电机的 16 位输出.
 * @return 已限幅并应用方向标志的输出值.
 * @note 闭环组合非法时返回 0；测试模式使用最近一次反馈，不判定掉线。
 */
    float ComputeOutput(DJI_Motor_s& motor, const Motor_Feedback_s& feedback, const float dt_s)
    {
        const Motor_Control_Setting_s& setting    = motor.motor_settings;
        Motor_Controller_s&            controller = motor.controller;

        const float speed_feedback = setting.speed_feedback_source == OTHER_FEED
                                         ? *controller.other_speed_feedback_ptr
                                         : feedback.speed;
        const float angle_feedback = setting.angle_feedback_source == OTHER_FEED
                                         ? *controller.other_angle_feedback_ptr
                                         : feedback.total_angle;
        const float speed_feedforward =
            setting.feedforward_flag & SPEED_FEEDFORWARD ? *controller.speed_feedforward_ptr : 0.0F;
        const float current_feedforward =
            setting.feedforward_flag & CURRENT_FEEDFORWARD ? *controller.current_feedforward_ptr : 0.0F;

        float output = 0.0F;
        switch (setting.close_loop_type)
        {
            case OPEN_LOOP:
            case CURRENT_LOOP:
                // 开环与电流环都把期望值直接当作输出, 开环常用于标定和方向确认.
                output = setting.current;
                break;

            case SPEED_AND_CURRENT_LOOP:
                {
                    controller.pid_ref = setting.speed;
                    output             = controller.speed_PID.Update(setting.speed + speed_feedforward,
                                                                     speed_feedback, dt_s) +
                                         current_feedforward + setting.current;
                    break;
                }

            case ANGLE_AND_SPEED_LOOP:
            case ALL_THERE_LOOP:
                {
                    controller.pid_ref    = setting.angle;
                    const float speed_ref =
                        controller.angle_PID.Update(setting.angle, angle_feedback, dt_s);
                    output = controller.speed_PID.Update(speed_ref + speed_feedforward,
                                                         speed_feedback, dt_s) +
                             current_feedforward + setting.current;
                    break;
                }

            case STOP_LOOP:
            case ANGLE_LOOP:
            default:
                // 仅角度环无法直接映射到电流指令, 视为配置错误, 输出保持为零.
                output = 0.0F;
                break;
        }

        if (setting.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            output = -output;
        return output;
    }
} // namespace

DJI_Motor_s* DJI_Motor_Init(const Motor_Init_Config_s& config_instance)
{
    const MotorTypeSpec* spec = FindSpec(config_instance.motor_type);
    if (spec == nullptr || config_instance.can_init_config.can_handle == nullptr)
        return nullptr;

    DjiCommandAddress address{};
    if (!DjiResolveCommand(config_instance.motor_type == GM6020,
                           config_instance.can_init_config.rx_id, address) ||
        config_instance.can_init_config.can_handle->Init.FrameFormat != FDCAN_FRAME_CLASSIC)
        return nullptr;

    // 电机实例必须是静态存储, 因为 CAN 回调会长期持有其地址.
    static DJI_Motor_s motor_pool[16];
    static uint8_t     motor_pool_used = 0;
    if (motor_pool_used >= 16U)
        return nullptr;

    for (uint8_t i = 0; i < motor_pool_used; ++i)
        if (motor_pool[i].can_instance->dependency.hcan == config_instance.can_init_config.can_handle &&
            motor_pool[i].can_instance->attribute.rx_id == config_instance.can_init_config.rx_id)
            return nullptr;

    DJI_Motor_s& motor = motor_pool[motor_pool_used];

    motor.motor_type     = config_instance.motor_type;
    motor.motor_settings = config_instance.controller_setting_init_config;

    // 未使用的环允许全零参数；当前闭环所需的 PID 必须已成功配置。
    if (!Motor_Controller_InitPids(motor.controller, config_instance.controller_param_init_config))
        return nullptr;

    switch (motor.motor_settings.close_loop_type)
    {
        case OPEN_LOOP:
        case CURRENT_LOOP:
            break;
        case SPEED_AND_CURRENT_LOOP:
            if (!motor.controller.speed_PID.IsConfigured())
                return nullptr;
            break;
        case ANGLE_AND_SPEED_LOOP:
        case ALL_THERE_LOOP:
            if (!motor.controller.angle_PID.IsConfigured() ||
                !motor.controller.speed_PID.IsConfigured())
                return nullptr;
            break;
        default:
            return nullptr;
    }

    // 指针型前馈与外部反馈必须有效, 否则控制计算会解引用空指针.
    motor.controller.other_angle_feedback_ptr = config_instance.controller_param_init_config.other_angle_feedback_ptr;
    motor.controller.other_speed_feedback_ptr = config_instance.controller_param_init_config.other_speed_feedback_ptr;
    motor.controller.speed_feedforward_ptr    = config_instance.controller_param_init_config.speed_feedforward_ptr;
    motor.controller.current_feedforward_ptr  = config_instance.controller_param_init_config.current_feedforward_ptr;

    if (motor.motor_settings.speed_feedback_source == OTHER_FEED &&
        motor.controller.other_speed_feedback_ptr == nullptr)
        return nullptr;
    if (motor.motor_settings.angle_feedback_source == OTHER_FEED &&
        motor.controller.other_angle_feedback_ptr == nullptr)
        return nullptr;
    if ((motor.motor_settings.feedforward_flag & SPEED_FEEDFORWARD) &&
        motor.controller.speed_feedforward_ptr == nullptr)
        return nullptr;
    if ((motor.motor_settings.feedforward_flag & CURRENT_FEEDFORWARD) &&
        motor.controller.current_feedforward_ptr == nullptr)
        return nullptr;

    // 反馈 ID 决定 CAN 过滤器, 因此由上层在 can_init_config.rx_id 中给出.
    // tx_id 仅作为兜底初值, 实际发送时由 DJI_Motor_Control() 按指令帧覆盖.
    classicCanConfigType can_config = config_instance.can_init_config;
    can_config.tx_id                = address.frame_id;
    can_config.can_callback_func    = MotorFeedbackCallback;
    // module_handle 回指本电机, 使回调无需额外全局映射即可找到实例.
    can_config.module_handle        = &motor;

    motor.can_instance = CAN_<canConfig::classicCan>::Register(can_config);
    if (motor.can_instance == nullptr)
        return nullptr;

    ++motor_pool_used;
    return &motor;
}

void DJI_Motor_Enable(DJI_Motor_s* motor)
{
    if (motor == nullptr)
        return;
    motor->controller.pid_ref = 0.0F;
    motor->controller.current_PID.Reset();
    motor->controller.speed_PID.Reset();
    motor->controller.angle_PID.Reset();
    motor->enabled = true;
}

void DJI_Motor_Disable(DJI_Motor_s* motor)
{
    if (motor == nullptr)
        return;
    motor->motor_settings.current = 0.0F;
    motor->motor_settings.speed   = 0.0F;
    motor->motor_settings.angle   = 0.0F;
    motor->controller.current_PID.Reset();
    motor->controller.speed_PID.Reset();
    motor->controller.angle_PID.Reset();
    motor->enabled = false;
}

void DJI_Motor_Control(DJI_Motor_s** motor_list, const uint8_t count, const float dt_s)
{
    if (motor_list == nullptr || count == 0U || !(dt_s > 0.0F))
        return;
    struct CommandFrame
    {
        FDCAN_HandleTypeDef* bus{};
        CAN_<classicCan>::Instance* sender{};
        uint32_t id{};
        std::array<uint8_t, 8> payload{};
    };
    // 同一物理总线 + 同一帧 ID 只发送一次，M3508/M2006 可共用 0x200。
    std::array<CommandFrame, 16> frames{};
    uint8_t frameCount = 0U;
    for (uint8_t i = 0; i < count; ++i)
    {
        DJI_Motor_s* motor = motor_list[i];
        if (motor == nullptr || motor->can_instance == nullptr)
            continue;
        const MotorTypeSpec* spec = FindSpec(motor->motor_type);
        DjiCommandAddress address{};
        if (spec == nullptr || !DjiResolveCommand(motor->motor_type == GM6020,
                                                  motor->can_instance->attribute.rx_id, address))
            continue;
        FDCAN_HandleTypeDef* bus = motor->can_instance->dependency.hcan;
        uint8_t frameIndex = 0U;
        while (frameIndex < frameCount &&
               (frames[frameIndex].bus != bus || frames[frameIndex].id != address.frame_id))
            ++frameIndex;
        if (frameIndex == frameCount)
        {
            if (frameCount >= frames.size())
                return;
            frames[frameIndex].bus = bus;
            frames[frameIndex].sender = motor->can_instance;
            frames[frameIndex].id = address.frame_id;
            ++frameCount;
        }
        taskENTER_CRITICAL();
        const Motor_Feedback_s feedback = motor->feedback;
        taskEXIT_CRITICAL();
        float output = 0.0F;
        const bool positionLoop = motor->motor_settings.close_loop_type == ANGLE_AND_SPEED_LOOP ||
                                  motor->motor_settings.close_loop_type == ALL_THERE_LOOP;
        const bool targetFresh = !positionLoop || motor->motor_settings.angle_epoch == feedback.angle_epoch;
        // 测试模式不等待首帧，也不因反馈超时置零；无反馈时按零初始化值闭环。
        if (motor->enabled && targetFresh)
            output = ComputeOutput(*motor, feedback, dt_s);
        const uint16_t value = static_cast<uint16_t>(ClampOutput(output, spec->output_limit));
        frames[frameIndex].payload[address.slot * 2U] = static_cast<uint8_t>(value >> 8U);
        frames[frameIndex].payload[address.slot * 2U + 1U] = static_cast<uint8_t>(value);
    }
    for (uint8_t i = 0; i < frameCount; ++i)
    {
        frames[i].sender->attribute.tx_id = frames[i].id;
        classicCan::Input input{};
        input.tx_len = classicCan::DATA_SIZE;
        input.tx_buffer = frames[i].payload;
        // FIFO 满时本周期不阻塞，下个控制周期重新发送最新值。
        const auto result = frames[i].sender->Transmit(input);
        for (uint8_t j = 0; j < count; ++j)
        {
            auto* motor = motor_list[j];
            DjiCommandAddress address{};
            if (motor == nullptr || motor->can_instance == nullptr ||
                motor->can_instance->dependency.hcan != frames[i].bus ||
                !DjiResolveCommand(motor->motor_type == GM6020,
                                   motor->can_instance->attribute.rx_id, address) ||
                address.frame_id != frames[i].id)
                continue;
            ++motor->tx_enqueue_attempts;
            if (result != HAL_OK) ++motor->tx_enqueue_failures;
        }
    }
}
