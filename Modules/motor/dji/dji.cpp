/**
 * @file dji.cpp
 * @brief DJI M3508、GM6020 与 M2006 的反馈解析、闭环计算和指令打包。
 *
 * 职责划分:
 *   - 反馈路径: FDCAN 中断 -> CAN_ 模板按 FilterIndex 投递 ->
 *     MotorFeedbackCallback() 解析 8 字节反馈帧, 维护单圈角度、
 *     转速、连续角度 total_angle 与掉线时间戳。回调只做解析与标量
 *     赋值, 绝不阻塞。
 *   - 控制路径: 上层任务每个周期调 DJI_Motor_Control(), 按型号分组
 *     计算各电机 16 位输出并打包进对应指令帧 (0x1FF/0x2FF/0x200)。
 *     同帧最多 4 台电机, 输出槽位由反馈 ID 决定。
 *
 * 连续角度与重锚 (angle_epoch):
 *   相邻两次反馈间用 13 位编码器增量解卷绕累计 total_angle; 首帧或
 *   超过 kOfflineTimeoutMs 无反馈时"重锚": total_angle 直接取当前
 *   单圈角度, angle_epoch 递增, 且正在用角度环的电机被自动 Disable,
 *   逼迫上层重新同步位置目标 (见 angle_target.h / gimbal)。
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

namespace
{
    /** @brief 一条标准帧最多容纳 4 个电机的 16 位输出. */
    constexpr uint8_t kMotorsPerFrame = 4;

    /** @brief 单个电机的输出占 2 字节. */
    constexpr uint8_t kBytesPerMotor = 2;

    // ===== 修改开始：按 DJI 协议校验完整的 8 字节反馈帧 =====
    /** @brief GM6020、C620 和 C610 的反馈帧 DLC 均为 8 字节。 */
    constexpr uint8_t kFeedbackLength = 8U;
    // ===== 修改结束：反馈帧长度 =====

    /** @brief 连续多久收不到反馈即判定掉线, 单位毫秒. */
    constexpr uint32_t kOfflineTimeoutMs = 100;

    /** @brief M3508 与 M2006 (C620/C610 电调) 的电流指令范围. */
    constexpr float kC620CurrentLimit = 16384.0F;

    /** @brief GM6020 内置电调的电压指令范围. */
    constexpr float kGM6020VoltageLimit = 25000.0F;

    /** @brief 每个型号的指令帧 ID 起点、反馈 ID 起点和输出上限. */
    struct MotorTypeSpec
    {
        Motor_Type_e type;
        uint32_t     command_id_first;  // 该型号第一条指令帧的 ID.
        uint32_t     feedback_id_first; // 该型号第一个反馈 ID.
        uint8_t      frame_count;       // 该型号使用的指令帧条数.
        float        output_limit;      // 16 位输出的绝对值上限.
        const char*  name;              // 日志用的型号名.
    };

    /**
 * @brief DJI 两类电调的指令帧与反馈 ID 分配.
 * @note GM6020 使用 0x1FF/0x2FF 与反馈 0x205 起, 其余型号使用 0x200 与反馈 0x201 起.
 */
    constexpr MotorTypeSpec kMotorSpecs[] = {

        {
            .type        = GM6020, .command_id_first = 0x1FFU, .feedback_id_first = 0x205U,
            .frame_count = 2U, .output_limit         = kGM6020VoltageLimit, .name = "GM6020"
        },

        {
            .type        = M3508, .command_id_first = 0x200U, .feedback_id_first = 0x201U,
            .frame_count = 1U, .output_limit        = kC620CurrentLimit, .name   = "M3508"
        },

        {
            .type        = M2006, .command_id_first = 0x200U, .feedback_id_first = 0x201U,
            .frame_count = 1U, .output_limit        = kC620CurrentLimit, .name   = "M2006"
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
 * @brief 判断反馈是否仍在有效期内.
 * @note 超时判定放在控制周期内做, 使掉线检测不依赖中断.
 */
    bool FeedbackAlive(const CAN_<canConfig::classicCan>::Instance& instance, const uint32_t now)
    {
        // ===== 修改开始：仅完整反馈帧可用于闭环 =====
        if (instance.output.rx_len != kFeedbackLength)
            return false;
        // ===== 修改结束：反馈有效性检查 =====
        return now - instance.feedback_tick <= kOfflineTimeoutMs;
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
        // C620 反馈电流 1 LSB 约为 20A/32768; GM6020 反馈电流同样按 ±20A 满量程解析.
        const float current_a = static_cast<float>(raw_current) * (20.0F / 32768.0F);

        const uint32_t    tick     = osKernelGetTickCount();
        Motor_Feedback_s& feedback = motor->feedback;
        //收到反馈,长度校验通过
        const bool        reanchor = !feedback.angle_initialized ||
                                     tick - feedback.last_update_tick > kOfflineTimeoutMs;
        if (reanchor)
        {
            // 断线后的首帧可能已跨过半圈；位置环必须等上层重新同步目标。
            if (feedback.angle_initialized &&
                (motor->motor_settings.close_loop_type == ANGLE_AND_SPEED_LOOP ||
                 motor->motor_settings.close_loop_type == ALL_THERE_LOOP))
                motor->enabled = false;
            //重锚:直接以当前角度为基准
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
 * @note 掉线或闭环组合非法时返回 0, 保证同帧其他电机不受影响.
 */
    float ComputeOutput(DJI_Motor_s& motor, const float dt_s)
    {
        const Motor_Control_Setting_s& setting    = motor.motor_settings;
        Motor_Controller_s&            controller = motor.controller;

        const float speed_feedback = setting.speed_feedback_source == OTHER_FEED
                                         ? *controller.other_speed_feedback_ptr
                                         : motor.feedback.speed;
        const float angle_feedback = setting.angle_feedback_source == OTHER_FEED
                                         ? *controller.other_angle_feedback_ptr
                                         : motor.feedback.total_angle;
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

    // 电机实例必须是静态存储, 因为 CAN 回调会长期持有其地址.
    static DJI_Motor_s motor_pool[16];
    static uint8_t     motor_pool_used = 0;
    if (motor_pool_used >= 16U)
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
    can_config.tx_id                = spec->command_id_first;
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
    if (motor_list == nullptr || count == 0 || !(dt_s > 0.0F))
        return;

    const uint32_t now = osKernelGetTickCount();

    // 按指令帧分组: 同一帧 ID 的电机必须打包进同一帧, 否则后发的会覆盖先发的.
    for (const MotorTypeSpec& spec : kMotorSpecs)
    {
        for (uint8_t frame_index = 0; frame_index < spec.frame_count; ++frame_index)
        {
            const uint32_t frame_id = spec.command_id_first + frame_index;

            std::array<uint8_t, classicCan::DATA_SIZE> payload{};
            uint8_t                                    packed = 0;

            // 第一遍: 计算各电机输出并填入本帧对应的 2 字节槽位.
            for (uint8_t i = 0; i < count && packed < kMotorsPerFrame; ++i)
            {
                DJI_Motor_s* motor = motor_list[i];
                // 电机指针无效 / 类型不匹配 / CAN实例为空，则跳过
                if (motor == nullptr || motor->motor_type != spec.type || motor->can_instance == nullptr)
                    continue;

                // 反馈 ID 决定该电机在指令帧中的 2 字节槽位.
                const uint32_t offset = motor->can_instance->attribute.rx_id - spec.feedback_id_first;
                if (offset >= kMotorsPerFrame || offset / kMotorsPerFrame != frame_index)
                    continue;

                float output = 0.0F;
                //电机未使能 / 反馈未存活
                if (motor->enabled && FeedbackAlive(*motor->can_instance, now))
                    output = ComputeOutput(*motor, dt_s); //todo 阅读该函数,ComputeOutput()

                const int16_t value = ClampOutput(output, spec.output_limit);
                const auto    slot  = static_cast<uint8_t>(offset % kMotorsPerFrame);

                //这两行把 16 位电机控制值 value 按大端序拆成高、低两个字节，
                //写入 CAN 缓冲区第 slot 个电机的两个字节位置（offset 和 offset+1）
                payload[slot * kBytesPerMotor]     = static_cast<uint8_t>(value >> 8U);
                payload[slot * kBytesPerMotor + 1] = static_cast<uint8_t>(value & 0xFFU);
                ++packed;
            }

            if (packed == 0)
                continue;

            // 第二遍: 用本帧内第一个可用实例把整帧发出, 成功后不再重复发送.
            for (uint8_t i = 0; i < count; ++i)
            {
                DJI_Motor_s* motor = motor_list[i];
                if (motor == nullptr || motor->motor_type != spec.type || motor->can_instance == nullptr)
                    continue;

                const uint32_t offset = motor->can_instance->attribute.rx_id - spec.feedback_id_first;
                if (offset < spec.frame_count * kMotorsPerFrame || offset / kMotorsPerFrame != frame_index)
                    continue;

                motor->can_instance->attribute.tx_id = frame_id;
                classicCan::Input input{};
                input.tx_len = classicCan::DATA_SIZE;
                std::memcpy(input.tx_buffer.data(), payload.data(), classicCan::DATA_SIZE);
                if (motor->can_instance->Transmit(input) == HAL_OK)
                    break;
            }
        }
    }
}
