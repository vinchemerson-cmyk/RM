/** @file dmmotor.cpp @brief MIT驱动，整理自原MSC1309实现；不含普通PID分支。 */
#include "dmmotor.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <limits>

namespace
{
DMMotorInstance motors[DM_MOTOR_CNT]{};
uint8_t count{}, cycle{};

HAL_StatusTypeDef Send(DMMotorInstance& motor, const classicCan::Input& input)
{
    if (motor.motor_can_instance == nullptr) return HAL_ERROR;
    ++motor.tx_enqueue_attempts;
    const auto status = motor.motor_can_instance->Transmit(input);
    if (status != HAL_OK) ++motor.tx_enqueue_failures;
    return status;
}
HAL_StatusTypeDef SendMode(DMMotorInstance& motor, DMMotor_Mode_e command)
{
    classicCan::Input input{};
    input.tx_len = 8U;
    input.tx_buffer.fill(0xFFU);
    input.tx_buffer[7] = static_cast<uint8_t>(command);
    return Send(motor, input); // 只填报文，不memset CAN实例。
}
bool SendControl(DMMotorInstance& motor, bool active)
{
    dmProtocol::Command command{};
    if (active)
    {
        const float sign = motor.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0F : 1.0F;
        command = {sign * motor.MIT_controller.ref_angle, sign * motor.MIT_controller.ref_speed,
            sign * motor.MIT_controller.ref_torque, motor.MIT_controller.Kp, motor.MIT_controller.Kd};
    }
    classicCan::Input input{};
    input.tx_len = 8U;
    const bool valid = dmProtocol::Pack(command, motor.protocol_limits, input.tx_buffer);
    if (!valid)
    {
        motor.stop_flag = MOTOR_STOP;
        if (!dmProtocol::Pack({}, motor.protocol_limits, input.tx_buffer)) return false;
    }
    return Send(motor, input) == HAL_OK && valid;
}
void DecodeFeedback(classicCanType* attribute)
{
    if (attribute == nullptr || attribute->module_handle == nullptr) return;
    auto& motor = *static_cast<DMMotorInstance*>(attribute->module_handle);
    if (motor.motor_can_instance == nullptr) return;
    const auto& rx = motor.motor_can_instance->output;
    dmProtocol::Feedback raw{};
    if (!dmProtocol::Unpack(rx.rx_buffer.data(), rx.rx_len, motor.protocol_limits, raw) ||
        raw.motor_id != attribute->master_id) return;

    const float sign = motor.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0F : 1.0F;
    const float position = sign * raw.position;
    auto& feedback = motor.measure;
    feedback.last_position = motor.rx_init_flag != 0 ? feedback.position : position;
    const float span = motor.protocol_limits.position_max - motor.protocol_limits.position_min;
    if (position - feedback.last_position > span * 0.5F && feedback.total_round > INT32_MIN)
        --feedback.total_round;
    else if (position - feedback.last_position < -span * 0.5F && feedback.total_round < INT32_MAX)
        ++feedback.total_round;
    feedback.position = position;
    feedback.total_position = position + static_cast<float>(feedback.total_round) * span;
    feedback.velocity = sign * raw.velocity;
    feedback.torque = sign * raw.torque;
    feedback.state = raw.state;
    feedback.T_Mos = raw.mos_temperature;
    feedback.T_Rotor = raw.rotor_temperature;
    feedback.last_update_tick = osKernelGetTickCount();
    motor.rx_init_flag = 1;
    ++motor.feedback_sequence;
    motor.rx_refresh_dt = 0;
    motor.motor_can_instance->feedback_tick = feedback.last_update_tick;
}
}

DMMotorInstance* MITMotorInit(const MIT_Init_Config_s* config)
{
    if (config == nullptr || count >= DM_MOTOR_CNT || config->can_init_config.can_handle == nullptr ||
        config->can_init_config.can_handle->Init.FrameFormat != FDCAN_FRAME_CLASSIC ||
        config->can_init_config.master_id == 0U || config->can_init_config.master_id > 0xFU ||
        config->can_init_config.can_id > 0x7FFU || !dmProtocol::Valid(config->protocol_limits) ||
        config->feedback_timeout_ms == 0U || config->feedback_timeout_ms >= 0x80000000U ||
        !std::isfinite(config->controller_param_init_config.Kp) ||
        !std::isfinite(config->controller_param_init_config.Kd) ||
        config->controller_param_init_config.Kp < 0.0F || config->controller_param_init_config.Kp > config->protocol_limits.kp_max ||
        config->controller_param_init_config.Kd < 0.0F || config->controller_param_init_config.Kd > config->protocol_limits.kd_max ||
        (config->motor_reverse_flag != MOTOR_DIRECTION_NORMAL && config->motor_reverse_flag != MOTOR_DIRECTION_REVERSE))
        return nullptr;
    for (uint8_t i = 0U; i < count; ++i)
        if (motors[i].motor_can_instance->dependency.hcan == config->can_init_config.can_handle &&
            (motors[i].motor_can_instance->attribute.can_id == config->can_init_config.can_id ||
             motors[i].motor_can_instance->attribute.master_id == config->can_init_config.master_id)) return nullptr;

    auto& motor = motors[count];
    motor = {};
    motor.motor_reverse_flag = config->motor_reverse_flag;
    motor.protocol_limits = config->protocol_limits;
    motor.feedback_timeout_ms = config->feedback_timeout_ms;
    motor.MIT_controller.Kp = config->controller_param_init_config.Kp;
    motor.MIT_controller.Kd = config->controller_param_init_config.Kd;
    auto can_config = config->can_init_config;
    can_config.can_callback_func = DecodeFeedback;
    can_config.module_handle = &motor;
    motor.motor_can_instance = DmCan::Register(can_config);
    if (motor.motor_can_instance == nullptr) return nullptr;
    ++count;
    return &motor;
}

void DMMotorEnable(DMMotorInstance* motor)
{
    if (motor == nullptr) return;
    taskENTER_CRITICAL();
    motor->enable_sequence = motor->feedback_sequence;
    taskEXIT_CRITICAL();
    motor->enable_tick = osKernelGetTickCount();
    motor->enable_pending = SendMode(*motor, DM_CMD_MOTOR_MODE) == HAL_OK;
    motor->stop_flag = motor->enable_pending ? MOTOR_ENABLE : MOTOR_STOP;
}
void DMMotorStop(DMMotorInstance* motor)
{
    if (motor == nullptr) return;
    motor->stop_flag = MOTOR_STOP;
    motor->enable_pending = false;
    motor->MIT_controller.ref_angle = motor->MIT_controller.ref_speed = motor->MIT_controller.ref_torque = 0.0F;
    (void)SendControl(*motor, false);
}
void DMMotorDisable(DMMotorInstance* motor)
{
    if (motor == nullptr) return;
    DMMotorStop(motor);
    (void)SendMode(*motor, DM_CMD_RESET_MODE);
}
void DMMotorCaliEncoder(DMMotorInstance* motor)
{
    if (motor == nullptr || motor->stop_flag != MOTOR_STOP) return;
    // 非阻塞；只是入队，不能当成电机已完成重设。
    if (SendMode(*motor, DM_CMD_ZERO_POSITION) == HAL_OK)
    {
        taskENTER_CRITICAL();
        motor->rx_init_flag = 0;
        motor->measure.total_round = 0;
        taskEXIT_CRITICAL();
    }
}
void DMMotorSetMITAngle(DMMotorInstance* motor, float value) { if (motor != nullptr) motor->MIT_controller.ref_angle = value; }
void DMMotorSetMITSpeed(DMMotorInstance* motor, float value) { if (motor != nullptr) motor->MIT_controller.ref_speed = value; }
void DMMotorSetMITTorque(DMMotorInstance* motor, float value) { if (motor != nullptr) motor->MIT_controller.ref_torque = value; }

void DMMotorTask()
{
    if (count == 0U) { cycle = 0U; return; }
    if (++cycle < DM_TASK_CYCLE) return;
    cycle = 0U;
    const uint32_t now = osKernelGetTickCount();
    for (uint8_t i = 0U; i < count; ++i)
    {
        auto& motor = motors[i];
        taskENTER_CRITICAL();
        const auto feedback = motor.measure;
        const bool seen = motor.rx_init_flag != 0;
        const uint32_t feedback_sequence = motor.feedback_sequence;
        if (motor.rx_refresh_dt < std::numeric_limits<int>::max()) ++motor.rx_refresh_dt;
        taskEXIT_CRITICAL();
        const bool fresh = seen && now - feedback.last_update_tick <= motor.feedback_timeout_ms;
        if (motor.enable_pending)
        {
            if (fresh && feedback_sequence != motor.enable_sequence && feedback.state == DM_STATE_ENABLE)
                motor.enable_pending = false;
            else if (now - motor.enable_tick > motor.feedback_timeout_ms)
            { motor.enable_pending = false; motor.stop_flag = MOTOR_STOP; }
        }
        if (!motor.enable_pending && seen && (!fresh || feedback.state != DM_STATE_ENABLE)) motor.stop_flag = MOTOR_STOP;
        const bool active = motor.stop_flag == MOTOR_ENABLE && !motor.enable_pending && fresh && feedback.state == DM_STATE_ENABLE;
        // 未见有效反馈时发送零转矩，故障不自动清错或恢复。
        if (!SendControl(motor, active)) motor.stop_flag = MOTOR_STOP;
    }
}
