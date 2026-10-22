/** @file dmmotor.h @brief 达妙MIT接口；不提供MCU普通PID模式。 */
#pragma once
#include "can/can.h"
#include "motor/motor_common.h"
#include "mit_protocol.h"

// MIT通过FDCAN外设的经典CAN模式收发，与BSP接收分发表保持一致。
using DmCan = CAN_<canConfig::classicCan>;
using DmCanInstance = DmCan::Instance;
inline constexpr uint8_t DM_MOTOR_CNT = 6U;
inline constexpr uint8_t DM_TASK_CYCLE = 2U; // 上层1ms调用，2次分频。

enum DMMotor_Err_state_e : uint8_t
{
    DM_STATE_DISABLE = 0x0, DM_STATE_ENABLE = 0x1,
    DM_STATE_OVER_VOLTAGE = 0x8, DM_STATE_UNDER_VOLTAGE = 0x9,
    DM_STATE_OVER_CURRENT = 0xA, DM_STATE_MOS_OVER_TEMP = 0xB,
    DM_STATE_MOTOR_COIL_OVER_TEMP = 0xC, DM_STATE_COMM_LOST = 0xD, DM_STATE_OVERLOAD = 0xE
};
enum DMMotor_Mode_e : uint8_t
{
    DM_CMD_MOTOR_MODE = 0xFC, DM_CMD_RESET_MODE = 0xFD,
    DM_CMD_ZERO_POSITION = 0xFE, DM_CMD_CLEAR_ERROR = 0xFB
};
struct MIT_Controller_Init_s { float Kp{}, Kd{}; };
struct MIT_Controller_s
{
    float ref_angle{}, ref_speed{}, ref_torque{}; // 电机协议坐标：rad、rad/s、N·m。
    float Kp{}, Kd{}; // 驱动器MIT增益，不是MCU PID。
};
struct MIT_Init_Config_s
{
    MIT_Controller_Init_s controller_param_init_config{};
    Motor_Reverse_Flag_e motor_reverse_flag{MOTOR_DIRECTION_NORMAL};
    classicCanConfigType can_init_config{}; // master_id发送指令ID，can_id反馈CAN ID。
    dmProtocol::Limits protocol_limits{};   // 必须与对应电机的调试助手量程一致。
    uint32_t feedback_timeout_ms{100U};
};
struct DM_Motor_Measure_s
{
    uint8_t state{};
    float velocity{}, last_position{}, position{}, total_position{}, torque{};
    float T_Mos{}, T_Rotor{};
    int32_t total_round{};
    uint32_t last_update_tick{};
};
struct DMMotorInstance
{
    DM_Motor_Measure_s measure{};
    MIT_Controller_s MIT_controller{};
    Motor_Reverse_Flag_e motor_reverse_flag{MOTOR_DIRECTION_NORMAL};
    Motor_Working_Type_e stop_flag{MOTOR_STOP};
    DmCanInstance* motor_can_instance{};
    dmProtocol::Limits protocol_limits{};
    uint32_t feedback_timeout_ms{100U};
    int rx_refresh_dt{}, rx_init_flag{};
    uint32_t tx_enqueue_attempts{}, tx_enqueue_failures{};
    uint32_t feedback_sequence{}, enable_sequence{}, enable_tick{};
    bool enable_pending{};
};

/** 调度前注册一次，返回静态对象；不自动使能。 */
DMMotorInstance* MITMotorInit(const MIT_Init_Config_s* config);
/** 显式发送0xFC；入队失败保持停止，非零输出还要求有效使能反馈。 */
void DMMotorEnable(DMMotorInstance* motor);
/** 软件停止并尝试发零Kp/Kd/转矩，不等同于硬件失能。 */
void DMMotorStop(DMMotorInstance* motor);
/** 软件停止后发送0xFD，不自动清错或恢复。 */
void DMMotorDisable(DMMotorInstance* motor);
/** 仅软件停止时允许；不等待完成，上层须确认反馈再启用。 */
void DMMotorCaliEncoder(DMMotorInstance* motor);
void DMMotorSetMITAngle(DMMotorInstance* motor, float angle_rad);
void DMMotorSetMITSpeed(DMMotorInstance* motor, float speed_rad_s);
void DMMotorSetMITTorque(DMMotorInstance* motor, float torque_nm);
/** 上层1ms周期调用；反馈超时/故障锁存停止，须显式Enable；不创建任务。 */
void DMMotorTask();
