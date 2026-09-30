/**
 * @file other_motors.h
 * @brief 达妙、MIT 模式、LK9025 和 HT04 的电机配置 (预留接口, 尚未接入驱动)。
 *
 * 本文件为非 DJI 电机定义初始化参数结构:
 *   - 达妙 (DM) 普通模式: 速度环+角度环双 PID, 控制方式配置待补;
 *   - MIT 模式: 下发 Kp/Kd 刚度阻尼, 由电调内部做位置阻抗控制;
 *   - LK9025 / HT04: 复用 motor_common.h 的三环 PID 结构。
 * 这些型号与 DJI 电调的指令帧格式不同, 接入时需要新建对应驱动
 * (类似 dji.cpp) 并按各自协议打包指令帧, 不复用 DJI_Motor_Control()。
 */

#ifndef INFANTRY_H723_OTHER_MOTORS_H
#define INFANTRY_H723_OTHER_MOTORS_H

#include "motor_common.h"
#include "can/canConfig.h"

/** @brief 需要单独配置的非 DJI 电机型号。 */
enum Other_Motor_Type_e
{
    LK9025 = 4,
    HT04 = 5,
};

/** @brief 达妙电机控制器初始化参数。 */
struct DMMotor_Controller_Init_s
{
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *force_feedforward_ptr;
    PID_Init_Config_s speed_PID;
    PID_Init_Config_s angle_PID;
};

/** @brief 达妙电机运行时速度环与角度环的独立 PID 状态。 */
struct DMMotor_Controller_s
{
    PIDInstance speed_PID;
    PIDInstance angle_PID;
};

/**
 * @brief 按初始化参数配置达妙电机的速度环和角度环。
 * @return 所有环参数合法时返回 true；失败时两个环均停用。
 */
bool DMMotor_Controller_InitPids(DMMotor_Controller_s& controller,
                                 const DMMotor_Controller_Init_s& config);

/** @brief 达妙普通模式的控制器和经典 CAN 注册配置。 */
struct DMMotor_Init_Config_s
{
    DMMotor_Controller_Init_s controller_param_init_config;
    // TODO: 达妙普通模式的控制方式配置尚未定义。
    classicCanConfigType can_init_config{};
};

/** @brief MIT 控制模式下的刚度和阻尼系数。 */
struct MIT_Controller_Init_s
{
    float Kp;
    float Kd;
};

/** @brief MIT 模式的控制器、方向及经典 CAN 配置。 */
struct MIT_Init_Config_s
{
    MIT_Controller_Init_s controller_param_init_config;
    Motor_Reverse_Flag_e motor_reverse_flag;
    classicCanConfigType can_init_config{};
};

/** @brief LK9025 或 HT04 的通用控制器和经典 CAN 注册配置。 */
struct OtherMotor_Init_Config_s
{
    Motor_Controller_Init_s controller_param_init_config;
    Motor_Control_Setting_s controller_setting_init_config{};
    Other_Motor_Type_e motor_type;
    classicCanConfigType can_init_config{};
};

#endif // INFANTRY_H723_OTHER_MOTORS_H
