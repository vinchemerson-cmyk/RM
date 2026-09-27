/**
 * @file motor_common.h
 * @brief 各型号电机共用的控制状态、反馈选择和 PID 配置。
 */

#ifndef INFANTRY_H723_MOTOR_COMMON_H
#define INFANTRY_H723_MOTOR_COMMON_H

#include "algorithm/pid.h"

/** @brief 将可写变量限制在给定闭区间；宏会多次求值参数。 */
#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/** @brief 选择控制器参与的电流、速度和角度闭环。 */
enum Closeloop_Type_e
{
    OPEN_LOOP = 0b0000,
    CURRENT_LOOP = 0b0001,
    STOP_LOOP = 0b0010,
    ANGLE_LOOP = 0b0100,
    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP = 0b0110,
    ALL_THERE_LOOP = 0b0111,
};

/** @brief 组合控制器的前馈来源。 */
enum Feedfoward_Type_e
{
    FEEDFORWARD_NONE = 0b000,
    CURRENT_FEEDFORWARD = 0b001,
    SPEED_FEEDFORWARD = 0b010,
    FORCE_FEEDFORWARD = 0b100,
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
};

/** @brief 电机输出方向。 */
enum Motor_Reverse_Flag_e
{
    MOTOR_DIRECTION_NORMAL = 0,
    MOTOR_DIRECTION_REVERSE = 1,
};

/** @brief 反馈来自电机自身或外部传感器。 */
enum Feedback_Source_e
{
    MOTOR_FEED = 0,
    OTHER_FEED = 1,
};

/** @brief 反馈方向是否反转。 */
enum Feedback_Reverse_Flag_e
{
    FEEDBACK_DIRECTION_NORMAL = 0,
    FEEDBACK_DIRECTION_REVERSE = 1,
};

/** @brief 电机停止或使能状态。 */
enum Motor_Wokking_Type_e
{
    MOTOR_STOP = 0,
    MOTOR_ENABLE = 1,
};

/** @brief 通用控制环路、反馈方向和前馈选择。 */
struct Motor_Control_Setting_s
{
    Closeloop_Type_e        outer_loop_type;
    Closeloop_Type_e        close_loop_type;
    Motor_Reverse_Flag_e    motor_reverse_flag;
    Feedback_Reverse_Flag_e feedback_reverse_flag;
    Feedback_Source_e       angle_feedback_source;
    Feedback_Source_e       speed_feedback_source;
    Feedfoward_Type_e       feedforward_flag;
};

/** @brief 控制器运行时所引用的反馈、前馈和三个 PID 环。 */
struct Motor_Controller_s
{
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float pid_ref;
};

/** @brief 普通电机控制器初始化参数。 */
struct Motor_Controller_Init_s
{
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    PID_Init_Config_s current_PID;
    PID_Init_Config_s speed_PID;
    PID_Init_Config_s angle_PID;
};

/**
 * @brief 按初始化参数配置普通电机的三个 PID 环。
 * @return 所有环参数合法时返回 true；失败时三个环均停用。
 */
bool Motor_Controller_InitPids(Motor_Controller_s& controller,
                               const Motor_Controller_Init_s& config);

/** @brief 电机初始化和就绪状态。 */
enum Motor_state_e
{
    MOTOR_INIT = 0,
    MOTOR_READY,
};

#endif // INFANTRY_H723_MOTOR_COMMON_H
