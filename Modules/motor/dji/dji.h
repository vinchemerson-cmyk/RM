/**
 * @file dji.h
 * @brief DJI GM6020、M3508 和 M2006 的电机初始化配置。
 */

#ifndef INFANTRY_H723_DJI_H
#define INFANTRY_H723_DJI_H

#include "motor/motor_common.h"
#include "can/canConfig.h"

/** @brief 当前 DJI 电机驱动所支持的型号。 */
enum Motor_Type_e
{
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
};

/** @brief DJI 电机的控制器、型号和经典 CAN 注册配置。 */
struct Motor_Init_Config_s
{
    Motor_Controller_Init_s controller_param_init_config;
    Motor_Control_Setting_s controller_setting_init_config{};
    Motor_Type_e motor_type;
    classicCanConfigType can_init_config{};
};

#endif // INFANTRY_H723_DJI_H
