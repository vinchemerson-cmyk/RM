#pragma once
/** @file dji_controller.h @brief DJI专用控制设置、运行PID与初始化配置。 */
#include "algorithm/pid.h"
#include "motor/motor_common.h"

/**
 * @brief 通用控制环路、反馈方向、前馈选择以及上层写入的期望值。
 * @note angle 单位为度, speed 单位为度每秒, current 为直接叠加到输出上的电流指令。
 * @note 期望值由上层每个控制周期写入, 控制函数据此和 feedback 计算输出。
 */
struct DJ_Motor_Control_Setting_s
{
	Closeloop_Type_e     outer_loop_type{};    //外环
	Closeloop_Type_e     close_loop_type{};    //使用的(串级)闭环
	Motor_Reverse_Flag_e motor_reverse_flag{}; //电机反转标志

	Feedback_Source_From_e angle_feedback_source{}; //角度反馈
	Feedback_Source_From_e speed_feedback_source{}; //速度反馈
	Feedfoward_Type_e feedforward_flag{};      //前馈标志

	float    angle{};
	float    speed{};
	float    current{};
	uint32_t angle_epoch{}; // 位置目标对应的反馈版本；重锚后旧位置指令不得发送。
};

/** @brief 控制器运行时所引用的反馈、前馈和三个 PID 环。 */
struct DJ_Motor_Controller_s
{
	float*      other_angle_feedback_ptr{}; //其他来源的角度反馈数据指针
	float*      other_speed_feedback_ptr{};//其他来源的速度反馈数据指针

	float*      speed_feedforward_ptr{};//速度前馈
	float*      current_feedforward_ptr{};//电流前馈

	PIDInstance current_PID{};	//电流环
	PIDInstance speed_PID{};	//速度环
	PIDInstance angle_PID{};	//角度环

	float       pid_ref{};	//pid反馈
};

/** @brief 大疆电机控制器初始化参数。 */
struct DJ_Motor_Controller_Init_s
{
	float*            other_angle_feedback_ptr{};//其他角度反馈
	float*            other_speed_feedback_ptr{};//其他速度反馈
	float*            speed_feedforward_ptr{};//速度前馈
	float*            current_feedforward_ptr{};//电流前馈

	Feedback_Source_From_e angle_feedback_source{}; // 兼容既有配置字段；当前有效来源由motor_settings选择。
	Feedback_Source_From_e speed_feedback_source{};

	PID_Init_Config_s current_PID{};
	PID_Init_Config_s speed_PID{};
	PID_Init_Config_s angle_PID{};
};

bool DJ_Motor_Controller_InitPids(DJ_Motor_Controller_s& controller,
    const DJ_Motor_Controller_Init_s& config);
