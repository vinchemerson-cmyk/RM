//
// Created by 19923 on 2026/10/16.
//

#ifndef INFANTRY_H723_DM_H
#define INFANTRY_H723_DM_H
#include <cstdint>

#include "can/can.h"
#include "can/canConfig.h"
#include "motor/motor_common.h"

namespace DM
{
	enum Motor_Type_e
	{
		MOTOR_TYPE_UNKNOWN = 0,
		DM4310,
	};

	struct Motor_Init_Config_s
	{
		Motor_Controller_Init_s controller_param_init_config{};
		Motor_Control_Setting_s controller_setting_init_config{};
		Motor_Type_e            motor_type{};
		FdcanConfigType    can_init_config{};
	};
}

struct DM_Motor_s
{
	Motor_Controller_s                     controller{};
	Motor_Control_Setting_s                motor_settings{};
	DM::Motor_Type_e                       motor_type{DM::MOTOR_TYPE_UNKNOWN};
	Motor_Feedback_s                       feedback{};
	CAN_<canConfig::Fdcan>::Instance* can_instance{};
	bool                                   enable{false};

	//尝试入队成功次数
	uint32_t							   tx_enqueue_attempts{};

	//尝试入队失败次数
	uint32_t							   tx_enqueue_failures{};
};

struct MIT_Init_Config_s
{
	MIT_Controller_Init_s controller_param_init_config{};
	Motor_Reverse_Flag_e motor_reverse_flag{};
	canConfig::Fdcan::Config can_init_config{};

};

DM_Motor_s* DM_Motor_Init(const DM::Motor_Init_Config_s& config);

void DM_Motor_Enable(DM_Motor_s* motor);

void DM_Motor_Disable(DM_Motor_s* motor);

void DM_Motor_Control(DM_Motor_s** motor_list,uint8_t count,float dt_s);

#endif //INFANTRY_H723_DM_H
