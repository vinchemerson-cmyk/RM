/**
 * @file dmmotor.cpp
 * @author MSC1309(msc1309@hnu.edu.cn)
 * @brief 达妙电机驱动，模仿dji_motor.c，对老代码进行优化而产出的统一，稳定的版本
 * @version 0.1
 * @date 2026-07-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "dmmotor.h"
#include <cstring>
#include "bsp_log.h"
#include "dm.h"
#include "memory.h"
#include "dwt/bsp_dwt.h"

static uint8_t idx_dm = 0;//用于记录注册的电机数量
static DMMotorInstance dm_motor_instance[DM_MOTOR_CNT];//提前定义好空白电机实例
static uint8_t dm_cycle_cnt = 0;//用于分频计数，控制DM任务周期

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

//用于给电机发送设置性质的指令，包含使能，失能，重设零点，清除错误
static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instance, 0xff, 7);  //发送电机指令的时候前面7bytes都是0xff//bug

	Fdcan::Input input{};
	input.tx_len = 8U;
	input.tx_buffer.fill(0xFF);
	input.tx_buffer[7] = static_cast<uint8_t>(cmd);

	motor->motor_can_instance->Transmit(input); // 最后一位是命令id
}

//DM电机接收回调函数，用于处理接收到的数据
static void DMMotorDecode(FdcanType* attribute)
{
	if (attribute == nullptr || attribute->module_handle == nullptr)
		return;

	auto* motor =
	static_cast<DMMotorInstance*>(attribute->module_handle);

	if (motor->motor_can_instance == nullptr)
		return;

	const auto& rx = motor->motor_can_instance->output;
	if (rx.rx_len != 8U)
		return;

	const uint8_t* rxbuff  = rx.rx_buffer.data();
	auto*          measure = &motor->measure;
	// 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量

	motor->rx_refresh_dt = 0; //一旦接收到反馈重置计数

	uint8_t ERR    = (rxbuff[0] >> 4) & 0x0F;
	measure->state = ERR; //解析状态位

	measure->last_position = measure->position; //保存上一次的角度，用于多圈计算

	auto tmp          = static_cast<uint16_t>((rxbuff[1] << 8) | rxbuff[2]);
	measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16); //解析角度

	if (motor->rx_init_flag == 0)
	{
		measure->last_position = measure->position; //防止因为第一次接受到数据时measure->last_position默认为0导致计数错误
		motor->rx_init_flag    = 1;
	}

	tmp               = static_cast<uint16_t>((rxbuff[3] << 4) | rxbuff[4] >> 4);
	measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12); //解析速度

	tmp             = static_cast<uint16_t>(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
	measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12); //解析力矩

	measure->T_Mos   = static_cast<float>(rxbuff[6]);
	measure->T_Rotor = static_cast<float>(rxbuff[7]); //解析电机温度

	// 多圈角度计数算法，前提是两次采样周期间隔内电机转动的角度不超过DM_P_MAX
	if (motor->measure.position - motor->measure.last_position > DM_P_MAX)
		motor->measure.total_round -= 1;
	else if (motor->measure.position - motor->measure.last_position < -DM_P_MAX)
		motor->measure.total_round += 1;
	motor->measure.total_position = motor->measure.position + motor->measure.total_round * 2 * DM_P_MAX;
}

//用于自动恢复通信（清除timeout错误），自动使能意外失能的电机
static void DMAutoRecoverComm(DMMotorInstance *motor)
{
    if (motor->measure.state == DM_STATE_DISABLE)
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    if (motor->measure.state == DM_STATE_COMM_LOST)
        DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor);
}

//普通模式初始化函数
DMMotorInstance *DMMotorInit(DM::Motor_Init_Config_s *config)
{
    if (idx_dm >= DM_MOTOR_CNT){//注册电机数量超过最大值，报错
        LOGERROR("[dmmotor] Too many motors!");
        return nullptr;
    }

    if(config->can_init_config.tx_id > 0x0F){//电机的txid大于0xF，会使电机状态位解析值不可预测，报错
        LOGERROR("[dmmotor] txID is greater than 0x0F.");
        return nullptr;
    }

    DMMotorInstance *motor = &dm_motor_instance[idx_dm];
    idx_dm++;

    //将注册信息填入实例中
    motor->motor_settings = config->controller_setting_init_config;
	motor->speed_PID.Reset();
	motor->angle_PID.Reset();
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    motor->force_feedforward_ptr = config->controller_param_init_config.force_feedforward_ptr;

    //为电机实例注册CAN总线
    config->can_init_config.can_callback_func = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instance = Register(&config->can_init_config);

    //模式为普通模式
    motor->working_type = NORMAL_MODE;

    //开启并使能电机
    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    
    return motor;
}

//MIT模式初始化函数
DMMotorInstance *MITMotorInit(MIT_Init_Config_s *config)
{
    if (idx_dm >= DM_MOTOR_CNT) {//注册电机数量超过最大值，报错
        LOGERROR("[dmmotor] Too many motors!");
        return nullptr;
    }

    if(config->can_init_config.tx_id > 0x0F){//电机的txid大于0xF，会使电机状态位解析值不可预测，报错
        LOGERROR("[dmmotor] txID is greater than 0x0F.");
        return nullptr;
    }

    DMMotorInstance *motor = &dm_motor_instance[idx_dm];
    idx_dm++;

    //将注册信息填入实例中
    motor->motor_settings.motor_reverse_flag = config->motor_reverse_flag;

    motor->MIT_controller.Kp = config->controller_param_init_config.Kp;
    motor->MIT_controller.Kd = config->controller_param_init_config.Kd;

    //为电机实例注册CAN总线
	config->can_init_config.can_callback_func = DMMotorDecode;
	config->can_init_config.module_handle     = motor;

	motor->motor_can_instance =
		CAN_<Fdcan>::Register(config->can_init_config);

    //模式为MIT模式
    motor->working_type = MIT_MODE;

    //开启并使能电机
    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    
    return motor;
}

int16_t ClampOutput(const float output, const float limit)
{
	if (output > limit)
		return static_cast<int16_t>(limit);
	if (output < -limit)
		return static_cast<int16_t>(-limit);
	return static_cast<int16_t>(output);
}

/// @brief 两种模式下均可使用的通用函数
//开启电机（注意不是使能）
void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENABLE;
}

//关闭电机，使电机的输出为0，清空PID（注意不是失能）
void DMMotorStop(DMMotorInstance *motor) 
{
    motor->stop_flag = MOTOR_STOP;
}

//用于重设零点
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.01);//重设零点会使电机短时间内无法使用，需要延时一段时间
}

/// @brief 普通模式下设置各种参数
//切换电机最外层闭环
void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

//切换电机反馈模式
void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
}

//设置最外环参考值，内环参考值请通过前馈指针设置
void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

/// @brief MIT模式下设置各种参数
void DMMotorSetMITAngle(DMMotorInstance *motor, float angle)
{
    motor->MIT_controller.ref_angle = angle;
}

void DMMotorSetMITSpeed(DMMotorInstance *motor, float speed)
{
    motor->MIT_controller.ref_speed = speed;
}

void DMMotorSetMITTorque(DMMotorInstance *motor, float torque)
{
    motor->MIT_controller.ref_torque = torque;
}

//DM任务函数，以一定频率执行（取决于DM_TASK_CYCLE），用于PID计算，发送控制帧，以及维护通信
void DMMotorTask()
{
	//用于控制函数执行周期
	if (idx_dm == 0)
		return;

	if (++dm_cycle_cnt < DM_TASK_CYCLE)
		return;

	dm_cycle_cnt = 0;

	// 执行电机控制

	DMMotor_Send_s       motor_send_mailbox;

    for(int i = 0;i < idx_dm; i++){//遍历所有电机实例，分别执行任务

        DMMotorInstance*         dmmotor       = &dm_motor_instance[i];
        Motor_Control_Setting_s* motor_setting = &dmmotor->motor_settings;
        DM_Motor_Measure_s*      measure       = &dmmotor->measure;

		dmmotor->rx_refresh_dt++;//每遍历到这个电机一次，计数就自增一次

        DMAutoRecoverComm(dmmotor);//自动恢复通信，将通信丢失和意外失能的电机重新拉回到使能状态

        if (dmmotor->working_type == MIT_MODE)//MIT模式需要发送的控制帧
        {
			DMMotor_Send_s dmmotor_send{};
			//将参数从浮点数转换为uint形式
            motor_send_mailbox.position_des = ClampOutput(dmmotor_send.position_des, DM_P_MAX);
            motor_send_mailbox.velocity_des = ClampOutput(dmmotor_send.velocity_des, DM_V_MAX);
            motor_send_mailbox.torque_des   = ClampOutput(dmmotor_send.torque_des, DM_T_MAX);
            motor_send_mailbox.Kp           = ClampOutput(dmmotor_send.Kp, KP_MAX);
            motor_send_mailbox.Kd           = ClampOutput(dmmotor_send.Kp, KD_MAX);

            //有反向标志位的情况
            if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE){
				MIT_Controller_s MIT_Controller{};
				motor_send_mailbox.position_des = ClampOutput(- MIT_Controller.ref_angle, DM_P_MAX);
                motor_send_mailbox.velocity_des = ClampOutput(- MIT_Controller.ref_speed, DM_V_MAX);
                motor_send_mailbox.torque_des   = ClampOutput(- MIT_Controller.ref_torque, DM_T_MAX);
            }

            //如果电机被关闭，则清空其PID以及力矩输出
            if (dmmotor->stop_flag == MOTOR_STOP)
            {
                motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
                motor_send_mailbox.Kp         = float_to_uint(0, KP_MIN, KP_MAX, 12);
                motor_send_mailbox.Kd         = float_to_uint(0, KD_MIN, KD_MAX, 12);
            }

        	//将参数按通信协议中的格式填入到tx_buff中
        	Fdcan::Input input{};
        	input.tx_len = 8U;
        	auto& buffer = input.tx_buffer;

        	buffer[0] = static_cast<uint8_t>(motor_send_mailbox.position_des >> 8);
        	buffer[1] = static_cast<uint8_t>(motor_send_mailbox.position_des);
        	buffer[2] = static_cast<uint8_t>(motor_send_mailbox.velocity_des >> 4);
        	buffer[3] = static_cast<uint8_t>(
				((motor_send_mailbox.velocity_des & 0xFU) << 4) |
				(motor_send_mailbox.Kp >> 8));
        	buffer[4] = static_cast<uint8_t>(motor_send_mailbox.Kp);
        	buffer[5] = static_cast<uint8_t>(motor_send_mailbox.Kd >> 4);
        	buffer[6] = static_cast<uint8_t>(
				((motor_send_mailbox.Kd & 0xFU) << 4) |
				(motor_send_mailbox.torque_des >> 8));
        	buffer[7] = static_cast<uint8_t>(motor_send_mailbox.torque_des);

        	const auto status = dmmotor->motor_can_instance->Transmit(input);

            //发送控制帧
        	dmmotor->motor_can_instance->Transmit(input);
        }
        else if (dmmotor->working_type == NORMAL_MODE)//普通模式需要发送的控制帧
        {
            float pid_ref, pid_measure;

            pid_ref = dmmotor->pid_ref;

            if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)//如果设置反向，直接将参考值反转符号
                pid_ref *= -1;

            if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)//如果启用了角度环且最外层闭环为角度环，则计算角度环
            {
                if (motor_setting->angle_feedback_source == OTHER_FEED)//判断是否使用电机反馈
                    pid_measure = *dmmotor->other_angle_feedback_ptr;
                else
                    pid_measure = measure->total_position;

                pid_ref = PIDCalculate(&dmmotor->angle_PID, pid_measure, pid_ref); //对总角度进行闭环，而不是单圈角度
            }

            if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))//如果启用了速度环且最外层闭环为角度环或速度环，则计算速度环
            {
                if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                    pid_ref += *dmmotor->speed_feedforward_ptr; //速度前馈

                if (motor_setting->speed_feedback_source == OTHER_FEED)//判断是否使用电机反馈
                    pid_measure = *dmmotor->other_speed_feedback_ptr;
                else
                    pid_measure = measure->velocity;

                pid_ref = PIDCalculate(&dmmotor->speed_PID, pid_measure, pid_ref);//对速度环闭环
            }


            if (motor_setting->feedforward_flag & FORCE_FEEDFORWARD)
                pid_ref += *dmmotor->force_feedforward_ptr;//力矩前馈

            float set = pid_ref;//得到最终的控制值

            LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);//进行限幅防止溢出

            //只对力矩（控制量）进行设置，其他均为0
            motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox.torque_des   = float_to_uint(set, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox.Kp           = float_to_uint(0, KP_MIN, KP_MAX, 12);
            motor_send_mailbox.Kd           = float_to_uint(0, KD_MIN, KD_MAX, 12);

            //如果电机被关闭，则清空其PID以及力矩输出
            if (dmmotor->stop_flag == MOTOR_STOP){
                motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
                PIDClear(&dmmotor->angle_PID);
                PIDClear(&dmmotor->speed_PID);
            }

            //将参数按通信协议中的格式填入到tx_buff中
            dmmotor->motor_can_instance->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
            dmmotor->motor_can_instance->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
            dmmotor->motor_can_instance->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
            dmmotor->motor_can_instance->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
            dmmotor->motor_can_instance->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
            dmmotor->motor_can_instance->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
            dmmotor->motor_can_instance->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
            dmmotor->motor_can_instance->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

            //发送控制帧
            dmmotor->motor_can_instance->Transmit(tx_buff[]);
        }
    }
}
