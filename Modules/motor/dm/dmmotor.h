#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <cstdint>
#include "algorithm/pid.h"
#include "can/can.h"
#include "motor/motor_common.h"

struct MIT_Init_Config_s;

namespace DM
{
	struct Motor_Init_Config_s;
}

using DmCan = CAN_<Fdcan>;
using DmCanInstance = DmCan::Instance;
//以下宏定义在第一次使用DM框架时请务必根据实际情况更改

#define DM_MOTOR_CNT 6 //达妙电机注册数量最大值
#define DM_TASK_CYCLE 2 //达妙任务运行周期，决定控制频率，PID计算频率以及电机反馈频率，必须为整数，单位为ms，推荐值为2

//以下为位置，速度，力矩的映射范围，电机第一次使用时一定要配合实际情况以及DM调试助手修改
#define DM_P_MIN (-3.14159265358979f)
#define DM_P_MAX 3.14159265358979f // 位置范围推荐设置为3.14159265358979f，可以方便多圈计数和就近转位
#define DM_V_MIN (-45.0f)
#define DM_V_MAX 45.0f
#define DM_T_MIN (-54.0f)
#define DM_T_MAX 54.0f

//这两个映射范围是固定的，请不要修改
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

typedef enum
{
	DM_STATE_DISABLE              = 0x0, //达妙电机未使能
	DM_STATE_ENABLE               = 0x1, //达妙电机使能
	DM_STATE_OVER_VOLTAGE         = 0x8, //达妙电机驱动过压故障
	DM_STATE_UNDER_VOLTAGE        = 0x9, //达妙电机驱动欠压
	DM_STATE_OVER_CURRENT         = 0xA, //达妙电机过流故障
	DM_STATE_MOS_OVER_TEMP        = 0xB, //驱动MOS管过热故障状态
	DM_STATE_MOTOR_COIL_OVER_TEMP = 0xC, //电机线圈过热故障
	DM_STATE_COMM_LOST            = 0xD, //通讯丢失
	DM_STATE_OVERLOAD             = 0xE, //过载故障
} DMMotor_Err_state_e;                   //电机反馈的状态位枚举

typedef struct
{
	uint8_t state;          //状态
	float   velocity;       //速率
	float   last_position;  //上次位置
	float   position;       //位置
	float   total_position; //总体位置
	float   torque;         //扭矩
	float   T_Mos;          //驱动板MOS管温度
	float   T_Rotor;        //电机温度
	int     total_round;    //总转动圈数
} DM_Motor_Measure_s;       //存储电机反馈值的结构体

typedef struct
{
	uint16_t position_des; //目标位置
	uint16_t velocity_des; //目标速率
	uint16_t torque_des;   //目标扭矩
	uint16_t Kp;           //kp
	uint16_t Kd;           //kd
} DMMotor_Send_s;

typedef enum
{
    NORMAL_MODE = 0,
    MIT_MODE
} Motor_Control_Type_e;//DM电机封装的两个模式

typedef struct
{
	float ref_angle;//角度参考值
	float ref_speed;//速度参考值
	float ref_torque;//力矩参考值
	float Kp;
	float Kd;
} MIT_Controller_s;


typedef struct
{
    DM_Motor_Measure_s measure{};

    //普通模式配置的PID相关设置
    Motor_Control_Setting_s motor_settings{};
    PIDInstance speed_PID{};
    PIDInstance angle_PID{};

    //普通模式配置的反馈与前馈指针
    float *other_angle_feedback_ptr{};
    float *other_speed_feedback_ptr{};
    float *speed_feedforward_ptr{};
    float *force_feedforward_ptr{};
    float pid_ref{};

    //MIT模式配置的参数
	MIT_Controller_s MIT_controller{};

    int rx_refresh_dt{};//用于通过实时观测变量来监测总线通信质量不参与通信超时的判断（看门狗依赖DM内置的timeout）
    //如果观察该变量大于等于2，则说明通信质量低，请检查布线是否规范，总线负载是否过高
    int rx_init_flag{};//第一次收到反馈标志位，用于防止多圈角度计数错误

    Motor_Working_Type_e stop_flag{};//软件关闭电机标志位
    DmCanInstance *motor_can_instance{};//达妙电机对应的FDCAN通信实例
    Motor_Control_Type_e working_type{};//工作模式
} DMMotorInstance;//DM电机实例

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,    // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,    // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb    // 清除电机过热错误
} DMMotor_Mode_e;// DM电机特殊控制帧对应的枚举

/**
 * @brief DM普通模式注册函数
 * @param config 电机初始化结构体，包含了电机控制设置，电机PID参数设置，电机类型以及电机挂载的CAN设置
 * @return 返回一个DM电机实例
 */
inline DMMotorInstance *DMMotorInit(DM::Motor_Init_Config_s *config);

/**
 * @brief DM普通模式下设置最外环参考值，内环参考值请通过前馈指针设置
 * @param motor 对应的电机实例
 * @param ref 设置的参考值
 */
void DMMotorSetRef(DMMotorInstance *motor, float ref);

/**
 * @brief DM普通模式下设置最外环的闭环类型
 * @param motor 对应的电机实例
 * @param closeloop_type 闭环类型
 */
void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e closeloop_type);

/**
 * @brief DM普通模式下切换反馈类型
 * @param motor 对应的电机实例
 * @param loop 需要切换反馈类型的闭环
 * @param type 反馈类型
 */
void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

/**
 * @brief MIT模式注册函数
 * @param config 电机初始化结构体，包含了MIT参数设置，电机反转标志位设置，电机挂载的CAN设置
 * @return 返回一个DM电机实例
 */
inline DMMotorInstance *MITMotorInit(MIT_Init_Config_s *config);

/**
 * @brief MIT模式下设置位置参考值
 * @param motor 对应的电机实例
 * @param angle 设置的参考值
 */
void DMMotorSetMITAngle(DMMotorInstance *motor, float angle);

/**
 * @brief MIT模式下设置速度参考值
 * @param motor 对应的电机实例
 * @param speed 设置的参考值
 */
void DMMotorSetMITSpeed(DMMotorInstance *motor, float speed);

/**
 * @brief MIT模式下设置力矩参考值
 * @param motor 对应的电机实例
 * @param torque 设置的参考值
 */
void DMMotorSetMITTorque(DMMotorInstance *motor, float torque);

/**
 * @brief 软件开启电机，电机初始化后默认就是开启状态
 * @param motor 对应的电机实例
 */
void DMMotorEnable(DMMotorInstance *motor);

/**
 * @brief 软件关闭电机，使电机输出为0并清空PID参数，请调用该接口来对电机进行急停而不是直接发送失能帧
 * @param motor 对应的电机实例
 */
void DMMotorStop(DMMotorInstance *motor);

/**
 * @brief 将电机当前位置设置为电机的零点，不推荐调用
 * @param motor 对应的电机实例
 */
void DMMotorCaliEncoder(DMMotorInstance *motor);

/**
 * @brief DM电机任务函数，应该将其放置在MotorControlTask();中以一定频率运行
 */
void DMMotorTask();

#endif // DMMOTOR_H
