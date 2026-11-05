/** @file dmmotor.h @brief 达妙MIT接口；不提供MCU普通PID模式。 */
#pragma once
#include "mit_protocol.h"
#include "can/can.h"
#include "motor/motor_common.h"
#include "motor/dji/dji.h"

// 真正CAN FD帧，MIT自身仍为8字节；可与同后端的板间长帧共享控制器。
using DmCan                            = CAN_<canConfig::Fdcan>;
using DmCanInstance                    = DmCan::Instance;
inline constexpr uint8_t DM_MOTOR_CNT  = 6U;
inline constexpr uint8_t DM_TASK_CYCLE = 2U; // 上层1ms调用，2次分频。

enum DMMotor_Type_e
{
	NONE,
	dm4310,
	dm4340
};

enum DMMotor_Err_state_e : uint8_t
{
	DM_STATE_DISABLE              = 0x0, //失能
	DM_STATE_ENABLE               = 0x1, //使能
	DM_STATE_OVER_VOLTAGE         = 0x8, //超电压
	DM_STATE_UNDER_VOLTAGE        = 0x9, //欠压
	DM_STATE_OVER_CURRENT         = 0xA, //超电流
	DM_STATE_MOS_OVER_TEMP        = 0xB, //MOS过温
	DM_STATE_MOTOR_COIL_OVER_TEMP = 0xC, //线圈过温
	DM_STATE_COMM_LOST            = 0xD, //通讯丢失
	DM_STATE_OVERLOAD             = 0xE  //过载
};

enum DMMotor_Mode_e : uint8_t
{
	DM_CMD_MOTOR_MODE    = 0xFC, //使能
	DM_CMD_RESET_MODE    = 0xFD, //失能
	DM_CMD_ZERO_POSITION = 0xFE, //保存位置零点
	DM_CMD_CLEAR_ERROR   = 0xFB  //清除错误
};

struct MIT_Controller_Init_s
{
	float Kp{}, Kd{};
};

//MIT控制器参数
struct MIT_Controller_s
{
	float ref_angle{};
	float ref_speed{};
	float ref_torque{};
	float Kp{};
	float Kd{};
};


struct MIT_Init_Config_s
{
	// 应用按本板接线传入句柄，驱动不写死总线；同路板间通信也走FD后端。
	MIT_Controller_Init_s  controller_param_init_config{}; //mit控制器参数初始化
	Motor_Reverse_Flag_e   motor_reverse_flag{MOTOR_DIRECTION_NORMAL};
	FdcanConfigType        can_init_config{}; // master_id发送指令ID，can_id反馈CAN ID。
	dmProtocol::Limits     protocol_limits{}; // 必须与对应电机的调试助手量程一致。
	uint32_t               feedback_timeout_ms{100U};
	DMMotor_Type_e         Motor_Type{NONE};
	Feedfoward_Type_e      Feedforward_Type{static_cast<Feedfoward_Type_e>(MOTOR_DIRECTION_NORMAL)};
	Feedback_Source_From_e Feedback_Source_From{MOTOR_FEED};
};

/**
 * @brief 取某型号电机的 MIT 协议量程。
 * @return 对应量程；型号为 NONE 时返回不合法值，MITMotorInit() 会据此拒绝注册。
 * @note 4310 与 4340 的出厂预设相同 (Pos ±12.5 / Vel ±30 / Torque ±10, KP 0~500, KD 0~5)，
 *       依据达妙说明书与官方 dm4310 例程 dm4310_drv.h。
 * @note 量程是"驱动器内设定值"，不是电机铭牌参数：若在达妙调试助手里改过 Pos/Vel/Torque
 *       范围，必须在此按型号覆盖，否则下发命令会等比例失真（力矩可偏小数倍且无报错）。
 */
inline dmProtocol::Limits LimitsFor(DMMotor_Type_e motor_type)
{
	if (motor_type == dm4310 || motor_type == dm4340)
		return dmProtocol::Limits{}; // 出厂预设，见 mit_protocol.h 中 Limits 的注释。
	// 非法型号：返回 Valid() 必然拒绝的量程，避免用默认值静默发帧。
	return dmProtocol::Limits{-1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F};
}

//DM电机反馈数据
struct DM_Motor_Measure_s
{
	uint8_t  state{};            //状态
	float    velocity{};         //速度
	float    last_position{};    //上次位置
	float    position{};         //位置
	float    total_position{};   //多圈位置
	float    torque{};           //转矩
	float    T_Mos{};            //MOS过温
	float    T_Rotor{};          //电机内部线圈的平均温度
	int32_t  total_round{};      //总圈数
	uint32_t last_update_tick{}; //上次更新滴答
};

struct DMMotorInstance
{
	DM_Motor_Measure_s   measure{};                                  //反馈数据
	MIT_Controller_s     MIT_controller{};                           //MIT控制参数
	Motor_Reverse_Flag_e motor_reverse_flag{MOTOR_DIRECTION_NORMAL}; //电机转动反馈
	Motor_Working_Type_e stop_flag{MOTOR_STOP};                      //停止标志
	DmCanInstance*       motor_can_instance{};                       //电机的FDCAN实例
	dmProtocol::Limits   protocol_limits{};                          //DM电机参数限制
	uint32_t             feedback_timeout_ms{100U};                  //反馈超时
	int                  cycles_since_feedback{};                    //自从上次收到反馈后已经过去的DMMotorTask()轮数
	int                  rx_init_flag{};                             //初始接收标志位
	uint32_t             tx_enqueue_attempts{};                      //发送队列尝试
	uint32_t             tx_enqueue_failures{};                      //如果该值过高请检查接线
	uint32_t             feedback_sequence{};                        //反馈序列
	uint32_t             enable_sequence{};                          //使能序列
	uint32_t             enable_tick{};                              //使能的时刻
	bool                 enable_pending{};                           //发送但未反馈
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
