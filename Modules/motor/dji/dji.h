/**
 * @file dji.h
 * @brief DJI GM6020、M3508 和 M2006 的电机初始化配置。
 *
 * 支持型号与总线资源 (与 dji.cpp 中的 kMotorSpecs 表保持一致):
 *   型号    指令帧                    反馈 ID         16 位输出范围
 *   GM6020  0x1FE (ID1-4) / 0x2FE (ID5-7)  0x205..0x20B  ±16384 (转矩电流 ±3A)
 *   M3508   0x200 (ID1-4) / 0x1FF (ID5-8)  0x201..0x208  ±16384 (电流)
 *   M2006   0x200 (ID1-4) / 0x1FF (ID5-8)  0x201..0x208  ±10000 (电流)
 *
 * 反馈帧格式 (DLC 为 8 字节, 数值字段为大端):
 *   [0:1] 机械角 13 位有效, 0..8191 对应 0..360°;
 *   [2:3] 转速 (RPM, 转子轴);
 *   [4:5] 电流/转矩指令码 (C620: 1 LSB = 20A/32768);
 *   [6]   GM6020/C620 的温度 (°C); C610 的保留字节;
 *   [7]   保留字节。
 * 驱动内部由相邻反馈解卷绕出连续角度 total_angle, 供角度环使用。
 * @note GM6020 电机参数必须开启电流环；软件速度环输出电流指令码，电机内部闭合电流环。
 *
 * 典型用法:
 *   1. Init 阶段: 用 Motor_Init_Config_s 描述型号/PID/闭环组合/CAN 反馈 ID,
 *      调 DJI_Motor_Init() 得到实例指针 (静态池, 无动态分配);
 *   2. 控制周期: 上层写 motor_settings.speed 或 .angle, 再调
 *      DJI_Motor_Control() 统一计算输出并按物理总线、指令帧分组打包发送;
 *   3. 安全: 上层通过 DJI_Motor_Enable()/DJI_Motor_Disable() 控制输出,
 *      未使能的电机按零输出打包；测试阶段移除电机反馈超时停机。
 */

#ifndef INFANTRY_H723_DJI_H
#define INFANTRY_H723_DJI_H

#include "motor/motor_common.h"
#include "can/can.h"
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
	Motor_Controller_Init_s controller_param_init_config{};
	Motor_Control_Setting_s controller_setting_init_config{};
	Motor_Type_e            motor_type{};
	classicCanConfigType    can_init_config{};
};

/**
 * @brief 单个 DJI 电机的完整运行实例。
 * @note 本结构由 DJI_Motor_Init() 填充, 上层只读取 feedback 并写入 motor_settings。
 * @note 反馈数据的权威副本保存在 can_instance->output 中, 由 CAN 中断直接填充。
 */
struct DJI_Motor_s
{
	Motor_Controller_s                     controller{};
	Motor_Control_Setting_s                motor_settings{};
	Motor_Type_e                           motor_type{MOTOR_TYPE_NONE};
	Motor_Feedback_s                       feedback{};
	CAN_<canConfig::classicCan>::Instance* can_instance{};
	bool                                   enabled{};
	/** @brief CAN 入队诊断，按本电机所在合并帧计数；HAL_OK 不代表总线发送成功。 */
	uint32_t                               tx_enqueue_attempts{};
	uint32_t                               tx_enqueue_failures{};
};

/**
 * @brief 初始化一个 DJI 电机并注册其 CAN 反馈过滤器。
 * @param config_instance 电机型号、PID 参数、控制设置和 CAN 注册配置。
 * @return 指向静态电机实例的指针; motor_type 非法时返回 nullptr。
 * @note 调用方填写 Classic CAN 的 can_handle 和合法 rx_id，回调由驱动填写。
 * @note 同一总线反馈 ID 不得重复；非法型号、ID、总线模式或重复 ID 返回 nullptr。
 */
DJI_Motor_s* DJI_Motor_Init(const Motor_Init_Config_s& config_instance);

/**
 * @brief 启用一个已初始化的 DJI 电机。
 * @note 仅把状态标记为就绪; 实际输出由 DJI_Motor_Control() 在每个控制周期发送。
 */
void DJI_Motor_Enable(DJI_Motor_s* motor);

/**
 * @brief 松开一个 DJI 电机, 停止输出并清空 PID 历史。
 * @note 下一次 DJI_Motor_Control() 会继续发送零输出帧；重新 Enable 时保留 PID 配置。
 */
void DJI_Motor_Disable(DJI_Motor_s* motor);

/**
 * @brief 按各电机的闭环设置计算输出并打包成一帧发到 CAN 总线。
 * @param motor_list 电机实例数组, 数组内 nullptr 元素会被跳过。
 * @param count 数组长度，静态电机池最多 16 个。
 * @param dt_s 本次控制周期, 单位秒。
 * @note 同总线同指令帧的所有电机必须在同一次调用中提交，避免不同任务互相覆盖槽位。
 * @note 位置指令必须写入对应反馈的 motor_settings.angle_epoch；版本不匹配时发送零输出。
 * @note 未使能的电机按零输出处理。测试阶段不等待首帧且不检查反馈超时，使用最近一次反馈。
 */
void DJI_Motor_Control(DJI_Motor_s** motor_list, uint8_t count, float dt_s);

#endif // INFANTRY_H723_DJI_H
