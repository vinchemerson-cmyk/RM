/**
 * @file motor_common.h
 * @brief 电机公共标志及基础反馈类型；品牌控制器见各自目录。
 */

#ifndef INFANTRY_H723_MOTOR_COMMON_H
#define INFANTRY_H723_MOTOR_COMMON_H

#include <cstdint>


/** @brief 将可写变量限制在给定闭区间；宏会多次求值参数。 */
#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/** @brief 电机机械角与反馈转速的单位换算；角度按单圈 8192 计数折算。 */
#define MOTOR_ANGLE_TO_DEGREES(raw) ((static_cast<float>(static_cast<uint16_t>(raw) & 0x1FFFU)) / 8192.0F * 360.0F)
#define MOTOR_RPM_TO_DEGREES_PER_SECOND(raw) (static_cast<float>(raw) * 6.0F)

/** @brief 选择控制器参与的电流、速度和角度闭环。 */
enum Closeloop_Type_e
{
	OPEN_LOOP              = 0b0000,//开环
	CURRENT_LOOP           = 0b0001,//电流环
	SPEED_LOOP             = 0b0010,//速度环
	ANGLE_LOOP             = 0b0100,//角度环
	SPEED_AND_CURRENT_LOOP = 0b0011,//速度电流串环
	ANGLE_AND_SPEED_LOOP   = 0b0110,//角度速度串环
	ALL_THERE_LOOP         = 0b0111,//三环
};

/** @brief 组合控制器的前馈来源。 */
enum Feedfoward_Type_e
{
	FEEDFORWARD_NONE              = 0b000,                                   //不启用任何前馈
	CURRENT_FEEDFORWARD           = 0b001,                                   //启动电流前馈
	SPEED_FEEDFORWARD             = 0b010,                                   //启动速度前馈
	FORCE_FEEDFORWARD             = 0b100,                                   //启动力前馈
	CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD, //启动电流速度双环
};

/** @brief 电机输出方向。 */
enum Motor_Reverse_Flag_e
{
	MOTOR_DIRECTION_NORMAL  = 0,//正转
	MOTOR_DIRECTION_REVERSE = 1,//反转
};

/** @brief 反馈来自电机自身或外部传感器。 */
enum Feedback_Source_From_e
{
	MOTOR_FEED = 0,
	IMU_FEED   = 1,
	OTHER_FEED = 2,
};

/** @brief 电机停止或使能状态。 */
enum Motor_Working_Type_e
{
	MOTOR_STOP   = 0,//禁止输出
	MOTOR_ENABLE = 1,//允许输出
};


/** @brief 电机初始化和就绪状态。 */
enum Motor_state_e
{
	MOTOR_INIT = 0,
	MOTOR_READY,
};

/**
 * @brief 从 CAN 反馈帧解析出的电机实际状态。
 * @note angle 为单圈机械角；total_angle 为按相邻反馈解开的连续角度，单位均为度。
 * @note 这里保留DJI原始字段和单温度布局；达妙MIT使用独立反馈结构，含MOS/转子温度。
 */
struct Motor_Feedback_s
{
	int16_t raw_angle{};   //原始单圈机械角
	int16_t raw_speed{};   //原始速度
	int16_t raw_current{}; //原始电流

	float angle{};       //单圈机械角
	float total_angle{}; //多圈合计
	float speed{};       //速度
	float current{};     //电流
	float temperature{}; //温度

	//todo该角度版本号是否该保留
	uint32_t angle_epoch{};       // 首帧建立角度基准时递增；测试阶段反馈中断后不重锚。
	bool     angle_initialized{}; //是否角度初始化
	uint32_t last_update_tick{};  // 最近一次收到反馈的 RTOS tick, 用于判定掉线。
};

#endif // INFANTRY_H723_MOTOR_COMMON_H
