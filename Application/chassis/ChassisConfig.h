//
// Created by 19923 on 2026/10/24.
//

#ifndef INFANTRY_H723_CHASSISCONFIG_H
#define INFANTRY_H723_CHASSISCONFIG_H
#include "chassis_control.h"
#include "fdcan.h"
#include "stm32h7xx_hal_fdcan.h"
#include "algorithm/pid.h"
#include "motor/dji/dji.h"
#include "rc/rc.h"

// 底盘电机所在的物理CAN控制器，不是某个电调的ID；总线中若存在同指令分组的其他电机，
// 必须统一发送，不能让多个任务分别打包同一帧并互相覆盖其他电机的槽位。
inline FDCAN_HandleTypeDef* const kChassisCanHandle = &hfdcan1;

// 原设计为实物参数确认开关；当前chassis.cpp把校验入口注释了，因此此值暂不参与运行拦截。
// 它与电机enabled不同：硬件确认是配置条件，enabled是某个控制周期的软件输出状态。
inline constexpr bool kChassisHardwareConfirmed = false;
/** @brief 单轮电机及传动配置；与下方几何参数数组使用相同轮序。 */
struct ChassisWheelHardware
{
    DJI::Motor_Type_e type{DJI::MOTOR_TYPE_NONE}; // 必须填写实际型号，默认NONE不是一台可用电机。
    uint32_t feedback_id{}; // MCU接收电机反馈的标准帧ID；M3508/M2006通常为0x201..0x208，0表示未填。
    bool reverse{};        // true统一反转该电机的反馈和输出，使逻辑正轮速符合drive_direction_deg。
    float gear_ratio{};    // 转子速度/输出轴速度；例如19:1填写19，不能把轮径或拨盘槽数填在这里。
};
// FL、FR、BL、BR；同总线同指令帧必须由本任务独占发送
// 四个条目都必须填实物数据；当前空条目会值初始化为NONE/0/false/0，不是自动分配电调ID。
inline constexpr std::array<ChassisWheelHardware, chassis::kWheelCount>
kChassisWheels{
	//todo:
	{

		{
			//FL
		},
		{
			//FR
		},
		{
			//BL
		},
		{
			//BR
		}
	},
};


// 车体右手坐标系：x前、y左，俯视逆时针为正。下面每个坐标单位均为m。
// 目前±0.236表示轮心相对原点的位置；正负由轮位决定，尺寸数值仍须与实物核对。
// kFL/FR/BL/BRDirectionDeg、kWheelRadiusM、kWheelMaxSpeedDps须有对应定义，不能仅凭变量名推断数值。
inline constexpr std::array<chassis::WheelGeometry, chassis::kWheelCount>
kChassisGeometry{
	{
		{ // 左前 FL
			.x_m                  = +0.236F,
			.y_m                  = +0.236F,
			.drive_direction_deg  = kFLDirectionDeg,
			.radius_m             = kWheelRadiusM,
			.max_output_speed_dps = kWheelMaxSpeedDps,
		},
		{ // 右前 FR
			.x_m                  = +0.236F,
			.y_m                  = -0.236F,
			.drive_direction_deg  = kFRDirectionDeg,
			.radius_m             = kWheelRadiusM,
			.max_output_speed_dps = kWheelMaxSpeedDps,
		},
		{ // 左后 BL
			.x_m                  = -0.236F,
			.y_m                  = +0.236F,
			.drive_direction_deg  = kBLDirectionDeg,
			.radius_m             = kWheelRadiusM,
			.max_output_speed_dps = kWheelMaxSpeedDps,
		},
		{ // 右后 BR
			.x_m                  = -0.236F,
			.y_m                  = -0.236F,
			.drive_direction_deg  = kBRDirectionDeg,
			.radius_m             = kWheelRadiusM,
			.max_output_speed_dps = kWheelMaxSpeedDps,
		},
	}
};
// 这是整车的速度/加速度能力，不是某个电调的电流上限；空配置各项为0，不能用于有效规划。
inline constexpr chassis::MotionLimits
kChassisMotionLimits{
	//todo:电机的限制
};

//todo:底盘运动映射到摇杆通道
// 通道索引与rc::RC_Data_s.channels对应；当前定义被注释，不能认为Task已经有可用映射。
// 恢复时逐项实测：前后、左右平移、左右旋转各自使用哪个物理摇杆轴。
// inline constexpr unsigned kChassisForwardChannel    = 1U;
// inline constexpr unsigned kChassisLeftChannel       = 0U;
// inline constexpr unsigned kChassisYawChannel        = 2U;

// 右手系规定目标vx前正、vy左正、yaw逆时针正；下面的Sign只修正遥控输入方向。
// 三个系数相互独立，+1保留通道符号，-1反转；不能用它们补偿单轮电机安装错误。
inline constexpr float    kChassisForwardSign       = 1.0F;
inline constexpr float    kChassisLeftSign          = 1.0F;
inline constexpr float    kChassisYawSign           = 1.0F;

inline constexpr float    kChassisStickDeadband     = 0.03F; // 归一化幅值死区，0.03表示中位附近3%；须小于1。
inline constexpr auto     kChassisRunSwitch         = rc::Left_Switch_e::MIDDLE; // 当前用左拨杆中位请求运行，仍须安全总许可。
inline constexpr uint32_t kChassisFeedbackTimeoutMs = 100U; // 任一轮反馈达到该年龄即整组撤输出，单位ms而非tick。

/**
 * @brief 四轮速度 PI 的联调初值，尚未经过底盘负载验证。
 * @note 当前 DJI 驱动的速度目标和反馈均为转子 °/s，输出为电调电流指令码，非 A。
 *       应用层输出轴目标接入时须在边界乘实际减速比，不得直接混用两种速度。
 *       后续根据电机型号、负载和功率预算调整；此配置不会注册或使能电机。
 */
inline constexpr PID_Init_Config_s kChassisWheelSpeedPi{
    .kp           = 0.04F,    // P项：速度误差乘kp，误差单位为转子°/s，输出为电流指令码。
    .ki           = 0.05F,    // I项：累计速度误差*实际dt，再乘ki；增大可消除静差，也可能增加超调。
    .kd           = 0.0F,    // D为0，当前只使用PI；不对带噪声的轮速反馈做微分。
    .output_min   = -4000.0F, // PI总输出下限，单位指令码，不能直接解释为-4000A。
    .output_max   = 4000.0F,  // PI总输出上限；需与具体电调协议、负载及整车功率预算核对。
    .integral_min = -2000.0F, // 积分项对输出的贡献下限，不是误差积分的时间上限。
    .integral_max = 2000.0F,  // 积分项贡献上限；与PI实现中的条件积分抗饱和共同限制积累。
};
static_assert(kChassisWheelSpeedPi.kd == 0.0F, "Chassis wheel control uses PI only");

inline constexpr float kChassisControlPeriodS = 0.001F; // 标称控制周期1ms，单位s；实际dt由相邻tick计算。

inline constexpr float kChassisMaxDtS = 0.1F; // 可接受的实际dt上限，超过则停机；不是把异常dt裁成此值继续运动。

#endif //INFANTRY_H723_CHASSISCONFIG_H
