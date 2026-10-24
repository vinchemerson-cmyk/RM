//
// Created by 19923 on 2026/10/21.
//

#ifndef INFANTRY_H723_CHASSIS_CONTROL_H
#define INFANTRY_H723_CHASSIS_CONTROL_H
#include <cstdint>

namespace chassis
{
	// 明确采用右手系：x前、y左、z上，逆时针yaw为正；与参考代码y右约定不同。
	enum class Mode : uint8_t
	{
		Disabled,
		NoFollow,
		FollowGimbal,
		Spin,
		Direct
	};

	enum class chassis_mode : uint8_t
	{
		CHASSIS_NO_FOLLOW,         // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
		CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随云台,不单独设置pid,以误差角度平方为速度输出
		CHASSIS_ROTATE,            //自旋,同时保持全向机动;当前wz维持定值,todo后续增加不规则的变速策略
		poweroff
	};

	enum class CommandFrame : uint8_t
	{
		Chassis,
		Gimbal
	};


}

#endif //INFANTRY_H723_CHASSIS_CONTROL_H
