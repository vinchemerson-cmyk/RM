//
// Created by 19923 on 2026/10/31.
//

#ifndef INFANTRY_H723_ROBOTCOMMAND_HPP
#define INFANTRY_H723_ROBOTCOMMAND_HPP
#include <cstdint>
#include <type_traits>

namespace chassis
{
	/**
	 * @brief 底盘速度指令；固定 8 字节，发送/接收两侧必须使用同一布局。
	 * @note vx/vy 为车体坐标速度的千分之一（mm/s），范围 ±32.767 m/s，分辨率 1 mm/s。
	 *       yaw 为车体偏航角速度的十分之一（0.1 °/s），范围 ±3276.7 °/s，分辨率 0.1 °/s。
	 *       sequence 由发送侧递增，仅用于诊断丢帧；接收侧按本地到达时间判超时。
	 *       两板均为小端 ARM，消息中心按原始字节复制，不做字节序转换。
	 */
	struct Command
	{
		std::int16_t  com_vx_mmps{};
		std::int16_t  com_vy_mmps{};
		std::int16_t  com_yaw_ddps{};
		std::uint16_t sequence{};
	};
	static_assert(sizeof(Command) == 8U, "Chassis command layout must stay 8 bytes");
	static_assert(std::is_trivially_copyable_v<Command>, "Message center copies raw bytes");

	inline constexpr std::uint8_t kCommandTopicId = 1U; // 0 保留给板间心跳。
	inline char                   kCommandTopicName[] = "chassis/command";
}

namespace gimbal
{
	struct Command
	{
		std::int16_t  vx_mmps{};
		std::int16_t  vy_mmps{};
		std::int16_t  yaw_ddps{};
		std::uint16_t sequence{};

		std:: uint8_t poweroff{};     //失能
		std:: uint8_t NoFollow{};     //不跟随
		std:: uint8_t FollowGimbal{}; //跟随
		std:: uint8_t Spin{};         //小陀螺
	};
}
#endif //INFANTRY_H723_ROBOTCOMMAND_HPP
