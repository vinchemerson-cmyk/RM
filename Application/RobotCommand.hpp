//
// Created by 19923 on 2026/10/31.
//

#ifndef INFANTRY_H723_ROBOTCOMMAND_HPP
#define INFANTRY_H723_ROBOTCOMMAND_HPP
#include <cstdint>
#include <type_traits>

namespace chassis
{
	/** @brief 底盘运动模式；写入 Command::mode，发送/接收两侧必须使用同一数值。 */
	enum class Mode : std::uint8_t
	{
		Disabled = 0,
		NoFollow = 1,
		FollowGimbal = 2,
		Spin = 3,
		Direct = 4,
	};

	/**
	 * @brief 底盘速度指令；固定 10 字节，发送/接收两侧必须使用同一布局。
	 * @note vx/vy 为车体坐标速度的千分之一（mm/s），范围 ±32.767 m/s，分辨率 1 mm/s。
	 *       yaw 为车体偏航角速度的十分之一（0.1 °/s），范围 ±3276.7 °/s，分辨率 0.1 °/s。
	 *       mode 使用 chassis::Mode 的数值；reserved 必须置 0，保证显式布局。
	 *       sequence 由发送侧递增，仅用于诊断丢帧；接收侧按本地到达时间判超时。
	 *       两板均为小端 ARM，消息中心按原始字节复制，不做字节序转换。
	 */
	struct Command
	{
		std::int16_t  com_vx_mmps{};
		std::int16_t  com_vy_mmps{};
		std::int16_t  com_yaw_ddps{};
		std::uint8_t  mode{};     // chassis::Mode
		std::uint8_t  reserved{}; // 显式填充，发送方必须置 0
		std::uint16_t sequence{};
	};
	static_assert(sizeof(Command) == 10U, "Chassis command layout must stay 10 bytes");
	static_assert(std::is_trivially_copyable_v<Command>, "Message center copies raw bytes");

	inline constexpr std::uint8_t kCommandTopicId = 1U; // 0 保留给板间心跳。
	inline char                   kCommandTopicName[] = "chassis/command";
	inline constexpr std::uint32_t kCommandTimeoutMs = 100U;

	// 安全状态与速度指令分开传输：即使没有新的速度指令，云台板也要持续刷新安全状态。
	inline constexpr std::uint16_t kSafetyOutputEnabled = 1U << 0U; // 云台板 safety::ShouldOutput()
	inline constexpr std::uint16_t kSafetyRcOnline      = 1U << 1U; // 云台板 RC 在线
	inline constexpr std::uint16_t kSafetyEStopped      = 1U << 2U; // 云台板急停
	inline constexpr std::uint16_t kSafetyBothDown      = 1U << 3U; // 双拨杆 DOWN
	inline constexpr std::uint16_t kSafetyRunRequested  = 1U << 4U; // 底盘运行拨杆请求

	struct SafetyState
	{
		std::uint16_t flags{};
		std::uint16_t sequence{};
	};
	static_assert(sizeof(SafetyState) == 4U, "Chassis safety state layout must stay 4 bytes");
	static_assert(std::is_trivially_copyable_v<SafetyState>, "Message center copies raw bytes");

	inline constexpr std::uint8_t kSafetyTopicId = 2U;
	inline char                   kSafetyTopicName[] = "chassis/safety";
	inline constexpr std::uint32_t kSafetyTimeoutMs = 100U;
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
	inline constexpr std::uint8_t kCommandTopicId = 1U; // 0 保留给板间心跳。
	inline char                   kCommandTopicName[] = "chassis/command";
	inline constexpr std::uint32_t kCommandTimeoutMs = 100U;

}
#endif //INFANTRY_H723_ROBOTCOMMAND_HPP
