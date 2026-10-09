/** @file dji_protocol.h @brief 根据电机反馈 ID 计算指令帧和两字节槽位。 */
#pragma once
#include <cstdint>

struct DjiCommandAddress
{
	uint32_t frame_id{};
	uint8_t  slot{};
};

/** @brief GM6020 电流模式指令码范围；±16384 对应转矩电流 ±3A，反馈电流按同一比例解析。 */
inline constexpr float kDjiGm6020CurrentCommandLimit = 16384.0F;
inline constexpr float kDjiGm6020CommandUnitsPerAmp  = kDjiGm6020CurrentCommandLimit / 3.0F;

/** @brief GM6020 使用电流指令，支持 ID1..7；C610/C620 支持 ID1..8，协议不变。 */
inline bool DjiResolveCommand(bool gm6020, uint32_t feedback_id, DjiCommandAddress& address)
{
	const uint32_t first = gm6020 ? 0x205U : 0x201U;
	const uint32_t count = gm6020 ? 7U : 8U;
	if (feedback_id < first || feedback_id >= first + count)
		return false;
	const uint32_t offset = feedback_id - first;
	address.frame_id      = offset < 4U ? (gm6020 ? 0x1FEU : 0x200U) : (gm6020 ? 0x2FEU : 0x1FFU);
	address.slot          = static_cast<uint8_t>(offset % 4U);
	return true;
}
