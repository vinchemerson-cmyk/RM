/** @file dbus.h @brief 无硬件依赖的 18 字节 DBUS 解码及跨 DMA 回调的帧重组。 */
#pragma once
#include <array>
#include <cstddef>
#include <cstring>
#include "rc.h"

namespace rc
{
	/** @brief 解码完整 DR16 帧；失败时保持 result 原值。 */
	inline bool DecodeDbusFrame(const uint8_t* bytes, const size_t size, RC_Data_s& result)
	{
		if (bytes == nullptr || size != FRAME_LENGTH)
			return false;
		RC_Data_s decoded{};
		decoded.channels[0] = (bytes[0] | (bytes[1] << 8U)) & 0x07FFU;
		decoded.channels[1] = ((bytes[1] >> 3U) | (bytes[2] << 5U)) & 0x07FFU;
		decoded.channels[2] = ((bytes[2] >> 6U) | (bytes[3] << 2U) |
							   (bytes[4] << 10U)) & 0x07FFU;
		decoded.channels[3] = ((bytes[4] >> 1U) | (bytes[5] << 7U)) & 0x07FFU;
		for (uint16_t channel : decoded.channels)
			if (channel < CHANNEL_MIN || channel > CHANNEL_MAX)
				return false;
		const uint8_t left  = (bytes[5] >> 6U) & 0x03U;
		const uint8_t right = (bytes[5] >> 4U) & 0x03U;
		if (left == 0U || right == 0U || bytes[12] > 1U || bytes[13] > 1U)
			return false;
		decoded.switch_left  = static_cast<Left_Switch_e>(left);
		decoded.switch_right = static_cast<Right_Switch_e>(right);
		const auto read16    = [](const uint8_t* p) -> uint16_t
		{
			return static_cast<uint16_t>(p[0] | (p[1] << 8U));
		};
		decoded.wheel_raw          = read16(bytes + 16U);
		decoded.received           = true;
		result                     = decoded;
		return true;
	}

	/** @brief 缓存残帧；非法候选逐字节滑动，合法帧不依赖固定首字节。 */
	class DbusStreamParser
	{
		public:
			/** @brief 清除断线前的残帧。 */
			void Reset() { used_ = 0U; }

			/** @brief 加入 DMA 数据，返回合法帧数，result 保存最后一帧。 */
			size_t Push(const uint8_t* bytes, size_t size, RC_Data_s& result)
			{
				if (bytes == nullptr)
					return 0U;
				size_t frames = 0U;
				for (size_t i = 0; i < size; ++i)
				{
					pending_[used_++] = bytes[i];
					if (used_ < FRAME_LENGTH)
						continue;
					if (DecodeDbusFrame(pending_.data(), FRAME_LENGTH, result))
					{
						++frames;
						used_ = 0U;
					} else
					{
						std::memmove(pending_.data(), pending_.data() + 1U, FRAME_LENGTH - 1U);
						used_ = FRAME_LENGTH - 1U;
					}
				}
				return frames;
			}

		private:
			std::array<uint8_t, FRAME_LENGTH> pending_{};
			size_t                            used_{};
	};
} // namespace rc
