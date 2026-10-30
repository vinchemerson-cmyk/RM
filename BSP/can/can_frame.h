/** @file can_frame.h @brief CAN FD DLC与实际长度的纯逻辑工具，无HAL依赖。 */
#pragma once
#include <array>
#include <cstdint>

namespace canFrame
{
    inline constexpr std::array<uint8_t, 16> kLengths{0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    inline constexpr uint8_t kInvalidDlc = 0xFFU;
    constexpr uint8_t DlcForLength(unsigned length)
    {
    	for (size_t i = 0; i < kLengths.size(); ++i)
            if (length <= kLengths[i]) return i;
        return kInvalidDlc;
    }
    constexpr uint8_t LengthForDlc(unsigned dlc)
    { return dlc < kLengths.size() ? kLengths[dlc] : 0xFFU; }
    constexpr uint8_t PaddedLength(unsigned length)
    { return LengthForDlc(DlcForLength(length)); }
}
