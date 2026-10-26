/** @file interboard_wire.h @brief 板间线路协议纯逻辑；FD单帧与经典CAN兼容分片。 */
#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include "can/can_frame.h"

namespace interboard::wire
{
    inline constexpr uint8_t kFdHeaderSize = 4U;
    inline constexpr uint8_t kMaxPayload = 64U - kFdHeaderSize;
    inline constexpr uint8_t kClassicChunk = 6U;
    inline constexpr uint8_t kVersion = 0xA1U;
    struct Frame
    {
        std::array<uint8_t, 64> data{};
        uint32_t received_tick{}; // 本地ISR接收时间，不传输到另一块板。
        uint8_t length{};         // 实际DLC对应字节数，含协议头和填充。
        bool fd{};
    };
    struct PayloadView
    {
        uint8_t topic{}, sequence{}, length{};
        const uint8_t* bytes{}; // 借用Frame内部存储，不能在Frame销毁后使用。
    };

    /** FD: [topic,8位序号,真实业务长度,版本0xA1,payload...,零填充]，最多60字节业务。 */
    inline bool EncodeFd(uint8_t topic, uint8_t sequence, const uint8_t* bytes, uint8_t length, Frame& frame)
    {
        frame = {};
        if (length > kMaxPayload || (length != 0 && bytes == nullptr) ||
            (topic == 0 ? (length != 0 || sequence != 0) : length == 0)) return false;
        frame.fd = true;
        frame.length = canFrame::PaddedLength(kFdHeaderSize + length);
        frame.data[0] = topic; frame.data[1] = sequence;
        frame.data[2] = length; frame.data[3] = kVersion;
        if (length != 0) std::memcpy(frame.data.data() + kFdHeaderSize, bytes, length);
        return true;
    }

    inline bool DecodeFd(const Frame& frame, PayloadView& payload)
    {
        payload = {};
        if (!frame.fd || frame.length < kFdHeaderSize || frame.length > 64 || frame.data[3] != kVersion)
            return false;
        const uint8_t length = frame.data[2];
        if (length > kMaxPayload || frame.length != canFrame::PaddedLength(kFdHeaderSize + length) ||
            (frame.data[0] == 0 ? (length != 0 || frame.data[1] != 0) : length == 0)) return false;
        for (unsigned i = kFdHeaderSize + length; i < frame.length; ++i)
            if (frame.data[i] != 0) return false;
        payload = {frame.data[0], frame.data[1], length, frame.data.data() + kFdHeaderSize};
        return true;
    }

    /** 经典格式保持旧的2字节头；序号低4位+分片号，各片最多6字节。 */
    inline bool EncodeClassic(uint8_t topic, uint8_t sequence, uint8_t fragment,
                              const uint8_t* bytes, uint8_t length, Frame& frame)
    {
        frame = {};
        if (length > kClassicChunk || (length != 0 && bytes == nullptr) || fragment > 15U ||
            (topic == 0 ? (length != 0 || fragment != 0 || sequence != 0) : length == 0)) return false;
        frame.length = length + 2U;
        frame.data[0] = topic;
        frame.data[1] = static_cast<uint8_t>(((sequence & 0x0FU) << 4U) | fragment);
        if (length != 0) std::memcpy(frame.data.data() + 2, bytes, length);
        return true;
    }

    inline bool IsClassicHeartbeat(const Frame& frame)
    { return !frame.fd && frame.length == 2 && frame.data[0] == 0 && frame.data[1] == 0; }

    struct ClassicAssembly
    {
        std::array<uint8_t, kMaxPayload> bytes{};
        uint32_t last_tick{};
        uint8_t sequence{}, next_fragment{};
        bool assembling{};

        // 返回本片是否有效；complete为true时bytes中有一份完整固定长度消息。
        bool Accept(const Frame& frame, uint8_t expectedLength, uint32_t timeoutTicks, bool& complete)
        {
            complete = false;
            if (frame.fd || frame.length < 3 || frame.length > 8 || expectedLength == 0 || expectedLength > kMaxPayload)
                return Reject();
            const uint8_t seq = frame.data[1] >> 4U, fragment = frame.data[1] & 0x0FU;
            const unsigned offset = fragment * kClassicChunk;
            if (offset >= expectedLength) return Reject();
            const unsigned count = expectedLength - offset < kClassicChunk ? expectedLength - offset : kClassicChunk;
            if (frame.length != count + 2U) return Reject();
            if (fragment == 0) { assembling = true; sequence = seq; next_fragment = 0; }
            if (!assembling || sequence != seq || next_fragment != fragment ||
                (fragment != 0 && frame.received_tick - last_tick >= timeoutTicks)) return Reject();
            std::memcpy(bytes.data() + offset, frame.data.data() + 2, count);
            last_tick = frame.received_tick;
            ++next_fragment;
            complete = offset + count == expectedLength;
            if (complete) assembling = false;
            return true;
        }
    private:
        bool Reject() { assembling = false; return false; }
    };
}
