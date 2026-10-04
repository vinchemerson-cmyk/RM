/** @file encoder_telemetry.h @brief 电机快照和 UART7 FireWater 文本格式；无 HAL 依赖。 */
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cmath>

namespace serialTasks
{
/** @brief seen 表示收到过合法反馈；age 仅作观测，不参与电机掉线停机。 */
struct EncoderSample
{
    uint16_t raw{};
    bool seen{};
    uint32_t last_update_tick{};
};
struct EncoderSnapshot
{
    EncoderSample pitch{}, yaw{};
    /** @brief GM6020 反馈帧 DATA[4:5] 换算后的实际转矩电流，单位 A。 */
    float pitch_current_a{};
    /** @brief pitch 位置目标和反馈位置，相对水平零位，单位 °。 */
    float pitch_position_target_deg{};
    float pitch_position_actual_deg{};
};
using EncoderProvider = EncoderSnapshot (*)();

/**
 * @brief 格式化 FireWater 三通道：目标位置、实际位置、实际电流。
 * @param buffer 输出缓冲区。
 * @param capacity 含结尾 NUL 的缓冲区容量。
 * @param snapshot 同一临界区采集的目标和反馈。
 * @param now 当前 RTOS tick，单位 ms，用于标明反馈新鲜度。
 * @return 可发送的字节数（不含 NUL）；参数无效或空间不足返回 0。
 */
inline size_t FormatPitchTelemetryLine(char* buffer, size_t capacity,
                                      const EncoderSnapshot& snapshot, uint32_t now)
{
    if (buffer == nullptr || capacity == 0U) return 0U;
    const auto format_decimal = [](float value, char (&text)[24])
    {
        // FireWater 只接受数值；无效值输出 0.000，不限制或修改控制目标。
        if (!std::isfinite(value) || std::fabs(value) > 1000000.0F)
        {
            std::snprintf(text, sizeof(text), "0.000");
            return;
        }
        const auto milli = static_cast<uint32_t>(std::fabs(static_cast<double>(value)) * 1000.0 + 0.5);
        std::snprintf(text, sizeof(text), "%s%lu.%03lu",
            value < 0.0F && milli != 0U ? "-" : "",
            static_cast<unsigned long>(milli / 1000U),
            static_cast<unsigned long>(milli % 1000U));
    };
    char target_text[24]{}, actual_text[24]{}, current_text[24]{};
    format_decimal(snapshot.pitch_position_target_deg, target_text);
    // 只影响串口显示；反馈过期不会改变测试阶段的电机使能策略。
    if (snapshot.pitch.seen && now - snapshot.pitch.last_update_tick <= 100U)
    {
        format_decimal(snapshot.pitch_position_actual_deg, actual_text);
        format_decimal(snapshot.pitch_current_a, current_text);
    }
    else
    {
        format_decimal(0.0F, actual_text);
        format_decimal(0.0F, current_text);
    }
    const int count = std::snprintf(buffer, capacity, "%s,%s,%s\n",
                                    target_text, actual_text, current_text);
    if (count <= 0 || static_cast<size_t>(count) >= capacity)
    {
        buffer[0] = '\0';
        return 0U;
    }
    return static_cast<size_t>(count);
}
}
