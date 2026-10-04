/** @file encoder_telemetry_test.cpp @brief pitch 三通道 FireWater 文本与缓冲区测试。 */
#include <cassert>
#include <cstring>
#include <limits>
#include <initializer_list>
#include "communication/encoder_telemetry.h"

int main()
{
    char buffer[64]{};
    serialTasks::EncoderSnapshot snapshot{};
    const auto check = [&buffer, &snapshot](float target_deg, uint32_t now, const char* expected)
    {
        snapshot.pitch_position_target_deg = target_deg;
        const auto size = serialTasks::FormatPitchTelemetryLine(buffer, sizeof(buffer), snapshot, now);
        assert(size == std::strlen(expected));
        assert(std::strcmp(buffer, expected) == 0);
    };

    check(0.0F, 123U, "0.000,0.000,0.000\n");
    snapshot.pitch = {893U, true, 90U};
    snapshot.pitch_current_a = 1.5F;
    snapshot.pitch_position_actual_deg = -10.0F;
    check(-12.0F, 100U, "-12.000,-10.000,1.500\n");
    snapshot.pitch_current_a = -3.0F;
    snapshot.pitch_position_actual_deg = 9.5F;
    check(12.5F, 101U, "12.500,9.500,-3.000\n");
    check(-0.0004F, 101U, "0.000,9.500,-3.000\n");
    check(-1000000.0F, 89U, "-1000000.000,0.000,0.000\n");
    check(12.5F, 190U, "12.500,9.500,-3.000\n");
    check(12.5F, 191U, "12.500,0.000,0.000\n");
    snapshot.pitch_current_a = std::numeric_limits<float>::quiet_NaN();
    check(0.0F, 90U, "0.000,9.500,0.000\n");
    snapshot.pitch_current_a = 0.0F;
    snapshot.pitch_position_actual_deg = std::numeric_limits<float>::infinity();
    check(0.0F, 90U, "0.000,0.000,0.000\n");
    for (const float invalid : {std::numeric_limits<float>::quiet_NaN(),
                               std::numeric_limits<float>::infinity(), 1.0e30F})
        check(invalid, 90U, "0.000,0.000,0.000\n");

    assert(serialTasks::FormatPitchTelemetryLine(buffer, 8U, snapshot, 90U) == 0U);
    assert(buffer[0] == '\0');
    buffer[0] = 'X';
    assert(serialTasks::FormatPitchTelemetryLine(buffer, 0U, snapshot, 90U) == 0U && buffer[0] == 'X');
    assert(serialTasks::FormatPitchTelemetryLine(nullptr, sizeof(buffer), snapshot, 90U) == 0U);
}
