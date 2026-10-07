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
    char yawBuffer[128]{};
    const auto checkYaw = [&](uint32_t now, const char* expected)
    {
        const auto size = serialTasks::FormatYawTelemetryLine(yawBuffer, sizeof(yawBuffer), snapshot, now);
        assert(size == std::strlen(expected));
        assert(std::strcmp(yawBuffer, expected) == 0);
    };
    checkYaw(100U, "0.000,0.000,0.000,0.000,-1,0.000\n");
    snapshot.yaw = {3741U, true, 90U};
    snapshot.yaw_target_speed_dps = -360.5F;
    snapshot.yaw_position_actual_deg = -358.0F;
    snapshot.yaw_speed_dps = -6.0F;
    snapshot.yaw_command_current_a = -0.25F;
    snapshot.yaw_feedback_current_a = -0.4F;
    checkYaw(100U, "-360.500,-358.000,-6.000,-0.250,10,-0.400\n");
    checkYaw(191U, "-360.500,-358.000,-6.000,-0.250,101,-0.400\n"); // 过期数据不隐藏。
    snapshot.yaw.last_update_tick = UINT32_MAX - 2U;
    checkYaw(2U, "-360.500,-358.000,-6.000,-0.250,5,-0.400\n");
    snapshot.yaw_speed_dps = std::numeric_limits<float>::quiet_NaN();
    checkYaw(2U, "-360.500,-358.000,0.000,-0.250,5,-0.400\n");
    snapshot.yaw_feedback_current_a = std::numeric_limits<float>::infinity();
    checkYaw(2U, "-360.500,-358.000,0.000,-0.250,5,0.000\n");
    assert(serialTasks::FormatYawTelemetryLine(yawBuffer, 8U, snapshot, 2U) == 0U);
    assert(yawBuffer[0] == '\0');
    assert(serialTasks::FormatYawTelemetryLine(nullptr, 128U, snapshot, 2U) == 0U);
    assert(serialTasks::FormatYawTelemetryLine(yawBuffer, 0U, snapshot, 2U) == 0U);
    char controlBuffer[256]{};
    snapshot = {};
    snapshot.control_tick = 100U;
    snapshot.control_sequence = 7U;
    snapshot.control_dt_s = 0.001F;
    snapshot.yaw = {3741U, true, 99U};
    snapshot.yaw_position_target_deg = -200.0F;
    snapshot.yaw_position_actual_deg = -201.0F;
    snapshot.yaw_speed_dps = 6.0F;
    snapshot.yaw_position_current_a = 0.1F;
    snapshot.yaw_speed_current_a = -0.06F;
    snapshot.yaw_command_current_a = 0.04F;
    snapshot.yaw_feedback_current_a = -0.2F;
    snapshot.yaw_output_enabled = true;
    snapshot.yaw_tx_failures = 3U;
    const char* expected = "-200.000,-201.000,6.000,0.100,-0.060,0.040,-0.200\n";
    assert(serialTasks::FormatYawControlTelemetryLine(controlBuffer, sizeof(controlBuffer), snapshot, 999U)
           == std::strlen(expected));
    assert(std::strcmp(controlBuffer, expected) == 0); // 严格只输出指定的七列。
    snapshot.yaw.seen = false;
    assert(serialTasks::FormatYawControlTelemetryLine(controlBuffer, sizeof(controlBuffer), snapshot, 0U) > 0U);
    assert(std::strcmp(controlBuffer, expected) == 0); // 缺帧标记不增加通道，不隐藏最近反馈。
    snapshot.yaw_speed_current_a = std::numeric_limits<float>::quiet_NaN();
    assert(serialTasks::FormatYawControlTelemetryLine(controlBuffer, sizeof(controlBuffer), snapshot, 0U) > 0U);
    assert(std::strcmp(controlBuffer, "-200.000,-201.000,6.000,0.100,0.000,0.040,-0.200\n") == 0);
    assert(serialTasks::FormatYawControlTelemetryLine(controlBuffer, 8U, snapshot, 0U) == 0U);
    assert(controlBuffer[0] == '\0');
    assert(serialTasks::FormatYawControlTelemetryLine(nullptr, 256U, snapshot, 0U) == 0U);
}
