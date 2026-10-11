/** @file encoder_telemetry_test.cpp @brief yaw 控制快照 FireWater 文本与缓冲区测试。 */
#include <cassert>
#include <cstring>
#include <limits>
#include <initializer_list>
#include "communication/encoder_telemetry.h"

int main()
{
	serialTasks::EncoderSnapshot snapshot{};
	char controlBuffer[256]{};
	snapshot                         = {};
	snapshot.control_tick            = 100U;
	snapshot.control_sequence        = 7U;
	snapshot.control_dt_s            = 0.001F;
	snapshot.yaw                     = {3741U, true, 99U};
	snapshot.yaw_position_target_deg = -200.0F;
	snapshot.yaw_position_actual_deg = -201.0F;
	snapshot.yaw_target_speed_dps    = 120.0F;
	snapshot.yaw_speed_dps           = 6.0F;
	snapshot.yaw_position_current_a  = 0.1F;
	snapshot.yaw_speed_current_a     = -0.06F;
	snapshot.yaw_command_current_a   = 0.04F;
	snapshot.yaw_feedback_current_a  = -0.2F;
	snapshot.yaw_output_enabled      = true;
	snapshot.yaw_tx_failures         = 3U;
	const char* expected             = "-200.000,-201.000,120.000,6.000,0.100,-0.060,0.040,-0.200\n";
	assert(serialTasks::FormatYawControlTelemetryLine(controlBuffer, sizeof(controlBuffer), snapshot, 999U)
		   == std::strlen(expected));
	assert(std::strcmp(controlBuffer, expected) == 0); // 严格只输出指定的八列。
	snapshot.yaw.seen = false;
	assert(serialTasks::FormatYawControlTelemetryLine(controlBuffer, sizeof(controlBuffer), snapshot, 0U) > 0U);
	assert(std::strcmp(controlBuffer, expected) == 0); // 缺帧标记不增加通道，不隐藏最近反馈。
	snapshot.yaw_speed_current_a = std::numeric_limits<float>::quiet_NaN();
	assert(serialTasks::FormatYawControlTelemetryLine(controlBuffer, sizeof(controlBuffer), snapshot, 0U) > 0U);
	assert(std::strcmp(controlBuffer, "-200.000,-201.000,120.000,6.000,0.100,0.000,0.040,-0.200\n") == 0);
	assert(serialTasks::FormatYawControlTelemetryLine(controlBuffer, 8U, snapshot, 0U) == 0U);
	assert(controlBuffer[0] == '\0');
	assert(serialTasks::FormatYawControlTelemetryLine(nullptr, 256U, snapshot, 0U) == 0U);
}
