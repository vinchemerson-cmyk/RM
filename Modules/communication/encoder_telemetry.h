/** @file encoder_telemetry.h @brief 电机快照和 UART7 FireWater 文本格式；无 HAL 依赖。 */
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace serialTasks
{
	struct EncoderSample
	{
		uint16_t raw{};
		bool     seen{};//表示收到过合法反馈；
		uint32_t last_update_tick{};
	};

	/**
	 * @brief 编码器快照
	 */
	struct EncoderSnapshot
	{
		EncoderSample pitch{}, yaw{};
		/** @brief GM6020 反馈帧 DATA[4:5] 换算后的实际转矩电流，单位 A。 */
		float         pitch_current_a{};
		/** @brief pitch 位置目标和反馈位置，相对水平零位，单位 °。 */
		float         pitch_position_target_deg{};
		float         pitch_position_actual_deg{};
		/** @brief连续角度（°）、速度（°/s）和方向处理后的指令电流（A）。 */
		float         yaw_position_target_deg{};
		float         yaw_position_actual_deg{};
		float         yaw_speed_dps{};
		float         pitch_speed_dps{};
		float         yaw_target_speed_dps{}; ///< 位置模式：摇杆给定的目标角度变化率；非独立速度环目标。
		float         pitch_target_speed_dps{};
		float         yaw_command_current_a{};
		float         pitch_command_current_a{};
		/** @brief 反馈转矩电流（A），与指令电流采用相同逻辑方向。 */
		float         yaw_feedback_current_a{};
		float         pitch_feedback_current_a{};
		/** @brief 以下字段与目标/反馈一起在控制任务内发布，不重新采反馈。 */
		uint32_t      control_tick{};
		uint32_t      control_sequence{};
		float         control_dt_s{};
		float         yaw_position_current_a{};
		float         pitch_position_current_a{};
		float         yaw_speed_current_a{};
		float         pitch_speed_current_a{};
		bool          yaw_output_enabled{};
		uint32_t      yaw_tx_attempts{};
		uint32_t      yaw_tx_failures{};
	};

	using EncoderProvider = EncoderSnapshot (*)();//PublishYawControlSnapshot()

	/** @brief 单轮 UART7 遥测值；不含时间戳，避免大 tick 被 decimal() 截断。 */
	struct WheelTelemetrySample
	{
		float target_speed_dps{};
		float actual_speed_dps{};
		float feedback_current_a{};
	};

	/** @brief 底盘四轮快照，轮序固定为 FL/FR/BL/BR，与 chassis::Wheel 一致。 */
	struct ChassisSnapshot
	{
		std::array<WheelTelemetrySample, 4U> wheels{};
	};

	using ChassisProvider = ChassisSnapshot (*)();


	const auto decimal = [](float value, char (&text)[24])
	{
		if (!std::isfinite(value) || std::fabs(value) > 1000000.0F) value = 0.0F;//异常值保护
		const auto milli = static_cast<uint32_t>(std::fabs(static_cast<double>(value)) * 1000.0 + 0.5);//把 float 格式化成带 3 位小数的字符串
		std::snprintf(text, sizeof(text), "%s%lu.%03lu",
					  value < 0.0F && milli != 0U ? "-" : "",
					  static_cast<unsigned long>(milli / 1000U), static_cast<unsigned long>(milli % 1000U));
	};


	/** @brief 同步控制八通道：目标/实际位置、目标/实际速度、位置项、速度项、指令及反馈电流。 */
	inline size_t FormatYawControlTelemetryLine(char*                  buffer, size_t capacity,
												const EncoderSnapshot& sps, uint32_t /*now*/)
	{
		if (buffer == nullptr || capacity == 0U) return 0U;
		// 保持无浮点 printf 依赖；格式化仅影响串口显示。

		char        v[8][24]{};
		const float values[]{
			sps.yaw_position_target_deg,
			sps.yaw_position_actual_deg,
			sps.yaw_target_speed_dps,
			sps.yaw_speed_dps,
			sps.yaw_position_current_a,
			sps.yaw_speed_current_a,
			sps.yaw_command_current_a,
			sps.yaw_feedback_current_a
		};
		for (size_t i = 0; i < 8U; ++i) decimal(values[i], v[i]);
		const int count = std::snprintf(buffer, capacity,
										"%s,%s,%s,%s,%s,%s,%s,%s\n",
										v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
		if (count <= 0 || static_cast<size_t>(count) >= capacity)
		{
			buffer[0] = '\0';
			return 0U;
		}
		return static_cast<size_t>(count);
	}



	inline size_t FormatPitchControlTelemetryLine(char*                  buffer, size_t capacity,
												  const EncoderSnapshot& sps, uint32_t /*now*/)
	{
		if (buffer == nullptr || capacity == 0U) return 0U;
		// 保持无浮点 printf 依赖；格式化仅影响串口显示。

		char        v[8][24]{};
		const float values[]{
			sps.pitch_position_target_deg,
			sps.pitch_position_actual_deg,
			sps.pitch_target_speed_dps,
			sps.pitch_speed_dps,
			sps.pitch_position_current_a,
			sps.pitch_speed_current_a,
			sps.pitch_command_current_a,
			sps.pitch_feedback_current_a
		};
		for (size_t i = 0; i < 8U; ++i) decimal(values[i], v[i]);
		const int count = std::snprintf(buffer, capacity,
										"%s,%s,%s,%s,%s,%s,%s,%s\n",
										v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
		if (count <= 0 || static_cast<size_t>(count) >= capacity)
		{
			buffer[0] = '\0';
			return 0U;
		}
		return static_cast<size_t>(count);
	}


	/** @brief 底盘四轮十二通道：FL/FR/BL/BR × 目标轮速、实际轮速、反馈电流。 */
	inline size_t FormatChassisTelemetryLine(char*                  buffer, size_t capacity,
											  const ChassisSnapshot& sps, uint32_t /*now*/)
	{
		if (buffer == nullptr || capacity == 0U) return 0U;

		char        v[12][24]{};
		const float values[]{
			sps.wheels[0].target_speed_dps, sps.wheels[0].actual_speed_dps, sps.wheels[0].feedback_current_a,
			sps.wheels[1].target_speed_dps, sps.wheels[1].actual_speed_dps, sps.wheels[1].feedback_current_a,
			sps.wheels[2].target_speed_dps, sps.wheels[2].actual_speed_dps, sps.wheels[2].feedback_current_a,
			sps.wheels[3].target_speed_dps, sps.wheels[3].actual_speed_dps, sps.wheels[3].feedback_current_a
		};
		static_assert(sizeof(values) / sizeof(values[0]) == 12U, "Chassis telemetry requires 12 channels");
		for (size_t i = 0; i < 12U; ++i) decimal(values[i], v[i]);
		const int count = std::snprintf(buffer, capacity,
										"%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
										v[0], v[1], v[2], v[3], v[4], v[5],
										v[6], v[7], v[8], v[9], v[10], v[11]);
		if (count <= 0 || static_cast<size_t>(count) >= capacity)
		{
			buffer[0] = '\0';
			return 0U;
		}
		return static_cast<size_t>(count);
	}


}
