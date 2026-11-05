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
	 * @brief 单轴（Yaw / 瞄准Pitch / 折叠Pitch）运动、控制与通信快照
	 */
	struct AxisSnapshot
	{
		EncoderSample encoder{};               ///< 原始编码器采样数据

		/* ---- 运动学量 (单位: ° / °/s) ---- */
		float position_target_deg{};           ///< 目标位置（相对零位）
		float position_actual_deg{};           ///< 实际反馈位置
		float speed_target_dps{};              ///< 目标角速度
		float speed_actual_dps{};              ///< 实际角速度

		/* ---- 力矩与环路输出 (单位: N·m) ---- */
		float command_torque_nm{};             ///< 最终指令转矩
		float feedback_torque_nm{};            ///< 实际反馈转矩（与指令同方向）
		float position_loop_torque_nm{};       ///< 位置环输出分量
		float speed_loop_torque_nm{};          ///< 速度环输出分量

		/* ---- 链路状态与通信诊断 ---- */
		uint32_t tx_attempts{};                ///< CAN/串口发送尝试次数
		uint32_t tx_failures{};                ///< 发送失败计数
		bool     output_enabled{false};        ///< 电机/控制输出使能状态
	};

	/**
	 * @brief 包含瞄准/折叠 Pitch 的三轴云台全局控制快照
	 */
	struct GimbalControlSnapshot
	{
		AxisSnapshot yaw{};                    ///< 航向轴 (Yaw)
		AxisSnapshot aim_pitch{};              ///< 瞄准 Pitch 轴
		AxisSnapshot folding_pitch{};          ///< 折叠 Pitch 轴

		/* ---- 控制任务时序元数据 ---- */
		uint32_t control_tick{};               ///< 系统控制周期 Tick
		uint32_t control_sequence{};           ///< 帧序号
		float    control_dt_s{};               ///< 本次控制周期（秒）
	};

	using AxisSnapshotProvider = AxisSnapshot (*)();

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

	using ChassisSnapshotProvider = ChassisSnapshot (*)();


	const auto decimal = [](float value, char (&text)[24])
	{
		if (!std::isfinite(value) || std::fabs(value) > 1000000.0F) value = 0.0F;//异常值保护
		const auto milli = static_cast<uint32_t>(std::fabs(static_cast<double>(value)) * 1000.0 + 0.5);//把 float 格式化成带 3 位小数的字符串
		std::snprintf(text, sizeof(text), "%s%lu.%03lu",
					  value < 0.0F && milli != 0U ? "-" : "",
					  static_cast<unsigned long>(milli / 1000U), static_cast<unsigned long>(milli % 1000U));
	};


	/** @brief 单轴八通道：目标/实际位置、目标/实际速度、位置项、速度项、指令及反馈力矩。 */
	inline size_t FormatAxisTelemetryLine(char*                  buffer, size_t capacity,
												  const AxisSnapshot& sps, uint32_t /*now*/)
	{
		if (buffer == nullptr || capacity == 0U) return 0U;
		// 保持无浮点 printf 依赖；格式化仅影响串口显示。

		char        v[8][24]{};
		const float values[]{
			sps.position_target_deg,
			sps.position_actual_deg,
			sps.speed_target_dps,
			sps.speed_actual_dps,
			sps.position_loop_torque_nm,
			sps.speed_loop_torque_nm,
			sps.command_torque_nm,
			sps.feedback_torque_nm
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
		static_assert(std::size(values) == 12U, "Chassis telemetry requires 12 channels");
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
