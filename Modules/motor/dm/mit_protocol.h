/** @file mit_protocol.h @brief MIT八字节编解码；不依赖HAL，不执行PID。 */
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace dmProtocol
{
	struct Limits
	{
		// 达妙调试助手「参数设置」页的出厂预设值。
		//todo: 说明书明确要求: 发送控制命令时必须与驱动器内设定值一致, 否则命令会被等比例缩放。
		//   见 DM-J4340-2EC V1.1 减速电机使用说明书 V1.3 第 14~15 页:
		//   "Pos、Vel、Torque 分别预设为 ±12.5、±30、±10; KP、KD 默认 0.0~500.0、0.0~5.0"。
		// 若在调试助手里改过范围, 必须同步修改本结构, 否则力矩/位置指令会按比例失真。
		float position_min{-12.5F}, position_max{12.5F};
		float velocity_max{30.0F},  torque_max{10.0F};
		float kp_max{500.0F},       kd_max{5.0F};
	};

	struct Command
	{
		float position{};
		float velocity{};
		float torque{};
		float kp{};
		float kd{};
	};

	struct Feedback
	{
		uint8_t motor_id{}, state{};
		float   position{}, velocity{}, torque{}, mos_temperature{}, rotor_temperature{};
	};

	inline bool Valid(const Limits& l)
	{
		return std::isfinite(l.position_min) && std::isfinite(l.position_max) &&
			   l.position_min < 0.0F && l.position_max > 0.0F &&
			   std::isfinite(l.position_max - l.position_min) &&
			   std::isfinite(l.velocity_max) && l.velocity_max > 0.0F &&
			   std::isfinite(l.velocity_max * 2.0F) &&
			   std::isfinite(l.torque_max) && l.torque_max > 0.0F &&
			   std::isfinite(l.torque_max * 2.0F) &&
			   std::isfinite(l.kp_max) && l.kp_max > 0.0F &&
			   std::isfinite(l.kd_max) && l.kd_max > 0.0F;
	}

	inline uint16_t Encode(float value, float low, float high, unsigned bits)
	{
		value              = value < low ? low : (value > high ? high : value);
		const double scale = (1U << bits) - 1U;
		return static_cast<uint16_t>((static_cast<double>(value) - low) /
									 (static_cast<double>(high) - low) * scale);
	}

	inline float Decode(uint16_t value, float low, float high, unsigned bits)
	{
		return static_cast<float>(low + static_cast<double>(value) * (static_cast<double>(high) - low) / (
									  (1U << bits) - 1U));
	}

	//打包命令帧
	inline bool Pack(const Command& cmd, const Limits& limits, std::array<uint8_t, 8>& bytes)
	{
		if (!Valid(limits) || !std::isfinite(cmd.position) || !std::isfinite(cmd.velocity) ||
			!std::isfinite(cmd.torque) || !std::isfinite(cmd.kp) || !std::isfinite(cmd.kd))
			return false;
		const uint16_t p  = Encode(cmd.position, limits.position_min, limits.position_max, 16U);
		const uint16_t v  = Encode(cmd.velocity, -limits.velocity_max, limits.velocity_max, 12U);
		const uint16_t t  = Encode(cmd.torque, -limits.torque_max, limits.torque_max, 12U);
		const uint16_t kp = Encode(cmd.kp, 0.0F, limits.kp_max, 12U);
		const uint16_t kd = Encode(cmd.kd, 0.0F, limits.kd_max, 12U);
		bytes             = {
			static_cast<uint8_t>(p >> 8U), static_cast<uint8_t>(p), static_cast<uint8_t>(v >> 4U),
			static_cast<uint8_t>(((v & 0xFU) << 4U) | (kp >> 8U)), static_cast<uint8_t>(kp),
			static_cast<uint8_t>(kd >> 4U), static_cast<uint8_t>(((kd & 0xFU) << 4U) | (t >> 8U)),
			static_cast<uint8_t>(t)
		};
		bool mode_prefix = true;
		for (unsigned i = 0U; i < 7U; ++i) mode_prefix = mode_prefix && bytes[i] == 0xFFU;
		if (mode_prefix) return false; // 防止极端目标量化成特殊命令前缀，意外清错/使能/置零。
		return true;
	}

	inline bool Unpack(const uint8_t* bytes, unsigned length, const Limits& limits, Feedback& out)
	{
		if (bytes == nullptr || length != 8U || !Valid(limits)) return false;
		out.motor_id = bytes[0] & 0xFU;
		out.state    = bytes[0] >> 4U;
		out.position = Decode(static_cast<uint16_t>((bytes[1] << 8U) | bytes[2]), limits.position_min,
							  limits.position_max, 16U);
		out.velocity = Decode(static_cast<uint16_t>((bytes[3] << 4U) | (bytes[4] >> 4U)), -limits.velocity_max,
							  limits.velocity_max, 12U);
		out.torque = Decode(static_cast<uint16_t>(((bytes[4] & 0xFU) << 8U) | bytes[5]), -limits.torque_max,
							limits.torque_max, 12U);
		out.mos_temperature   = bytes[6];
		out.rotor_temperature = bytes[7];
		return true;
	}
}
