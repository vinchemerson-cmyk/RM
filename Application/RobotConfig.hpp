/**
 * @file RobotConfig.hpp
 * @brief 整车编译期配置: 板卡角色、CAN 资源分配和各功能模块的启停开关。
 *
 * 两块板 (云台板/底盘板) 共用本工程, 烧录前必须确认 INFANTRY_BOARD_GIMBAL:
 *   - 取 1: 云台板, 装配遥控/安全/云台/发射; 双板链路发送 0x301 / 接收 0x302。
 *   - 取 0: 底盘板, 装配安全/底盘; 双板链路发送 0x302 / 接收 0x301。
 *   两块板必须使用相反的值, 否则双板链路收不到对方心跳, 永远显示离线。
 *
 * 角色差异集中在本文件; 模块源码不得再按板卡角色复制逻辑。
 *
 * FDCAN 资源分配 (STM32H723 共 3 路 FDCAN, 具体模式见 board_can_config.h):
 *   云台板: FDCAN1 FD (云台 DM), FDCAN2 Classic (摩擦轮), FDCAN3 FD (空闲), 板间 FDCAN1。
 *   底盘板: FDCAN1 Classic (底盘 DJI), FDCAN2 FD (板间), FDCAN3 Classic (空闲)。
 */

//
// Created by 19923 on 2026/9/24.
//
#pragma once
#ifndef ROBOTINIT_HPP
#define ROBOTINIT_HPP
#include <cstdint>
#include "board_can_config.h"

// 板卡角色由CMake的INFANTRY_BOARD_GIMBAL(1/0)统一提供给C/C++。
// 每板CAN1/2/3的模式、时序和板间所在控制器在board_can_config.h独立配置；默认接线映射保持不变。

namespace InfantryInitialization
{
	enum class BoardRole : std::uint8_t
	{
		Gimbal,
		Chassis
	};
	inline constexpr BoardRole kBoardRole =
#if INFANTRY_BOARD_GIMBAL
		BoardRole::Gimbal;
#else
		BoardRole::Chassis;
#endif

	// ---------- 两板共有 ----------
	inline constexpr bool ENABLE_USB_SERIAL    = true;  // USB CDC 虚拟串口
	inline constexpr bool ENABLE_IMU           = true;  // BMI088 六轴采样与姿态解算
	inline constexpr bool ENABLE_SAFETY        = true;  // 本地输出总闸门
	inline constexpr bool ENABLE_INTERBOARD    = false; // 板间业务本阶段关闭
	inline constexpr bool ENABLE_ENCODER_UART7 = true;  // 云台 pitch / 底盘四轮遥测
	inline constexpr bool ENABLE_BUZZER        = false; // 两板共享开关

	// ---------- 按角色装配 ----------
	inline constexpr bool ENABLE_RC      = (kBoardRole == BoardRole::Gimbal);  // DR16 仅云台板
	inline constexpr bool ENABLE_GIMBAL  = (kBoardRole == BoardRole::Gimbal);
	inline constexpr bool ENABLE_SHOOT   = (kBoardRole == BoardRole::Gimbal);
	inline constexpr bool ENABLE_CHASSIS = (kBoardRole == BoardRole::Chassis);

	static_assert(!ENABLE_GIMBAL || (ENABLE_RC && ENABLE_SAFETY),
				  "Gimbal requires remote control and output safety");
	static_assert(!ENABLE_SHOOT || (ENABLE_RC && ENABLE_SAFETY),
				  "Shooter requires remote control and output safety");
	static_assert(ENABLE_GIMBAL == (kBoardRole == BoardRole::Gimbal),
				  "Gimbal role mismatch");
	static_assert(ENABLE_SHOOT == (kBoardRole == BoardRole::Gimbal),
				  "Shooter role mismatch");
	static_assert(ENABLE_CHASSIS == (kBoardRole == BoardRole::Chassis),
				  "Chassis role mismatch");
	static_assert(ENABLE_RC == (kBoardRole == BoardRole::Gimbal),
				  "DR16 belongs to the gimbal board");
	static_assert(ENABLE_SAFETY, "Both boards keep a local output gate");
}

#endif
