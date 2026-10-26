/**
 * @file RobotConfig.hpp
 * @brief 整车编译期配置: 板卡角色、CAN 资源分配和各功能模块的启停开关。
 *
 * 两块板 (云台板/底盘板) 共用本工程, 烧录前必须修改 INFANTRY_BOARD_GIMBAL:
 *   - 取 1: 云台板, 双板链路发送 0x301 / 接收 0x302。
 *   - 取 0: 底盘板, 双板链路发送 0x302 / 接收 0x301。
 *   两块板必须使用相反的值, 否则双板链路收不到对方心跳, 永远显示离线。
 *
 * FDCAN 资源分配 (STM32H723 共 3 路 FDCAN):
 *   FDCAN1 : pitch GM6020速度环调试、yaw连续位置控制及两轴编码器反馈。
 *   FDCAN2 : 两台 M3508 摩擦轮，拨弹当前不注册；双板通信关闭。
 *   FDCAN3 : 当前空闲；若迁移 DJI 电机到此路，须先将其设为 Classic CAN。
 * 调整 INFANTRY_INTERBOARD_FDCAN 或电机挂载时, 必须同步核对
 * interboard.cpp 的 canHandle 选择与各电机配置中的 kXxxCanHandle。
 */

//
// Created by 19923 on 2026/9/24.
//
#pragma once
#ifndef ROBOTINIT_HPP
#define ROBOTINIT_HPP

// 烧录云台板设为 1，烧录底盘板设为 0；两块板必须使用相反的值。
// TODO[CANFD-01 双板总线表]：按此角色分别列出FDCAN1/2/3的用途、经典/FD模式、波特率和收发容量。
// 将总线映射同时用于外设初始化及DM/板间等模块配置，不能只用该宏切换板间收发ID。
#define INFANTRY_BOARD_GIMBAL 1

// 双板链路独占的 FDCAN 控制器。当前默认 PB5/PB6 (FDCAN2)。
// TODO[CANFD-05 非独占链路]：DM+板间共线后，此项表示链路所在的共享FD总线，不再代表独占控制器。
// DM与板间应使用同后端的普通ID注册，共享过滤器分配；不要仅修改这里的数字就直接开启两模块。
#define INFANTRY_INTERBOARD_FDCAN 2

namespace InfantryInitialization
{
	// ---------- 通信与调试 ----------
	inline constexpr bool ENABLE_USB_SERIAL    = true;  // USB CDC 虚拟串口发送 Roll/Pitch/Yaw
	inline constexpr bool ENABLE_IMU           = true;  // BMI088 六轴采样与姿态解算
	inline constexpr bool ENABLE_INTERBOARD    = false; // 双板 CAN 通信 (interboard::Start, 独占上方选定的 FDCAN)
	// TODO[CANFD-09 启用前验证]：先完成上述硬件/缓冲区/打包/分发/协议联动，再开启共线板间任务。
	// 分别验证上下板配置：DM 8字节FD控制/反馈、板间64字节、混合高频收发、队列满/断链、过滤器不覆盖。
	// CAN注册统一在调度器启动前完成；顺序变化不能造成HAL重复启动或全局过滤器被覆盖。
	inline constexpr bool ENABLE_ENCODER_UART7 = true;  // UART7 TX-only pitch目标/实际位置与电流FireWater三通道，每50ms一行。

	// ---------- 输入与安全 ----------
	inline constexpr bool ENABLE_RC     = true; // 遥控 (UART5 DBUS 解析与掉线判定)
	inline constexpr bool ENABLE_SAFETY = true; // 急停与使能监控 (依赖遥控)

	// ---------- 执行器与提示 ----------
	inline constexpr bool ENABLE_GIMBAL = true;  // pitch速度环调试、yaw连续位置控制及编码器采集。
	inline constexpr bool ENABLE_SHOOT  = true;  // 当前只启用双摩擦轮，见 ShootConfig.hpp
	inline constexpr bool ENABLE_BUZZER = false; // TIM12 CH2 蜂鸣器任务
	inline constexpr bool ENABLE_CHASSIS = true; //底盘任务

	static_assert(!ENABLE_GIMBAL || (ENABLE_RC && ENABLE_SAFETY),
				  "Gimbal requires remote control and output safety");
	static_assert(!ENABLE_SHOOT || (ENABLE_RC && ENABLE_SAFETY),
				  "Shooter requires remote control and output safety");
	static_assert(!ENABLE_INTERBOARD || !ENABLE_SHOOT || INFANTRY_INTERBOARD_FDCAN != 2,
				  "FDCAN2 is occupied by friction wheels; interboard cannot share it");
}

#endif
