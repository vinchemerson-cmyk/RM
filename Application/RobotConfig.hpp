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
 *   FDCAN1 : 云台电机 (两台 GM6020, 见 GimbalConfig.hpp)。
 *   FDCAN2 : 双板通信 interboard 独占 (由下方 INFANTRY_INTERBOARD_FDCAN 选择)。
 *   FDCAN3 : 当前空闲, 可分配给底盘/发射电机或裁判系统。
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
#define INFANTRY_BOARD_GIMBAL 1
// 双板链路独占的 FDCAN 控制器。当前默认 PB5/PB6 (FDCAN2)。
#define INFANTRY_INTERBOARD_FDCAN 2

namespace InfantryInitialization
{
    // ---------- 通信与调试 ----------
    inline constexpr bool ENABLE_USB_SERIAL     = true;  // USB CDC 虚拟串口发送 Roll/Pitch/Yaw
    inline constexpr bool ENABLE_IMU            = true;  // BMI088 六轴采样与姿态解算
    inline constexpr bool ENABLE_INTERBOARD     = false;  // 双板 CAN 通信 (interboard::Start, 独占上方选定的 FDCAN)
    inline constexpr bool ENABLE_UART_SERIAL    = false;  // UART7 硬件串口回环任务 (DMA 收 -> 队列 -> DMA 发)

    // ---------- 输入与安全 ----------
    inline constexpr bool ENABLE_RC            = false; // 遥控 (UART5 DBUS 解析与掉线判定)
    inline constexpr bool ENABLE_SAFETY        = false; // 急停与使能监控 (依赖遥控)

    // ---------- 执行器与提示 ----------
    inline constexpr bool ENABLE_GIMBAL        = false; // yaw/pitch 云台 (两台 GM6020)
    inline constexpr bool ENABLE_BUZZER        = false; // TIM12 CH2 蜂鸣器任务
}

#endif
