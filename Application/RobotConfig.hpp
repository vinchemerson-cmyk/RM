/**
 * @file RobotConfig.hpp
 * @brief 集中配置云台板和底盘板的编译选项及启动功能。
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
    inline constexpr bool ENABLE_USB_SERIAL     = true;  // USB CDC 虚拟串口回环任务
    inline constexpr bool ENABLE_INTERBOARD     = true;  // 双板 CAN 通信
    inline constexpr bool ENABLE_UART_SERIAL    = true;  // UART7 硬件串口回环任务
    inline constexpr bool ENABLE_BUZZER         = false; //蜂鸣器初始化
    inline constexpr bool ENABLE_LEDS           = true;  //LED初始化
    inline constexpr bool ENABLE_MOTORS_SHOOTER = true;  //摩擦轮初始化
    inline constexpr bool ENABLE_MOTORS_FEEDER  = true;  //拨弹盘初始化
    inline constexpr bool ENABLE_MOTORS_PTYW    = true;  //pitch和yaw的初始化
    inline constexpr bool ENABLE_CHASSIC        = true;  //底盘初始化
    inline constexpr bool ENABLE_ROMOTERS       = true;  //遥控初始化
}


#endif
