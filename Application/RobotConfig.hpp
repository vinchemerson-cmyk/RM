//
// Created by 19923 on 2026/9/24.
//
#pragma once
#ifndef ROBOTINIT_HPP
#define ROBOTINIT_HPP

namespace InfantryInitialization
{
    inline constexpr bool ENABLE_BUZZER         = false; //蜂鸣器初始化
    inline constexpr bool ENABLE_LEDS           = true;  //LED初始化
    inline constexpr bool ENABLE_MOTORS_SHOOTER = true;  //摩擦轮初始化
    inline constexpr bool ENABLE_MOTORS_FEEDER  = true;  //拨弹盘初始化
    inline constexpr bool ENABLE_MOTORS_PTYW    = true;  //pitch和yaw的初始化
    inline constexpr bool ENABLE_CHASSIC        = true;  //底盘初始化
    inline constexpr bool ENABLE_ROMOTERS       = true;  //遥控初始化
}


#endif
