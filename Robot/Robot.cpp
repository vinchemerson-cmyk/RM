//
// Created by 19923 on 2026/9/24.
//

#include "Robot.h"
#include "RobotConfig.hpp"

namespace Infantry
{
    bool Initialized{};
}

/**
 * @brief 步兵初始模块
 */
void InfantryInit()
{
    //检查步兵是否初始化
    if (Infantry::Initialized)
    {
        return;
    }

    //检查蜂鸣器是否初始化
    if (InfantryConfig::ENABLE_BUZZER)
    {
        //todo
    }

    //检查LED是否初始化
    if (InfantryConfig::ENABLE_LEDS)
    {
        //todo
    }

    //检查摩擦轮是否初始化
    if (InfantryConfig::ENABLE_MOTORS_SHOOTER)
    {

    }
}