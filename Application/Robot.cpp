//
// Created by 19923 on 2026/9/24.
//

#include "Robot.h"
#include "RobotConfig.hpp"
#include "tasks/BUZZER/buzzer_task.h"

namespace Infantry
{
    bool Initialized{};
}

/**
 * @brief 步兵初始模块
 */
void InfantryInit(void)
{
    //检查步兵是否初始化
    if (Infantry::Initialized)
    {
        return;
    }

    //检查蜂鸣器是否初始化
    if (InfantryInitialization::ENABLE_BUZZER)
    {
        appTasks::CreateBuzzerTask();
        //todo
    }

    //检查LED是否初始化
     if (InfantryInitialization::ENABLE_LEDS)
     {
         //appTasks::CreateLedsTask();
         //todo
     }

    //检查摩擦轮是否初始化
    if (InfantryInitialization::ENABLE_MOTORS_SHOOTER)
    {
        //todo
    }

    //检查云台PITCH和YAW是否初始化
    if (InfantryInitialization::ENABLE_MOTORS_PTYW)
    {
        //todo
    }

    //检查底盘是否初始化
    if (InfantryInitialization::ENABLE_CHASSIC)
    {
        //todo
    }

    //检查遥控是否初始化
    if (InfantryInitialization::ENABLE_ROMOTERS)
    {
        //todo
    }

    Infantry::Initialized = true;
}
