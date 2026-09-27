/**
 * @file Robot.cpp
 * @brief 按板卡配置初始化日志、通信和应用任务。
 */

//
// Created by 19923 on 2026/9/24.
//

#include "Robot.h"
#include "RobotConfig.hpp"
#include "BUZZER/buzzer_task.h"
#include "communication/usb_serial_task.h"
#include "communication/uart_serial_task.h"
#include "communication/interboard.h"
#include "main.h"
#include "log/bsp_log.h"

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

    BSPLogInit();

    if (InfantryInitialization::ENABLE_USB_SERIAL)
        serialTasks::CreateUsbSerialTask();
    if (InfantryInitialization::ENABLE_UART_SERIAL)
        serialTasks::CreateUartSerialTask();
    if (InfantryInitialization::ENABLE_INTERBOARD && !interboard::Start())
        Error_Handler();

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
