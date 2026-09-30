/**
 * @file Robot.cpp
 * @brief 在调度器启动前装配整车已实现的模块。
 */

#include "Robot.h"
#include "RobotConfig.hpp"

#include "main.h"
#include "BUZZER/buzzer_task.h"
#include "communication/interboard.h"
#include "communication/uart_serial_task.h"
#include "communication/usb_serial_task.h"
#include "gimbal/gimbal.h"
#include "imu/ImuTask.h"
#include "log/bsp_log.h"
#include "rc/rc.h"
#include "safety/safety.h"

/** @brief 创建已启用的任务并注册通信接口；重复调用只装配一次。 */
void InfantryInit(void)
{
    static bool initialized = false;
    if (initialized)
        return;

    BSPLogInit();

    if (InfantryInitialization::ENABLE_USB_SERIAL)
        serialTasks::CreateUsbSerialTask();

    if (InfantryInitialization::ENABLE_IMU)
        imu::CreateTask();

    // 双板链路独占指定 FDCAN，须先于同一路 CAN 上的其他模块注册。
    if (InfantryInitialization::ENABLE_INTERBOARD && !interboard::Start())
        Error_Handler();

    if (InfantryInitialization::ENABLE_UART_SERIAL)
        serialTasks::CreateUartSerialTask();

    if (InfantryInitialization::ENABLE_BUZZER)
        appTasks::CreateBuzzerTask();

    if (InfantryInitialization::ENABLE_RC)
        rc::Init();

    if (InfantryInitialization::ENABLE_SAFETY)
        safety::Init();

    // 云台任务运行时每周期查询遥控和急停状态。
    if (InfantryInitialization::ENABLE_GIMBAL)
        gimbal::Init();

    initialized = true;
}
