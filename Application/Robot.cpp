/**
 * @file Robot.cpp
 * @brief 在调度器启动前装配整车已实现的模块。
 */

#include "Robot.h"
#include "RobotConfig.hpp"
#include "FreeRTOS.h"
#include "chassis/chassis.h"

static_assert(configTICK_RATE_HZ == 1000U, "Remote and motor timing requires 1ms RTOS ticks");

namespace
{
	// 应用栈按各任务 osThreadAttr_t 的字节数计；另留 8KB 给 TCB、内核任务和驱动对象。
	constexpr size_t kApplicationStackBytes = 4096U +
											  (InfantryInitialization::ENABLE_USB_SERIAL ? 2048U : 0U) +
											  (InfantryInitialization::ENABLE_IMU ? 3072U : 0U) +
											  (InfantryInitialization::ENABLE_RC ? 2048U : 0U) +
											  (InfantryInitialization::ENABLE_SAFETY ? 1024U : 0U) +
											  (InfantryInitialization::ENABLE_GIMBAL ? 2048U : 0U) +
											  (InfantryInitialization::ENABLE_SHOOT ? 3072U : 0U) +
											  (InfantryInitialization::ENABLE_INTERBOARD ? 2048U : 0U) +
											  (InfantryInitialization::ENABLE_ENCODER_UART7 ? 2048U : 0U) +
											  (InfantryInitialization::ENABLE_BUZZER ? 1024U : 0U)+
											  	(InfantryInitialization::ENABLE_CHASSIS ? 2048U : 0U) ;
	static_assert(configTOTAL_HEAP_SIZE >= kApplicationStackBytes + 8192U,
				  "RTOS heap is too small for the enabled tasks");
}

#include "main.h"
#include "BUZZER/buzzer_task.h"
#include "communication/interboard.h"
#include "communication/uart_serial_task.h"
#include "communication/encoder_serial_task.h"
#include "communication/usb_serial_task.h"
#include "gimbal/gimbal.h"
#include "imu/ImuTask.h"
#include "log/bsp_log.h"
#include "rc/rc.h"
#include "safety/safety.h"
#include "shoot/shoot.h"

/** @brief 创建已启用的任务并注册通信接口；重复调用只装配一次。 */
void InfantryInit(void)
{
	static bool initialized = false;
	if (initialized)
	{
		return;
	}
	BSPLogInit();

	if (InfantryInitialization::ENABLE_USB_SERIAL)
	{
		serialTasks::CreateUsbSerialTask();
	}
	if (InfantryInitialization::ENABLE_IMU)
	{
		imu::CreateTask();
	}
	// 双板链路与同路设备统一注册；各模块都须在调度器启动前完成装配，不独占或重启总线。
	if (InfantryInitialization::ENABLE_INTERBOARD && !interboard::Start())
	{
		Error_Handler();
	}

	if (InfantryInitialization::ENABLE_BUZZER)
		appTasks::CreateBuzzerTask();

	if (InfantryInitialization::ENABLE_RC)
		rc::Init();

	if (InfantryInitialization::ENABLE_SAFETY)
		safety::Init();

	// 云台任务运行时每周期查询遥控和急停状态。
	if (InfantryInitialization::ENABLE_GIMBAL)
		gimbal::Init();

	if (InfantryInitialization::ENABLE_CHASSIS)
	{
		chassis::Init();
	}

	if (InfantryInitialization::ENABLE_SHOOT)
	{
		shoot::Init();
	}

	if (InfantryInitialization::ENABLE_ENCODER_UART7)
	{
		//serialTasks::CreateEncoderSerialTask(gimbal::GetYawControlSnapshot);
		serialTasks::CreateEncoderSerialTask(gimbal::GetPitchControlSnapshot);
	}


	initialized = true;
}
