/**
 * @file buzzer_task.cpp
 * @brief 创建蜂鸣器 FreeRTOS 任务并周期控制鸣响。
 *
 * 结构: CreateBuzzerTask() 先初始化 BSP 蜂鸣器 (TIM12 CH2 PWM) 并
 * 设置占空比, 再创建 StartBuzzerTask 任务; 任务循环执行
 * "鸣响 BUZZER_ON_TIME_MS -> 静音 BUZZER_OFF_TIME_MS" 的周期提示音。
 * 后续可按整车上电解锁/故障状态扩展为提示音模式表。
 */

//
// Created by 19923 on 2026/9/24.
//
#include "buzzer_task.h"

#include "BUZZER/BUZZER.h"
#include "cmsis_os2.h"
#include "main.h"

namespace buzzer_tasks
{
	constexpr uint8_t  BUZZER_DUTY_PERCENT = 80U; //占空比
	constexpr uint32_t BUZZER_ON_TIME_MS   = 1000;
	constexpr uint32_t BUZZER_OFF_TIME_MS  = 1000;

	constexpr osThreadAttr_t BUZZER_TASK_ATTRS = {
		.name       = "BuzzerTask",
		.stack_size = 1024U,
		.priority   = osPriorityNormal,
	};


	osThreadId_t buzzerTaskHandle = nullptr;

	void Buzzer_Task(const uint32_t TimeMs, const uint32_t OffTimeMs)
	{
		BuzzerOn();
		osDelay(TimeMs);
		BuzzerOff();
		osDelay(OffTimeMs);
	}

	void StartBuzzerTask(void* argument)
	{
		(void)argument;

		for (;;)
		{
			Buzzer_Task(BUZZER_ON_TIME_MS, BUZZER_OFF_TIME_MS);
		}
	}
}

void appTasks::CreateBuzzerTask()
{
	if (buzzer_tasks::buzzerTaskHandle != nullptr)
	{
		return;
	}

	if (Buzzer_Init() != HAL_OK)
	{
		Error_Handler();
	}

	BuzzerSetDuty(buzzer_tasks::BUZZER_DUTY_PERCENT);

	buzzer_tasks::buzzerTaskHandle = osThreadNew(buzzer_tasks::StartBuzzerTask, nullptr,
												 &buzzer_tasks::BUZZER_TASK_ATTRS);

	if (buzzer_tasks::buzzerTaskHandle == nullptr)
	{
		Error_Handler();
	}
}
