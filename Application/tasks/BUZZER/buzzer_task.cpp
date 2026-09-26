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
