//
// Created by 19923 on 2026/9/24.
//
#include "buzzer_task.h"
#include "cmsis_os2.h"
#include "main.h"
#include "Robot.h"

namespace
{
    osThreadId_t buzzerTaskHandle = nullptr;

    void Buzzer_Task(const uint32_t TimeMs,const uint32_t OffTimeMs)
    {
        BuzzerOn();
        osDelay(TimeMs);
        BuzzerOff();
        osDelay(OffTimeMs);


    }
void StartBuzzerTask(void const *argument)
{
    (void)argument;;

    for (;;)
    {
        Buzzer_Task();
    }
}
}

void CreateBuzzerTask()
    {
        if (buzzerTaskHandle != nullptr)
        {
            return;
        }

        if (Buzzer_Init() != nullptr)
        {
            Error_Handler();
        }
        Buzzer_SetDuty(Buzzer_Duty);

        buzzerTaskHandle = osThreadNew(StartBuzzerTask, nullptr, &BUZZER_TASK_ATTRS);
    if (buzzerTaskHandle == nullptr)
    {
        Error_Handler();
    }

    }