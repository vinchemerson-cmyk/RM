//
// Created by 19923 on 2026/9/24.
//

#include "BUZZER.h"

#include "tim.h"

//蜂鸣器是否完成初始化
static bool buzzerInitialized = false;
//蜂鸣器是否完成使能
static bool buzzerEnabled = false;

static uint8_t buzzerDutyPercent = BUZZER_DEFAULT_DUTY_PERCENT;

static uint32_t Buzzer_DutyCompare(const uint8_t dutyPercent)
{
    const uint32_t periodCount = __HAL_TIM_GET_AUTORELOAD(&htim12) + 1U;

    return (periodCount * dutyPercent) / 100U;
};

HAL_StatusTypeDef Buzzer_Init()
{
    //如果已经初始化,则HAL_OK
    if (buzzerInitialized == true)
    {
        return HAL_OK;
    }

    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    if (status == HAL_OK)
    {
        buzzerInitialized = true;
        buzzerEnabled = true;
    }

    return status;



}

void BuzzerOn()
{
    if (!buzzerInitialized)
    {
        return;
    }

    if (buzzerDutyPercent == 0)
    {
        buzzerDutyPercent = BUZZER_DEFAULT_DUTY_PERCENT;
    }


    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, Buzzer_DutyCompare(buzzerDutyPercent));

    buzzerEnabled = true;
}

void BuzzerOff()
{
    if (!buzzerInitialized)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
    buzzerEnabled = false;
}

void BuzzerSetDuty(uint8_t dutyPercent)
{
    if (dutyPercent > 100U)
    {
        dutyPercent = 100;
    }
       buzzerDutyPercent = dutyPercent ;

    if (dutyPercent==0U)
    {
        BuzzerOff();
    }

    else if (buzzerEnabled)
    {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, Buzzer_DutyCompare(dutyPercent));
    }
}

bool BuzzerIsOn()
{
    return buzzerEnabled;
}
