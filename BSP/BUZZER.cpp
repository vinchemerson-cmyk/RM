//
// Created by 19923 on 2026/9/24.
//

#include "BUZZER.h"

#include "tim.h"

//蜂鸣器是否完成初始化
static bool buzzerInitialized = false;
//蜂鸣器是否完成使能
static bool buzzerEnabled = false;

HAL_StatusTypeDef Buzzer_Init()
{
    HAL_StatusTypeDef status;

    if (buzzerInitialized)
    {
        return HAL_OK;
    }

    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
    status = HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    if (status != HAL_OK)
    {
        buzzerInitialized = true;
        buzzerEnabled = true;
    }

    return status;



}

void Buzzer_On()
{
    if (!buzzerInitialized)
    {
        return;
    }

    if (buzzerDutyPercent == 0)
    {
        buzzerDutyPercent = BUZZER_DUTY_PERCENT;
    }


    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);

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
