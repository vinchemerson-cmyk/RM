//
// Created by 19923 on 2026/9/24.
//

#ifndef INFANTRY_H723_BUZZER_H
#define INFANTRY_H723_BUZZER_H

#include "stm32h7xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif

#define BUZZER_DEFAULT_DUTY_PERCENT 20U

/**
 *
 * @return
 */
HAL_StatusTypeDef Buzzer_Init();

void Buzzer_Task();

void BuzzerOn();

void BuzzerOff();

void BuzzerSetDuty(uint8_t dutyPercent);

bool BuzzerIsOn(void);

#ifdef __cplusplus
}
#endif

#endif //INFANTRY_H723_BUZZER_H
