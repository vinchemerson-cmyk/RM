/**
 * @file BUZZER.h
 * @brief 声明蜂鸣器底层控制接口及默认占空比。
 */

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
 * @brief 启动 TIM12 通道 2 的 PWM 输出；重复调用直接返回成功。
 * @return HAL 状态码，HAL_OK 表示 PWM 已启动。
 */
HAL_StatusTypeDef Buzzer_Init();

/** @brief 预留的蜂鸣器任务入口；当前文件没有实现。 */
void Buzzer_Task();

/** @brief 按当前保存的占空比打开蜂鸣器。 */
void BuzzerOn();

/** @brief 将 PWM 比较值清零并关闭蜂鸣器。 */
void BuzzerOff();

/**
 * @brief 保存新的占空比；已打开时立即更新 PWM，零值会关闭蜂鸣器。
 * @param dutyPercent 占空比百分数；大于 100 时按 100 处理。
 */
void BuzzerSetDuty(uint8_t dutyPercent);

/** @brief 查询软件记录的蜂鸣器开关状态。 @return true 表示处于打开状态。 */
bool BuzzerIsOn(void);

#ifdef __cplusplus
}
#endif

#endif //INFANTRY_H723_BUZZER_H
