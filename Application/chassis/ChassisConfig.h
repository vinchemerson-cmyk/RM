//
// Created by 19923 on 2026/10/24.
//

#ifndef INFANTRY_H723_CHASSISCONFIG_H
#define INFANTRY_H723_CHASSISCONFIG_H
#include "fdcan.h"
#include "stm32h7xx_hal_fdcan.h"

inline FDCAN_HandleTypeDef* const kChassisCanHandle = &hfdcan1;

#endif //INFANTRY_H723_CHASSISCONFIG_H
