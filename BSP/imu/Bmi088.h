/**
 * @file Bmi088.h
 * @brief 声明 BMI088 初始化、量程配置和六轴采样接口。
 */

//
// Created by 19923 on 2026/9/26.
//

#ifndef INFANTRY_H723_BMI088_H
#define INFANTRY_H723_BMI088_H

#include "main.h"
#include <array>

/** @brief BMI088 机体坐标系中的加速度和角速度。 */
struct Bmi088MotionSample
{
	std::array<float, 3> acceleration_mps2{};
	std::array<float, 3> angular_velocity_rad_s{};
};

/**
 * @brief 通过 SPI2 DMA 读取 BMI088 芯片 ID，必须从 FreeRTOS 任务调用。
 * @return HAL_OK 表示两个芯片 ID 均匹配。
 */
HAL_StatusTypeDef BMI088_Init();

/**
 * @brief 检查重复配置，初始化并使能加热器，切换陀螺仪到正常工作模式，配置加速度计和陀螺仪。
 * @note 加速度计 ±3 g/800 Hz，陀螺仪 ±2000 °/s/1000 Hz。
 */
HAL_StatusTypeDef BMI088_ConfigureMotion();

/** @brief 连续读取加速度和角速度三轴；必须先完成 ConfigureMotion。 */
HAL_StatusTypeDef BMI088_ReadMotion(Bmi088MotionSample* sample);

/** @brief 将加速度计切换到正常模式，以启用片内温度传感器。 */
HAL_StatusTypeDef BMI088_EnableTemperature();

/**
 * @brief 读取加速度计片内温度。
 * @param temperatureCelsius 成功时写入摄氏温度。
 * @return HAL_OK 表示读取有效；未启用、SPI 出错或读到无效编码时返回 HAL_ERROR。
 * @note 温度寄存器约每 1.28 秒更新一次。
 */
HAL_StatusTypeDef BMI088_ReadTemperature(float* temperatureCelsius);

#endif //INFANTRY_H723_BMI088_H
