#ifndef BMI088_H
#define BMI088_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMI088_ACCEL_EXPECTED_CHIP_ID  0x1EU
#define BMI088_GYRO_EXPECTED_CHIP_ID   0x0FU

typedef enum
{
  BMI088_STATUS_UNINITIALIZED = 0,
  BMI088_STATUS_OK,
  BMI088_STATUS_SPI_ERROR,
  BMI088_STATUS_ID_MISMATCH,
  BMI088_STATUS_CONFIG_ERROR
} BMI088_Status_t;

typedef enum
{
  BMI088_CONFIG_DEVICE_NONE = 0,
  BMI088_CONFIG_DEVICE_ACCEL,
  BMI088_CONFIG_DEVICE_GYRO
} BMI088_ConfigDevice_t;

typedef struct
{
  BMI088_Status_t status;
  uint8_t accel_chip_id;
  uint8_t gyro_chip_id;
  bool accel_transfer_ok;
  bool gyro_transfer_ok;
  bool configuration_ok;
  BMI088_ConfigDevice_t config_failed_device;
  uint8_t config_failed_register;
  uint8_t config_written_value;
  uint8_t config_readback_value;
  uint32_t read_error_count;
} BMI088_Diagnostic_t;

typedef struct
{
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  int32_t temperature_milli_c;
  uint32_t sample_count;
  uint32_t timestamp_ms;
  bool valid;
} BMI088_Sample_t;

/*
 * 初始化SPI通信、校验CHIP_ID并配置：
 *   Accel: +/-6 g, 800 Hz, normal bandwidth
 *   Gyro : +/-1000 dps, 1000 Hz ODR / 116 Hz bandwidth
 */
BMI088_Status_t BMI088_Init(SPI_HandleTypeDef *spi);

/* 由唯一的IMU采集任务每1 ms调用一次。 */
bool BMI088_ReadSample(void);

/* 获取一致的最新数据快照；尚无有效样本时返回false。 */
bool BMI088_GetLatestSample(BMI088_Sample_t *sample);

const BMI088_Diagnostic_t *BMI088_GetDiagnostic(void);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_H */
