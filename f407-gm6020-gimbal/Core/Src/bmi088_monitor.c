#include "bmi088_monitor.h"

#include "bmi088.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define BMI088_RAW_MONITOR_PERIOD_MS       20U
#define BMI088_ERROR_MONITOR_PERIOD_MS   1000U
#define BMI088_MONITOR_BUFFER_CAPACITY    128U

static uint8_t monitor_buffer[BMI088_MONITOR_BUFFER_CAPACITY];
static uint16_t monitor_length;
static uint32_t last_monitor_tx_ms;
static bool monitor_pending;

void BMI088_Monitor_Init(void)
{
  monitor_length = 0U;
  last_monitor_tx_ms =
      HAL_GetTick() - BMI088_ERROR_MONITOR_PERIOD_MS;
  monitor_pending = false;
}

void BMI088_Monitor_Process(void)
{
  const uint32_t now = HAL_GetTick();
  const BMI088_Diagnostic_t *diagnostic;
  BMI088_Sample_t sample;
  const char *status_text;
  const char *config_device_text;
  uint32_t period_ms;
  int length;

  if (monitor_pending)
  {
    if (CDC_Transmit_FS(
            monitor_buffer,
            monitor_length) == USBD_OK)
    {
      monitor_pending = false;
      last_monitor_tx_ms = now;
    }
    return;
  }

  diagnostic = BMI088_GetDiagnostic();
  if (diagnostic == NULL)
  {
    return;
  }

  period_ms = (diagnostic->status == BMI088_STATUS_OK)
      ? BMI088_RAW_MONITOR_PERIOD_MS
      : BMI088_ERROR_MONITOR_PERIOD_MS;
  if ((uint32_t)(now - last_monitor_tx_ms) < period_ms)
  {
    return;
  }

  if ((diagnostic->status == BMI088_STATUS_OK)
      && BMI088_GetLatestSample(&sample))
  {
    length = snprintf(
        (char *)monitor_buffer,
        sizeof(monitor_buffer),
        "IMU_RAW,%lu,%d,%d,%d,%d,%d,%d,%ld,%lu\r\n",
        (unsigned long)sample.sample_count,
        (int)sample.accel_raw[0],
        (int)sample.accel_raw[1],
        (int)sample.accel_raw[2],
        (int)sample.gyro_raw[0],
        (int)sample.gyro_raw[1],
        (int)sample.gyro_raw[2],
        (long)sample.temperature_milli_c,
        (unsigned long)diagnostic->read_error_count);
  }
  else
  {
    switch (diagnostic->status)
    {
      case BMI088_STATUS_OK:
        status_text = "WAITING_SAMPLE";
        break;
      case BMI088_STATUS_SPI_ERROR:
        status_text = "SPI_ERROR";
        break;
      case BMI088_STATUS_ID_MISMATCH:
        status_text = "ID_ERROR";
        break;
      case BMI088_STATUS_CONFIG_ERROR:
        status_text = "CONFIG_ERROR";
        break;
      case BMI088_STATUS_UNINITIALIZED:
      default:
        status_text = "UNINITIALIZED";
        break;
    }

    if (diagnostic->status == BMI088_STATUS_CONFIG_ERROR)
    {
      config_device_text =
          (diagnostic->config_failed_device
           == BMI088_CONFIG_DEVICE_GYRO)
          ? "GYRO"
          : "ACC";
      length = snprintf(
          (char *)monitor_buffer,
          sizeof(monitor_buffer),
          "IMU,%s,ACC=0x%02X,GYRO=0x%02X,"
          "CFG=%s:0x%02X,W=0x%02X,R=0x%02X,ERR=%lu\r\n",
          status_text,
          (unsigned int)diagnostic->accel_chip_id,
          (unsigned int)diagnostic->gyro_chip_id,
          config_device_text,
          (unsigned int)diagnostic->config_failed_register,
          (unsigned int)diagnostic->config_written_value,
          (unsigned int)diagnostic->config_readback_value,
          (unsigned long)diagnostic->read_error_count);
    }
    else
    {
      length = snprintf(
          (char *)monitor_buffer,
          sizeof(monitor_buffer),
          "IMU,%s,ACC=0x%02X,GYRO=0x%02X,ERR=%lu\r\n",
          status_text,
          (unsigned int)diagnostic->accel_chip_id,
          (unsigned int)diagnostic->gyro_chip_id,
          (unsigned long)diagnostic->read_error_count);
    }
  }
  if ((length <= 0)
      || ((uint32_t)length >= sizeof(monitor_buffer)))
  {
    return;
  }

  monitor_length = (uint16_t)length;
  monitor_pending = true;
  if (CDC_Transmit_FS(
          monitor_buffer,
          monitor_length) == USBD_OK)
  {
    monitor_pending = false;
    last_monitor_tx_ms = now;
  }
}
