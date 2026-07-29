#include "bmi088.h"

#include "main.h"

#include <string.h>

#define BMI088_CHIP_ID_REG            0x00U
#define BMI088_SPI_READ_BIT           0x80U
#define BMI088_SPI_TIMEOUT_MS         10U
#define BMI088_GYRO_STARTUP_DELAY_MS  30U
#define BMI088_SPI_SWITCH_DELAY_MS     1U
#define BMI088_REGISTER_WRITE_DELAY_MS 1U
#define BMI088_POWER_CONFIG_DELAY_MS   5U

#define BMI088_ACCEL_DATA_REG          0x12U
#define BMI088_ACCEL_TEMP_REG          0x22U
#define BMI088_ACCEL_CONF_REG          0x40U
#define BMI088_ACCEL_RANGE_REG         0x41U
#define BMI088_ACCEL_PWR_CONF_REG      0x7CU
#define BMI088_ACCEL_PWR_CTRL_REG      0x7DU

#define BMI088_GYRO_DATA_REG           0x02U
#define BMI088_GYRO_RANGE_REG          0x0FU
#define BMI088_GYRO_BANDWIDTH_REG      0x10U

#define BMI088_ACCEL_NORMAL_800HZ      0xABU
#define BMI088_ACCEL_RANGE_6G          0x01U
#define BMI088_ACCEL_ACTIVE_MODE       0x00U
#define BMI088_ACCEL_ENABLE            0x04U

#define BMI088_GYRO_RANGE_1000_DPS     0x01U
#define BMI088_GYRO_1000HZ_116HZ       0x02U

#define BMI088_ACCEL_BURST_LENGTH      8U
#define BMI088_GYRO_BURST_LENGTH       7U
#define BMI088_TEMP_READ_DIVIDER    1000U

static SPI_HandleTypeDef *bmi088_spi;
static BMI088_Diagnostic_t diagnostic;
static BMI088_Sample_t latest_sample;
static volatile uint32_t sample_sequence;
static int32_t latest_temperature_milli_c;
static bool temperature_valid;

static void bmi088_record_config_failure(
    BMI088_ConfigDevice_t device,
    uint8_t reg,
    uint8_t written_value,
    uint8_t readback_value)
{
  if (diagnostic.config_failed_device
      == BMI088_CONFIG_DEVICE_NONE)
  {
    diagnostic.config_failed_device = device;
    diagnostic.config_failed_register = reg;
    diagnostic.config_written_value = written_value;
    diagnostic.config_readback_value = readback_value;
  }
}

static HAL_StatusTypeDef bmi088_accel_read_register(
    uint8_t reg,
    uint8_t *value)
{
  uint8_t tx[3] = {
      (uint8_t)(reg | BMI088_SPI_READ_BIT),
      0U,
      0U
  };
  uint8_t rx[3] = {0U, 0U, 0U};
  HAL_StatusTypeDef status;

  if ((bmi088_spi == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(
      bmi088_spi,
      tx,
      rx,
      (uint16_t)sizeof(tx),
      BMI088_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    /*
     * BMI088加速度计读操作会先返回一个dummy byte，
     * 实际寄存器值位于地址字节之后的第二个接收字节。
     */
    *value = rx[2];
  }
  return status;
}

static HAL_StatusTypeDef bmi088_gyro_read_register(
    uint8_t reg,
    uint8_t *value)
{
  uint8_t tx[2] = {
      (uint8_t)(reg | BMI088_SPI_READ_BIT),
      0U
  };
  uint8_t rx[2] = {0U, 0U};
  HAL_StatusTypeDef status;

  if ((bmi088_spi == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(
      bmi088_spi,
      tx,
      rx,
      (uint16_t)sizeof(tx),
      BMI088_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    *value = rx[1];
  }
  return status;
}

static HAL_StatusTypeDef bmi088_accel_write_register(
    uint8_t reg,
    uint8_t value)
{
  uint8_t tx[2] = {
      (uint8_t)(reg & (uint8_t)~BMI088_SPI_READ_BIT),
      value
  };
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(
      bmi088_spi,
      tx,
      (uint16_t)sizeof(tx),
      BMI088_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_SET);
  HAL_Delay(BMI088_REGISTER_WRITE_DELAY_MS);
  return status;
}

static HAL_StatusTypeDef bmi088_gyro_write_register(
    uint8_t reg,
    uint8_t value)
{
  uint8_t tx[2] = {
      (uint8_t)(reg & (uint8_t)~BMI088_SPI_READ_BIT),
      value
  };
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(
      bmi088_spi,
      tx,
      (uint16_t)sizeof(tx),
      BMI088_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_SET);
  HAL_Delay(BMI088_REGISTER_WRITE_DELAY_MS);
  return status;
}

static bool bmi088_accel_write_and_verify(
    uint8_t reg,
    uint8_t value,
    uint8_t verify_mask)
{
  uint8_t readback = 0U;
  HAL_StatusTypeDef write_status;
  HAL_StatusTypeDef read_status = HAL_ERROR;

  write_status = bmi088_accel_write_register(reg, value);
  if (write_status == HAL_OK)
  {
    read_status = bmi088_accel_read_register(reg, &readback);
  }

  if ((write_status == HAL_OK)
      && (read_status == HAL_OK)
      && ((readback & verify_mask) == (value & verify_mask)))
  {
    return true;
  }

  bmi088_record_config_failure(
      BMI088_CONFIG_DEVICE_ACCEL,
      reg,
      value,
      readback);
  return false;
}

static bool bmi088_gyro_write_and_verify(
    uint8_t reg,
    uint8_t value,
    uint8_t verify_mask)
{
  uint8_t readback = 0U;
  HAL_StatusTypeDef write_status;
  HAL_StatusTypeDef read_status = HAL_ERROR;

  write_status = bmi088_gyro_write_register(reg, value);
  if (write_status == HAL_OK)
  {
    read_status = bmi088_gyro_read_register(reg, &readback);
  }

  if ((write_status == HAL_OK)
      && (read_status == HAL_OK)
      && ((readback & verify_mask) == (value & verify_mask)))
  {
    return true;
  }

  bmi088_record_config_failure(
      BMI088_CONFIG_DEVICE_GYRO,
      reg,
      value,
      readback);
  return false;
}

static HAL_StatusTypeDef bmi088_accel_read_burst(
    uint8_t start_reg,
    uint8_t *data,
    uint16_t length)
{
  uint8_t tx[BMI088_ACCEL_BURST_LENGTH] = {0U};
  uint8_t rx[BMI088_ACCEL_BURST_LENGTH] = {0U};
  HAL_StatusTypeDef status;

  if ((data == NULL)
      || (length > (BMI088_ACCEL_BURST_LENGTH - 2U)))
  {
    return HAL_ERROR;
  }

  tx[0] = (uint8_t)(start_reg | BMI088_SPI_READ_BIT);
  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(
      bmi088_spi,
      tx,
      rx,
      (uint16_t)(length + 2U),
      BMI088_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    memcpy(data, &rx[2], length);
  }
  return status;
}

static HAL_StatusTypeDef bmi088_gyro_read_burst(
    uint8_t start_reg,
    uint8_t *data,
    uint16_t length)
{
  uint8_t tx[BMI088_GYRO_BURST_LENGTH] = {0U};
  uint8_t rx[BMI088_GYRO_BURST_LENGTH] = {0U};
  HAL_StatusTypeDef status;

  if ((data == NULL)
      || (length > (BMI088_GYRO_BURST_LENGTH - 1U)))
  {
    return HAL_ERROR;
  }

  tx[0] = (uint8_t)(start_reg | BMI088_SPI_READ_BIT);
  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(
      bmi088_spi,
      tx,
      rx,
      (uint16_t)(length + 1U),
      BMI088_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    memcpy(data, &rx[1], length);
  }
  return status;
}

static int16_t bmi088_decode_le_i16(
    uint8_t lsb,
    uint8_t msb)
{
  return (int16_t)(
      (uint16_t)lsb | ((uint16_t)msb << 8U));
}

static HAL_StatusTypeDef bmi088_read_temperature(void)
{
  uint8_t data[2];
  uint16_t raw_unsigned;
  int16_t raw_signed;
  HAL_StatusTypeDef status;

  status = bmi088_accel_read_burst(
      BMI088_ACCEL_TEMP_REG,
      data,
      (uint16_t)sizeof(data));
  if (status != HAL_OK)
  {
    return status;
  }

  raw_unsigned =
      ((uint16_t)data[0] << 3U)
      | ((uint16_t)data[1] >> 5U);
  raw_signed = (raw_unsigned > 1023U)
      ? (int16_t)((int32_t)raw_unsigned - 2048)
      : (int16_t)raw_unsigned;

  /* 分辨率0.125°C/LSB，零点偏置23°C。 */
  latest_temperature_milli_c =
      23000 + ((int32_t)raw_signed * 125);
  temperature_valid = true;
  return HAL_OK;
}

BMI088_Status_t BMI088_Init(SPI_HandleTypeDef *spi)
{
  uint8_t dummy = 0U;
  HAL_StatusTypeDef accel_status;
  HAL_StatusTypeDef gyro_status;

  memset(&diagnostic, 0, sizeof(diagnostic));
  memset(&latest_sample, 0, sizeof(latest_sample));
  sample_sequence = 0U;
  latest_temperature_milli_c = 0;
  temperature_valid = false;
  bmi088_spi = spi;

  if ((bmi088_spi == NULL) || (bmi088_spi->Instance != SPI1))
  {
    diagnostic.status = BMI088_STATUS_SPI_ERROR;
    return diagnostic.status;
  }

  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_SET);
  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_SET);
  /*
   * 陀螺仪从上电到可通信最长需要30 ms。这里从进入初始化函数开始
   * 完整等待一次，不依赖MCU此前外设初始化已经消耗的时间。
   */
  HAL_Delay(BMI088_GYRO_STARTUP_DELAY_MS);

  /*
   * 加速度计上电默认进入I2C接口状态。第一次无效SPI读取提供
   * CS上升沿，使其切换到SPI模式；该次返回值按数据手册要求丢弃。
   */
  (void)bmi088_accel_read_register(BMI088_CHIP_ID_REG, &dummy);
  HAL_Delay(BMI088_SPI_SWITCH_DELAY_MS);

  accel_status = bmi088_accel_read_register(
      BMI088_CHIP_ID_REG,
      &diagnostic.accel_chip_id);
  gyro_status = bmi088_gyro_read_register(
      BMI088_CHIP_ID_REG,
      &diagnostic.gyro_chip_id);

  diagnostic.accel_transfer_ok = (accel_status == HAL_OK);
  diagnostic.gyro_transfer_ok = (gyro_status == HAL_OK);

  if (!diagnostic.accel_transfer_ok
      || !diagnostic.gyro_transfer_ok)
  {
    diagnostic.status = BMI088_STATUS_SPI_ERROR;
  }
  else if ((diagnostic.accel_chip_id
            != BMI088_ACCEL_EXPECTED_CHIP_ID)
           || (diagnostic.gyro_chip_id
               != BMI088_GYRO_EXPECTED_CHIP_ID))
  {
    diagnostic.status = BMI088_STATUS_ID_MISMATCH;
  }
  else
  {
    diagnostic.configuration_ok =
        bmi088_accel_write_and_verify(
            BMI088_ACCEL_PWR_CONF_REG,
            BMI088_ACCEL_ACTIVE_MODE,
            0xFFU);
    HAL_Delay(BMI088_POWER_CONFIG_DELAY_MS);
    diagnostic.configuration_ok =
        diagnostic.configuration_ok
        && bmi088_accel_write_and_verify(
            BMI088_ACCEL_PWR_CTRL_REG,
            BMI088_ACCEL_ENABLE,
            0xFFU);
    HAL_Delay(BMI088_POWER_CONFIG_DELAY_MS);
    diagnostic.configuration_ok =
        diagnostic.configuration_ok
        && bmi088_accel_write_and_verify(
            BMI088_ACCEL_CONF_REG,
            BMI088_ACCEL_NORMAL_800HZ,
            0xFFU)
        && bmi088_accel_write_and_verify(
            BMI088_ACCEL_RANGE_REG,
            BMI088_ACCEL_RANGE_6G,
            0x03U)
        && bmi088_gyro_write_and_verify(
            BMI088_GYRO_RANGE_REG,
            BMI088_GYRO_RANGE_1000_DPS,
            0x07U)
        && bmi088_gyro_write_and_verify(
            BMI088_GYRO_BANDWIDTH_REG,
            BMI088_GYRO_1000HZ_116HZ,
            0x07U);

    diagnostic.status = diagnostic.configuration_ok
        ? BMI088_STATUS_OK
        : BMI088_STATUS_CONFIG_ERROR;
  }

  return diagnostic.status;
}

bool BMI088_ReadSample(void)
{
  uint8_t accel_data[6];
  uint8_t gyro_data[6];
  BMI088_Sample_t sample;

  if (diagnostic.status != BMI088_STATUS_OK)
  {
    return false;
  }

  if ((bmi088_accel_read_burst(
           BMI088_ACCEL_DATA_REG,
           accel_data,
           (uint16_t)sizeof(accel_data)) != HAL_OK)
      || (bmi088_gyro_read_burst(
              BMI088_GYRO_DATA_REG,
              gyro_data,
              (uint16_t)sizeof(gyro_data)) != HAL_OK))
  {
    ++diagnostic.read_error_count;
    return false;
  }

  memset(&sample, 0, sizeof(sample));
  sample.accel_raw[0] =
      bmi088_decode_le_i16(accel_data[0], accel_data[1]);
  sample.accel_raw[1] =
      bmi088_decode_le_i16(accel_data[2], accel_data[3]);
  sample.accel_raw[2] =
      bmi088_decode_le_i16(accel_data[4], accel_data[5]);
  sample.gyro_raw[0] =
      bmi088_decode_le_i16(gyro_data[0], gyro_data[1]);
  sample.gyro_raw[1] =
      bmi088_decode_le_i16(gyro_data[2], gyro_data[3]);
  sample.gyro_raw[2] =
      bmi088_decode_le_i16(gyro_data[4], gyro_data[5]);

  sample.sample_count = latest_sample.sample_count + 1U;
  if (!temperature_valid
      || ((sample.sample_count % BMI088_TEMP_READ_DIVIDER) == 0U))
  {
    if (bmi088_read_temperature() != HAL_OK)
    {
      ++diagnostic.read_error_count;
    }
  }

  sample.temperature_milli_c =
      latest_temperature_milli_c;
  sample.timestamp_ms = HAL_GetTick();
  sample.valid = true;

  /*
   * 序列锁：写入时为奇数，完整发布后变为偶数。
   * USB监视任务可据此获得不撕裂的一致快照，无需互斥锁。
   */
  ++sample_sequence;
  __DMB();
  latest_sample = sample;
  __DMB();
  ++sample_sequence;
  return true;
}

bool BMI088_GetLatestSample(BMI088_Sample_t *sample)
{
  uint32_t begin_sequence;
  uint32_t end_sequence;
  uint32_t attempt;

  if (sample == NULL)
  {
    return false;
  }

  for (attempt = 0U; attempt < 3U; ++attempt)
  {
    begin_sequence = sample_sequence;
    if ((begin_sequence & 1U) != 0U)
    {
      continue;
    }

    __DMB();
    *sample = latest_sample;
    __DMB();
    end_sequence = sample_sequence;

    if ((begin_sequence == end_sequence)
        && ((end_sequence & 1U) == 0U))
    {
      return sample->valid;
    }
  }

  return false;
}

const BMI088_Diagnostic_t *BMI088_GetDiagnostic(void)
{
  return &diagnostic;
}
