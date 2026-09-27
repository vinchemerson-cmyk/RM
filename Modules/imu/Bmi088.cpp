/**
 * @file Bmi088.cpp
 * @brief 通过 SPI2 DMA 读取 BMI088 芯片标识并处理传输回调。
 */

//
// Created by 19923 on 2026/9/26.
//
//todo
#include "Bmi088.h"

#include "Bmi088_Reg.h"
#include "cmsis_os2.h"
#include "main.h"
#include "spi.h"

#include <cstring>

enum class BmiChip { Accel, Gyro };
static void Select(BmiChip chip, GPIO_PinState state);

namespace {
// DMA1/2 不能访问默认 DTCM 栈，因此收发缓冲区必须放在 AXI SRAM。
struct SpiDmaBuffers {
    alignas(32) uint8_t tx[32];
    alignas(32) uint8_t rx[32];
};

__attribute__((section(".axi_bss"), aligned(32))) SpiDmaBuffers spiDmaBuffers{};
osSemaphoreId_t transferDone = nullptr;
osMutexId_t spiMutex = nullptr;
volatile HAL_StatusTypeDef transferResult = HAL_ERROR;
constexpr uint32_t kTransferTimeoutTicks = 100;
bool identified = false;
bool temperatureEnabled = false;

/**
 * @brief 在互斥锁保护下完成一次 SPI2 DMA 寄存器访问。
 * @param dummy 加速度计读取时需要丢弃的额外空字节数。
 * @return HAL 状态码；超时后会终止本次 SPI DMA 传输。
 */
HAL_StatusTypeDef Transfer(BmiChip chip, uint8_t reg, uint8_t value,
                           uint16_t size, uint8_t *readData, uint8_t readCount,
                           uint8_t dummy)
{
    if (transferDone == nullptr || spiMutex == nullptr ||
        osMutexAcquire(spiMutex, kTransferTimeoutTicks) != osOK)
        return HAL_ERROR;

    // 清理上次超时留下的完成信号。
    while (osSemaphoreAcquire(transferDone, 0) == osOK) {}
    transferResult = HAL_ERROR;
    std::memset(spiDmaBuffers.tx, 0, size);
    spiDmaBuffers.tx[0] = readData != nullptr ? static_cast<uint8_t>(reg | 0x80)
                                               : static_cast<uint8_t>(reg & 0x7F);
    if (readData == nullptr)
        spiDmaBuffers.tx[1] = value;
    Select(chip, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = readData != nullptr
        ? HAL_SPI_TransmitReceive_DMA(&hspi2, spiDmaBuffers.tx, spiDmaBuffers.rx, size)
        : HAL_SPI_Transmit_DMA(&hspi2, spiDmaBuffers.tx, size);
    if (status == HAL_OK) {
        if (osSemaphoreAcquire(transferDone, kTransferTimeoutTicks) == osOK)
            status = transferResult;
        else {
            (void)HAL_SPI_Abort(&hspi2);
            status = HAL_TIMEOUT;
        }
    }

    Select(chip, GPIO_PIN_SET);
    if (status == HAL_OK && readData != nullptr)
        std::memcpy(readData, spiDmaBuffers.rx + 1 + dummy, readCount);
    (void)osMutexRelease(spiMutex);
    return status;
}
} // namespace

// 当前为未接入的任务预留句柄，暂未参与 BMI088 读取流程。
osThreadId_t buzzerTaskHandle = nullptr;

/** @brief 预留的 BMI088 周期任务，目前无采样逻辑。 */
void Bmi088_Task()
{

};

/** @brief 控制加速度计或陀螺仪各自的片选引脚。 */
static void Select(BmiChip chip, GPIO_PinState state)
{
    if (chip == BmiChip::Accel)
        HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, state);
    else
        HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, state);
}

/** @brief 读取最多 6 字节寄存器，并处理加速度计的空字节。 */
static HAL_StatusTypeDef ReadRegs(
    BmiChip chip, uint8_t reg, uint8_t* data, uint8_t count)
{
    if (data == nullptr || count == 0 || count > 6)
        return HAL_ERROR;

    const uint8_t dummy = (chip == BmiChip::Accel) ? 1 : 0;
    const uint16_t size = 1 + dummy + count;
    return Transfer(chip, reg, 0, size, data, count, dummy);
}

/** @brief 写入单个寄存器；当前初始化流程尚未调用。 */
static HAL_StatusTypeDef WriteReg(
    BmiChip chip, uint8_t reg, uint8_t value)
{
    return Transfer(chip, reg, value, 2, nullptr, 0, 0);
}

/** @brief 在 RTOS 启动后创建同步对象并验证两个传感器的芯片 ID。 */
HAL_StatusTypeDef BMI088_Init()
{
    // DMA 完成由中断通知，初始化必须在 FreeRTOS 任务中调用。
    if (osKernelGetState() != osKernelRunning)
        return HAL_ERROR;
    if (identified)
        return HAL_OK;
    if (transferDone == nullptr)
        transferDone = osSemaphoreNew(1, 0, nullptr);
    if (spiMutex == nullptr)
        spiMutex = osMutexNew(nullptr);
    if (transferDone == nullptr || spiMutex == nullptr)
        return HAL_ERROR;

    uint8_t accId = 0;
    uint8_t gyroId = 0;

    // 第一次加速度计读取用于切换 SPI 模式，结果丢弃
    if (ReadRegs(BmiChip::Accel, BMI088_ACCEL_CHIP_ID, &accId, 1) != HAL_OK)
        return HAL_ERROR;

    osDelay(1);

    if (ReadRegs(BmiChip::Accel, BMI088_ACCEL_CHIP_ID, &accId, 1) != HAL_OK ||
        ReadRegs(BmiChip::Gyro, BMI088_GYRO_CHIP_ID, &gyroId, 1) != HAL_OK)
        return HAL_ERROR;

    if (accId != BMI088_ACCEL_CHIP_ID_VALUE ||
        gyroId != BMI088_GYRO_CHIP_ID_VALUE)
        return HAL_ERROR;

    identified = true;
    return HAL_OK;
}

HAL_StatusTypeDef BMI088_EnableTemperature()
{
    if (!identified || osKernelGetState() != osKernelRunning)
        return HAL_ERROR;
    if (temperatureEnabled)
        return HAL_OK;

    // Bosch 的正常模式切换顺序：先配置电源模式，再使能加速度计和温度传感器。
    if (WriteReg(BmiChip::Accel, BMI088_ACCEL_PWR_CONF, BMI088_ACCEL_PWR_ACTIVE_MODE) != HAL_OK)
        return HAL_ERROR;
    osDelay(5);
    if (WriteReg(BmiChip::Accel, BMI088_ACCEL_PWR_CTRL, BMI088_ACCEL_ENABLE_ACCEL_ON) != HAL_OK)
        return HAL_ERROR;
    osDelay(5);
    temperatureEnabled = true;
    return HAL_OK;
}

HAL_StatusTypeDef BMI088_ReadTemperature(float* temperatureCelsius)
{
    if (!temperatureEnabled || temperatureCelsius == nullptr)
        return HAL_ERROR;

    uint8_t data[2]{};
    if (ReadRegs(BmiChip::Accel, BMI088_TEMP_M, data, 2) != HAL_OK || data[0] == 0x80U)
        return HAL_ERROR;

    int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 3U) |
                                       (data[1] >> 5U));
    if (raw > 1023)
        raw = static_cast<int16_t>(raw - 2048);
    *temperatureCelsius = static_cast<float>(raw) * 0.125F + 23.0F;
    return HAL_OK;
}

/** @brief SPI 收发 DMA 完成时唤醒等待中的任务。 */
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi2 && transferDone != nullptr) {
        transferResult = HAL_OK;
        (void)osSemaphoreRelease(transferDone);
    }
}

/** @brief SPI 发送 DMA 完成时唤醒等待中的任务。 */
extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi2 && transferDone != nullptr) {
        transferResult = HAL_OK;
        (void)osSemaphoreRelease(transferDone);
    }
}

/** @brief SPI 出错时将失败状态传给等待中的任务。 */
extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi2 && transferDone != nullptr) {
        transferResult = HAL_ERROR;
        (void)osSemaphoreRelease(transferDone);
    }
}
