/** @file encoder_serial_task.cpp @brief UART7 DMA 每 50ms 输出 pitch 目标/实际位置和反馈电流。 */
#include "encoder_serial_task.h"
#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"
#include "usart/usart.hpp"

namespace
{
osThreadId_t taskHandle = nullptr;
serialTasks::EncoderProvider feedbackProvider = nullptr;
constexpr osThreadAttr_t taskAttrs{
    .name = "EncoderSerialTask", .stack_size = 2048U, .priority = osPriorityNormal,
};
constexpr uint32_t kPrintPeriodMs = 50U;
static_assert(usartConfig::TX_BUF_SIZE >= 64U, "UART pitch telemetry line requires a 64-byte buffer");

/** @brief 在任务中注册 UART7；BSP 将临时文本复制至 AXI SRAM 长期 DMA 缓冲。 */
void Task(void*)
{
    const usartConfig::Config config{
        .usart_handle = &huart7,
        .tx_mode = usartConfig::TRANSFER_DMA,
        .rx_data_len = 0U,
        .usart_callback_func = nullptr,
        .module_instance = &taskHandle,
    };
    auto* uart = usart::Register(config);
    for (;;)
    {
        const auto snapshot = feedbackProvider();
        usartConfig::Input tx{};
        tx.tx_len = static_cast<uint32_t>(serialTasks::FormatPitchTelemetryLine(
            reinterpret_cast<char*>(tx.tx_buffer.data()), tx.tx_buffer.size(),
            snapshot, osKernelGetTickCount()));
        if (tx.tx_len != 0U)
            (void)uart->Transmit(tx); // BUSY/ERROR 不阻塞，下一周期发送最新快照。
        osDelay(kPrintPeriodMs);
    }
}
}

void serialTasks::CreateEncoderSerialTask(EncoderProvider provider)
{
    if (taskHandle != nullptr) return;
    if (provider == nullptr) Error_Handler();
    feedbackProvider = provider;
    taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
    if (taskHandle == nullptr) Error_Handler();
}
