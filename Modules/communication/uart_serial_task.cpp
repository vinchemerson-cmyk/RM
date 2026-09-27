/**
 * @file uart_serial_task.cpp
 * @brief 使用 UART7 接收队列和任务实现串口回环。
 */

#include "uart_serial_task.h"

#include <cstring>

#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"
#include "usart/usart.hpp"

namespace {
constexpr uint32_t kPacketSize = 64;
constexpr uint32_t kQueueDepth = 8;

/** @brief 从接收回调复制到任务队列的数据包。 */
struct Packet {
    uint16_t length;
    uint8_t data[kPacketSize];
};

// 更换硬件串口时修改此处，并核对 Core/Src/usart.c 中的波特率和引脚。
UART_HandleTypeDef *const kUartHandle = &huart7;

osMessageQueueId_t rxQueue = nullptr;
osThreadId_t taskHandle = nullptr;
usart *uart = nullptr;

constexpr osThreadAttr_t taskAttrs = {
    .name = "UartSerialTask",
    .stack_size = 2048U,
    .priority = osPriorityNormal,
};

/** @brief 将 UART DMA 本次收到的数据分包送入队列，供任务异步发送。 */
void OnReceive(usart *instance)
{
    if (rxQueue == nullptr || instance == nullptr)
        return;

    const auto &rx = instance->Receive();
    uint32_t remaining = rx.rx_size;
    if (remaining > rx.rx_buffer.size())
        remaining = rx.rx_buffer.size();
    const uint8_t *data = rx.rx_buffer.data();

    while (remaining != 0) {
        Packet packet{};
        packet.length = static_cast<uint16_t>(remaining > kPacketSize ? kPacketSize : remaining);
        std::memcpy(packet.data, data, packet.length);
        if (osMessageQueuePut(rxQueue, &packet, 0, 0) != osOK)
            return; // 满队列时不在中断中等待。
        data += packet.length;
        remaining -= packet.length;
    }
}

/** @brief 注册 UART7 并依次回传接收队列中的数据包。 */
void Task(void *argument)
{
    (void)argument;
    const usartConfig::Config config = {
        .usart_handle = kUartHandle,
        .tx_mode = usartConfig::TRANSFER_DMA,
        .rx_data_len = usartConfig::RX_BUF_SIZE,
        .usart_callback_func = OnReceive,
        .module_instance = &taskHandle,
    };
    uart = usart::Register(config);

    Packet packet{};
    for (;;) {
        if (osMessageQueueGet(rxQueue, &packet, nullptr, osWaitForever) != osOK)
            continue;

        usartConfig::Input tx{};
        tx.tx_len = packet.length;
        std::memcpy(tx.tx_buffer.data(), packet.data, packet.length);
        // BSP 会将数据复制到 AXI SRAM 的长期缓冲区，DMA 完成前不覆盖它。
        for (;;) {
            const HAL_StatusTypeDef status = uart->Transmit(tx);
            if (status != HAL_BUSY)
                break;
            osDelay(1);
        }
    }
}
} // namespace

/** @brief 创建 UART7 回环任务和接收队列；重复调用不重复创建。 */
void serialTasks::CreateUartSerialTask()
{
    if (taskHandle != nullptr)
        return;

    rxQueue = osMessageQueueNew(kQueueDepth, sizeof(Packet), nullptr);
    if (rxQueue == nullptr)
        Error_Handler();

    taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
    if (taskHandle == nullptr)
        Error_Handler();
}
