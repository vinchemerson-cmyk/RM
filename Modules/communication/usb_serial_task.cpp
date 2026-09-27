/**
 * @file usb_serial_task.cpp
 * @brief 使用 USB CDC 接收队列和任务实现虚拟串口回环。
 */

#include "usb_serial_task.h"

#include <cstring>

#include "cmsis_os2.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

extern "C" USBD_HandleTypeDef hUsbDeviceHS;

namespace {
constexpr uint32_t kPacketSize = 64;
constexpr uint32_t kQueueDepth = 8;

/** @brief 从 USB 接收回调复制到任务队列的数据包。 */
struct Packet {
    uint16_t length;
    uint8_t data[kPacketSize];
};

osMessageQueueId_t rxQueue = nullptr;
osThreadId_t taskHandle = nullptr;
uint8_t txBuffer[kPacketSize]{}; // CDC 异步发送完成前必须保持有效。

constexpr osThreadAttr_t taskAttrs = {
    .name = "UsbSerialTask",
    .stack_size = 1024U,
    .priority = osPriorityNormal,
};

/** @brief 检查主机是否已完成 USB CDC 配置。 */
bool IsConfigured()
{
    return hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED &&
           hUsbDeviceHS.pClassData != nullptr;
}

/** @brief 等待 CDC 发送端点空闲并回传接收队列中的数据。 */
void Task(void *argument)
{
    (void)argument;
    Packet packet{};

    for (;;) {
        if (osMessageQueueGet(rxQueue, &packet, nullptr, osWaitForever) != osOK)
            continue;

        // USB 断开后不等待发送，下一次连接仍可接收新数据。
        while (IsConfigured()) {
            auto *cdc = static_cast<USBD_CDC_HandleTypeDef *>(hUsbDeviceHS.pClassData);
            if (cdc->TxState == 0)
                break;
            osDelay(1);
        }
        if (!IsConfigured())
            continue;

        std::memcpy(txBuffer, packet.data, packet.length);
        while (IsConfigured()) {
            const uint8_t result = CDC_Transmit_HS(txBuffer, packet.length);
            if (result == USBD_OK || result == USBD_FAIL)
                break;
            osDelay(1);
        }
    }
}
} // namespace

/** @brief 在 USB 接收回调中将数据分包送入任务队列。 */
extern "C" void USBSerial_OnReceive(const uint8_t *data, uint32_t length)
{
    if (rxQueue == nullptr || data == nullptr)
        return;

    while (length != 0) {
        Packet packet{};
        packet.length = static_cast<uint16_t>(length > kPacketSize ? kPacketSize : length);
        std::memcpy(packet.data, data, packet.length);
        if (osMessageQueuePut(rxQueue, &packet, 0, 0) != osOK)
            return; // 队列满时丢弃剩余数据，避免在 USB 中断中等待。
        data += packet.length;
        length -= packet.length;
    }
}

/** @brief 创建 USB CDC 回环任务和接收队列；重复调用不重复创建。 */
void serialTasks::CreateUsbSerialTask()
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
