/**
 * @file usb_serial_task.h
 * @brief 声明 USB CDC 接收回调和任务创建入口。
 */

#ifndef INFANTRY_H723_USB_SERIAL_TASK_H
#define INFANTRY_H723_USB_SERIAL_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 将 USB CDC 收到的数据投递给通信任务，在 CDC 接收回调中调用。 */
void USBSerial_OnReceive(const uint8_t *data, uint32_t length);

#ifdef __cplusplus
}

namespace serialTasks {
/** @brief 创建 USB CDC 接收回环任务。 */
void CreateUsbSerialTask();
}
#endif

#endif
