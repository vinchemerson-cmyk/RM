/**
 * @file usb_serial_task.h
 * @brief 声明 USB CDC 姿态发送任务的创建入口。
 */

#ifndef INFANTRY_H723_USB_SERIAL_TASK_H
#define INFANTRY_H723_USB_SERIAL_TASK_H

namespace serialTasks
{
	/** @brief 创建 USB CDC 周期姿态发送任务。 */
	void CreateUsbSerialTask();
}

#endif
