/**
 * @file uart_serial_task.h
 * @brief 声明 UART7 串口任务创建入口。
 */

#ifndef INFANTRY_H723_UART_SERIAL_TASK_H
#define INFANTRY_H723_UART_SERIAL_TASK_H

namespace serialTasks
{
	/** @brief 创建 UART7 接收回环任务。 */
	void CreateUartSerialTask();
}

#endif
