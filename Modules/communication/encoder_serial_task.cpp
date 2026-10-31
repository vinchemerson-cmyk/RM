/** @file encoder_serial_task.cpp @brief UART7 DMA 每 10ms 输出最新云台 pitch 或底盘四轮诊断。 */
#include "encoder_serial_task.h"
#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"
#include "usart/usart.hpp"

namespace
{
	osThreadId_t                 taskHandle       = nullptr;
	serialTasks::EncoderProvider feedbackProvider = nullptr;
	serialTasks::ChassisProvider chassisProvider  = nullptr;
	constexpr osThreadAttr_t     taskAttrs{
		.name = "EncoderSerialTask", .stack_size = 2048U, .priority = osPriorityNormal,
	};
	constexpr uint32_t kPrintPeriodMs = 10U;//串口打印周期
	static_assert(usartConfig::TX_BUF_SIZE >= 256U, "UART control telemetry requires a 256-byte buffer");

	/** @brief 在任务中注册 UART7；BSP 将临时文本复制至 AXI SRAM 长期 DMA 缓冲。 */
	void Task(void*)
	{
		constexpr usartConfig::Config config{
			.usart_handle        = &huart7,
			.tx_mode             = usartConfig::TRANSFER_DMA,
			.rx_data_len         = 0U,
			.usart_callback_func = nullptr,
			.module_instance     = &taskHandle,
		};
		auto* uart = usart::Register(config);
		for (;;)
		{
			usartConfig::Input tx{};
			const uint32_t     now = osKernelGetTickCount();
			if (chassisProvider != nullptr)
			{
				const auto snapshot = chassisProvider();
				tx.tx_len = static_cast<uint32_t>(serialTasks::FormatChassisTelemetryLine(
					reinterpret_cast<char*>(tx.tx_buffer.data()), tx.tx_buffer.size(), snapshot, now));
			}
			else
			{
				const auto snapshot = feedbackProvider();
				tx.tx_len = static_cast<uint32_t>(serialTasks::FormatPitchControlTelemetryLine(
					reinterpret_cast<char*>(tx.tx_buffer.data()), tx.tx_buffer.size(), snapshot, now));
			}
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
	taskHandle       = osThreadNew(Task, nullptr, &taskAttrs);
	if (taskHandle == nullptr) Error_Handler();
}

void serialTasks::CreateChassisSerialTask(ChassisProvider provider)
{
	if (taskHandle != nullptr) return;
	if (provider == nullptr) Error_Handler();
	chassisProvider = provider;
	taskHandle      = osThreadNew(Task, nullptr, &taskAttrs);
	if (taskHandle == nullptr) Error_Handler();
}
