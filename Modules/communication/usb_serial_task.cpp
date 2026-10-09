/**
 * @file usb_serial_task.cpp
 * @brief USB CDC 虚拟串口周期发送 BMI088 姿态角。
 *
 * USB 枚举后每约 10 ms 读取最新姿态并发送一行文本。CDC 发送是异步的，
 * 静态发送缓冲区只有在上一次发送完成后才会更新。
 */

#include "usb_serial_task.h"

#include <cmath>
#include <cstdio>

#include "cmsis_os2.h"
#include "imu/ImuTask.h"
#include "main.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

extern "C" USBD_HandleTypeDef hUsbDeviceHS;

namespace
{
	osThreadId_t taskHandle = nullptr;
	char         txBuffer[96]{};
	char         notReadyMessage[] = "IMU not ready\r\n";

	constexpr osThreadAttr_t taskAttrs = {
		.name       = "UsbSerialTask",
		.stack_size = 2048U,
		.priority   = osPriorityNormal,
	};

	/** @brief 检查主机是否已完成 USB CDC 配置。 */
	bool IsConfigured()
	{
		return hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED &&
			   hUsbDeviceHS.pClassData != nullptr;
	}

	/** @brief 将角度换成两位小数，避免 newlib-nano 未开启浮点 printf 的问题。 */
	struct FixedAngle
	{
		char sign;
		long integer;
		long fraction;
	};

	FixedAngle ToFixedAngle(float degrees)
	{
		const long hundredths = static_cast<long>(std::fabs(degrees) * 100.0F + 0.5F);
		return {degrees < 0.0F ? '-' : '+', hundredths / 100L, hundredths % 100L};
	}

	/** @brief 以约 100 Hz 发送 Roll、Pitch、Yaw；无有效姿态时每秒提示一次。 */
	void Task(void* argument)
	{
		(void)argument;
		const uint32_t tickFrequency    = osKernelGetTickFreq();
		const uint32_t sendTicks        = tickFrequency >= 100U ? tickFrequency / 100U : 1U;
		uint32_t       lastNotReadyTick = 0U;
		for (;;)
		{
			if (IsConfigured())
			{
				auto* cdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceHS.pClassData);
				if (cdc->TxState == 0U)
				{
					imu::AttitudeSample sample{};
					if (imu::Latest(&sample))
					{
						const FixedAngle roll   = ToFixedAngle(sample.roll_deg);
						const FixedAngle pitch  = ToFixedAngle(sample.pitch_deg);
						const FixedAngle yaw    = ToFixedAngle(sample.yaw_deg);
						const int        length = std::snprintf(
														 txBuffer, sizeof(txBuffer),
														 "Roll=%c%ld.%02ld,Pitch=%c%ld.%02ld,Yaw=%c%ld.%02ld\r\n",
														 roll.sign, roll.integer, roll.fraction,
														 pitch.sign, pitch.integer, pitch.fraction,
														 yaw.sign, yaw.integer, yaw.fraction);
						if (length > 0 && static_cast<size_t>(length) < sizeof(txBuffer))
							(void)CDC_Transmit_HS(reinterpret_cast<uint8_t*>(txBuffer),
												  static_cast<uint16_t>(length));
					} else
					{
						const uint32_t now = osKernelGetTickCount();
						if (lastNotReadyTick == 0U || now - lastNotReadyTick >= tickFrequency)
						{
							const uint8_t result = CDC_Transmit_HS(
																   reinterpret_cast<uint8_t*>(notReadyMessage),
																   sizeof(notReadyMessage) - 1U);
							if (result == USBD_OK)
								lastNotReadyTick = now;
						}
					}
				}
			}
			osDelay(sendTicks);
		}
	}
} // namespace

/** @brief 创建 USB CDC 姿态发送任务；重复调用不重复创建。 */
void serialTasks::CreateUsbSerialTask()
{
	if (taskHandle != nullptr)
		return;

	taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
	if (taskHandle == nullptr)
		Error_Handler();
}
