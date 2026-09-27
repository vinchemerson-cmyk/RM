/**
 * @file usart.cpp
 * @brief 保存串口实例表并转发 HAL 串口回调。
 */

#include "usart.hpp"

__attribute__((section(".axi_bss")))
std::array<usart, usartConfig::MAX_INSTANCE> usart::_usart_instance_table{};

// -------- HAL 中断回调 --------
extern "C"
{
	void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
	{
		usart::isrHandler(huart, Size);
	}

	void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
	{
		usart::errorHandler(huart);
	}
}
