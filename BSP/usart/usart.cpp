#include "usart.hpp"
#include "memory_layout.h"

DMA_BUFFER
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
