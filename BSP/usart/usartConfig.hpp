/**
 * @file usartConfig.hpp
 * @brief 定义 UART 收发缓冲区、回调和注册配置。
 */

#ifndef USART_CONFIG_HPP
#define USART_CONFIG_HPP

#include <array>
#include <cstdint>

extern "C" {
#include "main.h"
}

class usart; // 前向声明, 避免 USART 回调类型缺失.

using bspUsartHandleTypeDef = UART_HandleTypeDef; // HAL_USART 句柄类型别名.

namespace usartConfig
{
	static constexpr uint32_t TX_BUF_SIZE  = 256; // 发送缓冲区大小, 单位字节.
	static constexpr uint32_t RX_BUF_SIZE  = 256; // 接收缓冲区大小, 单位字节.
	static constexpr uint32_t MAX_INSTANCE = 8;   // huart1, huart5, huart7, huart8, huart9, huart10.

	// 串口发送模式.
	enum TX_MODE : uint8_t
	{
		NO_TRANSFER,       // 不发送.
		TRANSFER_BLOCKING, // 阻塞发送.
		TRANSFER_IT,       // 中断发送.
		TRANSFER_DMA,      // DMA 发送.
	};

	// USART 发送输入.
	struct Input
	{
		std::array<uint8_t, TX_BUF_SIZE> tx_buffer{}; // 发送缓冲区.
		uint32_t                         tx_len{};    // 待发送数据长度.
	};

	// USART 接收输出.
	struct Output
	{
		std::array<uint8_t, RX_BUF_SIZE> rx_buffer{}; // DMA 接收缓冲区.
		uint32_t                         rx_len{};    // DMA 接收缓冲配置长度.
		uint32_t                         rx_size{};   // 本次实际接收字节数.
	};

	// USART 实例整个生命周期内持续存在的属性.
	struct Attribute
	{
		TX_MODE tx_mode{}; // 发送模式.

		void (*usart_callback_func)(usart*){}; // 接收完成回调.
		void*  module_instance{};              // 上层模块实例指针.
	};

	// USART 运行依赖.
	struct Dependency
	{
		bspUsartHandleTypeDef* usart_handle{}; // HAL UART 句柄.
	};

	// USART 注册配置.
	struct Config
	{
		bspUsartHandleTypeDef* usart_handle{}; // UART 句柄.

		TX_MODE tx_mode{}; // 发送模式.

		uint32_t rx_data_len{}; // DMA 接收缓冲长度.

		void (*usart_callback_func)(usart*){}; // 收到数据时的回调.
		void*  module_instance{};              // 上层模块实例, 用于去重和 ISR 转发.
	};
} // namespace usartConfig

#endif
