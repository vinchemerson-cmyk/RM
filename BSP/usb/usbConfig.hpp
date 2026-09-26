#ifndef USB_CONFIG_HPP
#define USB_CONFIG_HPP

#include <array>
#include <cstdint>

extern "C"
{
#include "main.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
}

class usb; // 前向声明, 避免 USB 回调类型缺失.

namespace usbConfig
{
	/// @brief 单次 CDC IN 端点最大发送长度.
	static constexpr uint32_t TX_BUF_SIZE = APP_TX_DATA_SIZE;

	/// @brief 单次 CDC OUT 端点最大接收长度.
	static constexpr uint32_t RX_BUF_SIZE = APP_RX_DATA_SIZE;

	/// @brief USB 上层逻辑实例最大注册数量.
	static constexpr uint32_t MAX_INSTANCE = 1;

	/**
	 * @brief USB 发送输入.
	 */
	struct Input
	{
		std::array<uint8_t, TX_BUF_SIZE> tx_buffer{}; ///< USB IN 端点发送缓冲区.
		uint32_t tx_len{};							  ///< 当前待发送数据长度.
	};

	/**
	 * @brief USB 接收输出.
	 */
	struct Output
	{
		std::array<uint8_t, RX_BUF_SIZE> rx_buffer{}; ///< USB OUT 端点接收缓冲区.
		uint32_t rx_size{};							  ///< 最近一次接收数据长度.
	};

	/**
	 * @brief USB 实例整个生命周期内持续存在的属性.
	 */
	struct Attribute
	{
		void (*usb_callback_func)(usb *){}; ///< 收到数据时的上层回调.
		void *module_instance{};			///< 注册该 USB 实例的上层模块对象.
	};

	/**
	 * @brief USB 注册配置.
	 * 仅保存上层在 Register() 时传入的信息.
	 */
	struct Config
	{
		void (*usb_callback_func)(usb *){}; ///< 收到数据时的上层回调.
		void *module_instance{};			///< 上层模块实例, 用于去重和回调关联.
	};

	/**
	 * @brief USB 运行依赖.
	 */
	struct Dependency
	{
		USBD_HandleTypeDef *usb_handle{}; ///< USB Device 全局句柄.
		USBD_CDC_ItfTypeDef cdc_fops{};	  ///< 注册给 USB Device 库的 CDC 回调表.
	};
} // namespace usbConfig

#endif // USB_CONFIG_HPP
