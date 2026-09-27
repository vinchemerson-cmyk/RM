/**
 * @file usb.hpp
 * @brief 封装 USB CDC 设备注册、接收和发送。
 */

#ifndef USB_HPP
#define USB_HPP

#include <array>
#include <cstdint>
#include <cstring>

#include "usbConfig.hpp"

extern "C"
{
#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_desc.h"

	// USB Device 库的全局设备句柄.
	extern USBD_HandleTypeDef hUsbDeviceHS;
}

class usb
{
private:
	using usbAttributeType	= usbConfig::Attribute;	 ///< USB 实例属性类型.
	using usbConfigType		= usbConfig::Config;	 ///< USB 注册配置类型.
	using usbDependencyType = usbConfig::Dependency; ///< USB Device 依赖类型.
	using usbInputType		= usbConfig::Input;		 ///< USB 发送输入类型.
	using usbOutputType		= usbConfig::Output;	 ///< USB 接收输出类型.

	/// @brief 单个 USB 逻辑实例的全部持久运行对象.
	struct Instance
	{
		usbAttributeType attribute{};					///< 注册配置的持久属性.
		usbDependencyType dependency{&hUsbDeviceHS};	///< USB Device 句柄和 CDC 回调表.
		usbInputType input{};							///< CDC 异步发送长期输入.
		usbOutputType output{};							///< CDC 接收运行时输出.
	};

	/// @brief USB 逻辑实例注册表, 定义在 usb.cpp 并放入 AXI SRAM.
	static std::array<usb, usbConfig::MAX_INSTANCE> _usb_instance_table;

	/// @brief 当前已注册 USB 逻辑实例数量.
	static inline uint8_t _usb_instance_amount{};

	/// @brief USB Device 核心是否已完成初始化和启动.
	static inline bool _usb_is_started{};

	/// @brief 当前 USB 逻辑实例的持久运行对象.
	Instance _instance{};

	/**
	 * @brief CDC 类初始化回调.
	 * @return USBD_OK 表示内部收发缓冲区绑定成功, 否则返回 USBD_FAIL.
	 *
	 * @note 由 USB Device 核心调用. 当前仅支持一个 CDC 接口, 使用注册表首个实例.
	 */
	static int8_t _cdcInit()
	{
		auto &usb_instance = _usb_instance_table[0];
		auto &dependency   = usb_instance._instance.dependency;

		if (_usb_instance_amount == 0 || dependency.usb_handle == nullptr)
			return USBD_FAIL;

		// 设置发送缓冲区首地址. 此时不提交实际发送数据.
		USBD_CDC_SetTxBuffer(dependency.usb_handle, usb_instance._instance.input.tx_buffer.data(), 0);

		// 设置后续 OUT 端点传输使用的接收缓冲区.
		USBD_CDC_SetRxBuffer(dependency.usb_handle, usb_instance._instance.output.rx_buffer.data());

		return USBD_OK;
	}

	/**
	 * @brief CDC 类反初始化回调.
	 * @return 始终返回 USBD_OK.
	 */
	static int8_t _cdcDeInit()
	{
		return USBD_OK;
	}

	/**
	 * @brief CDC 控制请求回调.
	 * @param cmd CDC 控制请求命令.
	 * @param pbuf 控制请求数据缓冲区.
	 * @param length 控制请求数据长度.
	 * @return 始终返回 USBD_OK.
	 *
	 * @note 当前不保存主机设置的波特率或串口参数.
	 */
	static int8_t _cdcControl(uint8_t cmd, uint8_t *pbuf, uint16_t length)
	{
		(void)cmd;
		(void)pbuf;
		(void)length;

		return USBD_OK;
	}

	/**
	 * @brief CDC OUT 端点接收完成回调.
	 * @param pbuf USB Device 库传入的接收缓冲区.
	 * @param len 本次接收长度指针.
	 * @return 始终返回 USBD_OK.
	 *
	 * @note 保持"更新长度, 执行上层回调, 重新提交接收"的既有顺序.
	 */
	static int8_t _cdcReceive(uint8_t *pbuf, uint32_t *len)
	{
		auto &usb_instance = _usb_instance_table[0];
		auto &dependency   = usb_instance._instance.dependency;

		// 仅处理有效数据. 空包不触发上层回调.
		if (pbuf != nullptr && len != nullptr && *len != 0)
			_isrHandler(pbuf, *len);

		// 接收处理后恢复内部接收缓冲区地址.
		if (_usb_instance_amount != 0)
		{
			USBD_CDC_SetRxBuffer(dependency.usb_handle,
								 usb_instance._instance.output.rx_buffer.data());
		}

		// 重新使能 CDC OUT 端点接收下一包数据.
		USBD_CDC_ReceivePacket(dependency.usb_handle);

		return USBD_OK;
	}

	/**
	 * @brief 初始化 USB Device 核心和 CDC 类.
	 *
	 * @note 仅由首次 Register() 调用.
	 * @note 不可与 CubeMX 的 MX_USB_DEVICE_Init() 同时初始化同一个 hUsbDeviceHS.
	 */
	static void _enable()
	{
		auto &dependency = _usb_instance_table[0]._instance.dependency;

		if (dependency.usb_handle == nullptr)
			Error_Handler();

		// CDC 回调表必须在 USB Device 注册接口前完成初始化.
		dependency.cdc_fops.Init		  = _cdcInit;
		dependency.cdc_fops.DeInit		  = _cdcDeInit;
		dependency.cdc_fops.Control	  = _cdcControl;
		dependency.cdc_fops.Receive	  = _cdcReceive;
		dependency.cdc_fops.TransmitCplt = nullptr;

		// 初始化 USB Device 核心, 其中会调用底层 USBD_LL_Init().
		if (USBD_Init(dependency.usb_handle, &HS_Desc, DEVICE_HS) != USBD_OK)
			Error_Handler();

		// 向 USB Device 核心注册 CDC 类.
		if (USBD_RegisterClass(dependency.usb_handle, &USBD_CDC) != USBD_OK)
			Error_Handler();

		// 向 CDC 类注册本封装提供的回调表.
		if (USBD_CDC_RegisterInterface(dependency.usb_handle, &dependency.cdc_fops) != USBD_OK)
			Error_Handler();

		// 启动底层 PCD. 主机枚举后才会进入已配置状态.
		if (USBD_Start(dependency.usb_handle) != USBD_OK)
			Error_Handler();

		// 开启 USB 供电电压检测.
		HAL_PWREx_EnableUSBVoltageDetector();

		_usb_is_started = true;
	}

	/**
	 * @brief 处理 CDC 接收数据并分发上层回调.
	 * @param data 本次接收数据首地址.
	 * @param size 本次接收数据长度.
	 *
	 * @note 上层回调应只做快速处理. 如需长期保存数据, 必须自行复制.
	 */
	static void _isrHandler(uint8_t *data, uint32_t size)
	{
		if (_usb_instance_amount == 0 || data == nullptr || size == 0)
			return;

		auto &usb_instance = _usb_instance_table[0];

		// 防止异常长度导致内部接收缓冲区越界.
		if (size > usbConfig::RX_BUF_SIZE)
			Error_Handler();

		// 正常情况下 data 已经是 rx_buffer. 仅在地址不同的情况下复制.
		if (data != usb_instance._instance.output.rx_buffer.data())
			std::memcpy(usb_instance._instance.output.rx_buffer.data(), data, size);

		// 在回调前更新长度, 供上层通过 Receive() 获取.
		usb_instance._instance.output.rx_size = size;

		// 将收到的数据通知给已注册的上层模块.
		if (usb_instance._instance.attribute.usb_callback_func != nullptr)
			usb_instance._instance.attribute.usb_callback_func(&usb_instance);
	}

public:
	/**
	 * @brief 默认构造 USB 逻辑实例.
	 */
	usb() = default;

	/**
	 * @brief 默认析构 USB 逻辑实例.
	 */
	~usb() = default;

	/**
	 * @brief 注册上层 USB 模块并启动 USB Device.
	 * @param config_instance 上层回调和模块实例配置.
	 * @return 已注册的 USB 逻辑实例指针.
	 *
	 * @note 当前 MAX_INSTANCE 为 1. 注册表满或同一模块重复注册会调用 Error_Handler().
	 * @note 实例先进入注册表, 再启动 USB, 确保 _cdcInit() 可访问当前实例.
	 */
	static usb *Register(const usbConfigType &config_instance)
	{
		if (config_instance.module_instance == nullptr)
			Error_Handler();

		if (_usb_instance_amount >= usbConfig::MAX_INSTANCE)
			Error_Handler();

		for (uint8_t i = 0; i < _usb_instance_amount; i++)
		{
			if (_usb_instance_table[i]._instance.attribute.module_instance == config_instance.module_instance)
				Error_Handler();
		}

		// 先写入注册表, 确保后续 _enable() 回调能找到当前实例.
		auto *certain_usb_instance = &_usb_instance_table[_usb_instance_amount++];

		// 清除上次运行状态并保存本次注册配置.
		certain_usb_instance->_instance = Instance{};

		certain_usb_instance->_instance.attribute.usb_callback_func = config_instance.usb_callback_func;
		certain_usb_instance->_instance.attribute.module_instance   = config_instance.module_instance;

		// 仅由首次注册启动 USB. 后续注册不重复初始化硬件.
		if (!_usb_is_started)
			_enable();

		return certain_usb_instance;
	}

	/**
	 * @brief 使用本次 Input 发送一帧 CDC 数据.
	 * @param input_instance 本次发送的外部数据输入.
	 * @return USBD_OK, USBD_BUSY 或 USBD_FAIL.
	 *
	 * @note 主机未完成配置或上一包仍在传输时返回 USBD_BUSY.
	 * @note CDC IN 端点异步读取发送数据, 因此仅在空闲时复制到实例长期 Input.
	 * @note 本接口不提供发送队列, 上层负责发送串行化.
	 */
	uint8_t Transmit(const usbInputType &input_instance)
	{
		auto &dependency = _instance.dependency;

		if (input_instance.tx_len == 0 ||
			input_instance.tx_len > usbConfig::TX_BUF_SIZE ||
			dependency.usb_handle == nullptr)
		{
			Error_Handler();
		}

		// 主机未完成 SET_CONFIGURATION 时, CDC 类数据尚不可发送.
		if (dependency.usb_handle->dev_state != USBD_STATE_CONFIGURED)
			return USBD_BUSY;

		// pClassData 在 CDC 类被主机配置后由 USB Device 库创建.
		auto *hcdc = static_cast<USBD_CDC_HandleTypeDef *>(dependency.usb_handle->pClassData);

		if (hcdc == nullptr)
			return USBD_FAIL;

		// TxState 非零表示上一包数据仍在传输, 此时不能覆盖实例长期发送输入.
		if (hcdc->TxState != 0)
			return USBD_BUSY;

		_instance.input = input_instance;

		USBD_CDC_SetTxBuffer(dependency.usb_handle,
							 _instance.input.tx_buffer.data(),
							 _instance.input.tx_len);

		// 启动 IN 端点传输. 完成后由 USB 中断链路清除 TxState.
		return USBD_CDC_TransmitPacket(dependency.usb_handle);
	}

	/**
	 * @brief 获取当前 USB 实例的接收输出.
	 * @return 最近一次 CDC 接收数据和实际接收长度.
	 *
	 * @note 下一帧接收会覆盖该输出中的 rx_buffer.
	 */
	[[nodiscard]] const usbOutputType &Receive() const
	{
		return _instance.output;
	}

	/**
	 * @brief 获取注册时传入的上层模块对象.
	 * @return 上层模块实例指针.
	 */
	[[nodiscard]] void *getModuleInstance() const
	{
		return _instance.attribute.module_instance;
	}
};

#endif // USB_HPP