#ifndef USART_HPP
#define USART_HPP

#include <array>
#include <cstdint>
#include <cstring>
#include "usartConfig.hpp"

class usart
{
private:
	// 类型萃取
	using usartAttributeType  = usartConfig::Attribute;	 // USART 生命周期属性类型, 保存发送模式、回调和上层实例.
	using usartConfigType	  = usartConfig::Config;	 // USART 注册配置类型, 保存 Register() 的上层输入.
	using usartDependencyType = usartConfig::Dependency; // USART 底层依赖类型, 保存 HAL UART 句柄.
	using usartInputType	  = usartConfig::Input;		 // USART 发送输入类型, 保存本次发送数据.
	using usartOutputType	  = usartConfig::Output;	 // USART 接收输出类型, 保存 DMA 接收缓冲区和接收状态.

	// 单个 USART 逻辑实例的全部持久运行对象.
	struct Instance
	{
		usartAttributeType attribute{};	  // 注册配置的持久属性.
		usartDependencyType dependency{}; // HAL UART 依赖.
		usartInputType input{};			  // IT 和 DMA 发送缓存.
		usartOutputType output{};		  // 运行时接收输出.
	};

	// AXI SRAM 实例表, DMA 直接可达.
	static std::array<usart, usartConfig::MAX_INSTANCE> _usart_instance_table;
	static inline uint8_t _usart_instance_amount{}; // 已注册实例数.

	Instance _instance{}; // 当前 USART 逻辑实例的持久运行对象.

	/**
	 * @brief 启动 DMA+Idle 接收, 清标志, 关 DMA 半传输中断.
	 * @param usart_instance 需要启动的串口实例.
	 */
	static void _enable(usart *usart_instance)
	{
		auto *usart_handle = usart_instance->_instance.dependency.usart_handle;

		// 启动 DMA 接收前, 先清掉串口残留状态.
		__HAL_UART_CLEAR_FLAG(usart_handle,
							  UART_CLEAR_IDLEF |
								  UART_CLEAR_OREF |
								  UART_CLEAR_NEF |
								  UART_CLEAR_FEF |
								  UART_CLEAR_PEF);

		__HAL_UART_SEND_REQ(usart_handle, UART_RXDATA_FLUSH_REQUEST);

		HAL_UARTEx_ReceiveToIdle_DMA(usart_handle,
									 usart_instance->_instance.output.rx_buffer.data(),
									 usart_instance->_instance.output.rx_len);

		// 关闭 dma half transfer 中断防止两次进入 HAL_UARTEx_RxEventCallback().
		// 这是 HAL 库的一个设计失误, 发生 DMA 传输完成, 半完成以及串口 IDLE 中断都会触发 HAL_UARTEx_RxEventCallback().
		// 我们只希望处理第一种和第三种情况, 因此直接关闭 DMA 半传输中断.
		__HAL_DMA_DISABLE_IT(usart_handle->hdmarx, DMA_IT_HT);
	}

public:
	usart() = default;

	~usart() = default;

	/**
	 * @brief   注册一个 USART 实例, 填入注册表并启动 DMA 接收.
	 * @param   config_instance 用户配置, 包含 handle, tx_mode, rx_len, 回调和 module_instance.
	 * @return  指向已注册 usart 实例的指针.
	 * @note    模块实例不可重复注册, 注册即启动 DMA+Idle 接收.
	 */
	static usart *Register(const usartConfigType &config_instance)
	{
		// UART 句柄和上层模块实例是注册及 ISR 回调分发的必要依赖.
		if (config_instance.usart_handle == nullptr || config_instance.module_instance == nullptr)
			Error_Handler();

		// DMA 接收配置长度不能超过实例长期接收缓冲区容量.
		if (config_instance.rx_data_len > usartConfig::RX_BUF_SIZE)
			Error_Handler();

		// 注册表满时禁止继续写入, 防止实例表越界.
		if (_usart_instance_amount >= usartConfig::MAX_INSTANCE)
			Error_Handler();

		// 同一上层模块只能注册一个 USART 实例.
		for (uint8_t i = 0; i < _usart_instance_amount; i++)
		{
			if (_usart_instance_table[i]._instance.attribute.module_instance == config_instance.module_instance)
				Error_Handler();
		}

		// 先增加实例数量, 确保注册阶段 DMA 启动失败时 errorHandler() 也能查到当前实例.
		auto *certain_usart_instance = &_usart_instance_table[_usart_instance_amount++];

		// 保存 HAL 句柄、运行属性及上层关联对象.
		certain_usart_instance->_instance.dependency.usart_handle		= config_instance.usart_handle;
		certain_usart_instance->_instance.attribute.tx_mode				= config_instance.tx_mode;
		certain_usart_instance->_instance.output.rx_len					= config_instance.rx_data_len;
		certain_usart_instance->_instance.output.rx_size				= 0;
		certain_usart_instance->_instance.attribute.usart_callback_func = config_instance.usart_callback_func;
		certain_usart_instance->_instance.attribute.module_instance		= config_instance.module_instance;

		// rx_data_len 为 0 表示 TX-only 实例, 不启动 DMA 接收.
		if (config_instance.rx_data_len > 0)
			_enable(certain_usart_instance);

		return certain_usart_instance;
	}

	/**
	 * @brief   使用本次 Input 发送一帧 USART 数据.
	 * @param   input_instance 本次发送的外部数据输入.
	 * @return  HAL_OK, HAL_BUSY 或 HAL_ERROR.
	 *
	 * @note    阻塞发送直接使用 Input 缓冲区.
	 * @note    IT 和 DMA 发送会复制到实例长期输入.
	 */
	HAL_StatusTypeDef Transmit(const usartInputType &input_instance)
	{
		if (input_instance.tx_len == 0 ||
			input_instance.tx_len > usartConfig::TX_BUF_SIZE ||
			_instance.dependency.usart_handle == nullptr)
		{
			Error_Handler();
		}

		switch (_instance.attribute.tx_mode)
		{
			case usartConfig::TRANSFER_BLOCKING:
			{
				return HAL_UART_Transmit(_instance.dependency.usart_handle,
										 input_instance.tx_buffer.data(),
										 input_instance.tx_len,
										 10);
			}
			case usartConfig::TRANSFER_IT:
			{
				if (_instance.dependency.usart_handle->gState != HAL_UART_STATE_READY)
					return HAL_BUSY;

				_instance.input = input_instance;

				return HAL_UART_Transmit_IT(_instance.dependency.usart_handle,
											_instance.input.tx_buffer.data(),
											_instance.input.tx_len);
			}

			case usartConfig::TRANSFER_DMA:
			{
				if (_instance.dependency.usart_handle->gState != HAL_UART_STATE_READY ||
					HAL_DMA_GetState(_instance.dependency.usart_handle->hdmatx) != HAL_DMA_STATE_READY)
				{
					return HAL_BUSY;
				}

				_instance.input = input_instance;

				return HAL_UART_Transmit_DMA(_instance.dependency.usart_handle,
											 _instance.input.tx_buffer.data(),
											 _instance.input.tx_len);
			}
			default:
				Error_Handler();
				return HAL_ERROR;
		}
	}

	/**
	 * @brief 获取当前 USART 实例的接收输出.
	 * @return 最近一次 DMA 接收数据和注册接收长度.
	 */
	[[nodiscard]] const usartOutputType &Receive() const
	{
		return _instance.output;
	}

	/**
	 * @brief 用于获得上层模块实例指针.
	 * @return 上层模块实例指针.
	 */
	[[nodiscard]] void *getModuleInstance() const
	{
		return _instance.attribute.module_instance;
	}

	/**
	 * @brief 用于当和使用串口通信的设备掉线后重启 DMA 接收, 先 Abort 再重新启动.
	 */
	void restartRx()
	{
		// 先中止当前 UART 收发状态, 确保后续 DMA 接收可以重新启动.
		HAL_UART_Abort(_instance.dependency.usart_handle);

		// 清除可能导致接收链路无法恢复的 UART 状态标志.
		__HAL_UART_CLEAR_FLAG(_instance.dependency.usart_handle,
							  UART_CLEAR_IDLEF |
								  UART_CLEAR_OREF |
								  UART_CLEAR_NEF |
								  UART_CLEAR_FEF |
								  UART_CLEAR_PEF);

		// 清空接收数据寄存器中的残留字节.
		__HAL_UART_SEND_REQ(_instance.dependency.usart_handle, UART_RXDATA_FLUSH_REQUEST);

		// 使用实例长期持有的接收缓冲区和注册长度重启 DMA+Idle 接收.
		HAL_UARTEx_ReceiveToIdle_DMA(_instance.dependency.usart_handle,
									 _instance.output.rx_buffer.data(),
									 _instance.output.rx_len);

		// 禁用 DMA 半传输中断, 保持正常接收回调的触发语义.
		__HAL_DMA_DISABLE_IT(_instance.dependency.usart_handle->hdmarx, DMA_IT_HT);
	}

	/**
	 * @brief 每次 dma 或 idle 中断发生时, 都会调用此函数.
	 *        对于每个 UART 实例会调用对应的回调进行进一步处理.
	 *        例如, 视觉协议解析, 遥控器解析, 裁判系统解析.
	 *
	 * @note  通过 __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT) 关闭 dma half transfer 中断.
	 *        这是 HAL 库的一个设计失误, 发生 DMA 传输完成, 半完成以及串口 IDLE 中断都会触发 HAL_UARTEx_RxEventCallback().
	 *        我们只希望处理第一种和第三种情况, 因此直接关闭 DMA 半传输中断.
	 *
	 * @param huart 发生中断的串口.
	 * @param size 此次接收到的总数据量.
	 */
	static void isrHandler(bspUsartHandleTypeDef *huart, uint16_t size)
	{
		// 仅遍历已注册实例, 避免访问未初始化的注册表项.
		for (uint8_t i = 0; i < _usart_instance_amount; i++)
		{
			auto *usart_instance = &_usart_instance_table[i];

			// 未匹配当前中断来源的 UART 句柄时, 继续查找下一实例.
			if (huart != usart_instance->_instance.dependency.usart_handle)
				continue;

			// 空帧不更新长度, 也不通知上层模块.
			if (size != 0)
			{
				// 回调前先保存本次实际接收长度, 供上层通过 Receive() 读取.
				usart_instance->_instance.output.rx_size = size;

				// 如果上层注册了接收回调, 则分发本次有效接收事件.
				if (usart_instance->_instance.attribute.usart_callback_func != nullptr)
					usart_instance->_instance.attribute.usart_callback_func(usart_instance);
			}

			// 保持原有顺序: 更新数据并执行回调后, 重新提交 DMA+Idle 接收.
			// size 为 0 时也需要重新提交接收, 防止接收链路停止.
			HAL_UARTEx_ReceiveToIdle_DMA(usart_instance->_instance.dependency.usart_handle,
										 usart_instance->_instance.output.rx_buffer.data(),
										 usart_instance->_instance.output.rx_len);

			// 禁用 DMA 半传输中断, 避免半传输事件重复触发接收回调.
			__HAL_DMA_DISABLE_IT(usart_instance->_instance.dependency.usart_handle->hdmarx, DMA_IT_HT);

			// 当前 UART 句柄已处理完成, 不再继续遍历注册表.
			return;
		}
	}

	/**
	 * @brief 当串口发送或接收出现错误时, 会调用此函数, 此时需要重新启动接收.
	 * @note 最常见的错误, 奇偶校验, 溢出, 帧错误.
	 * @param huart 发生错误的串口.
	 */
	static void errorHandler(bspUsartHandleTypeDef *huart)
	{
		for (uint8_t i = 0; i < _usart_instance_amount; i++)
		{
			auto *usart_instance = &_usart_instance_table[i];

			// 在已注册实例表中查找发生错误的 UART 句柄.
			if (huart != usart_instance->_instance.dependency.usart_handle)
				continue;

			// 清除 IDLE、溢出、噪声、帧和奇偶校验错误标志.
			// 防止残留错误状态影响下一次 DMA 接收启动.
			__HAL_UART_CLEAR_FLAG(huart,
								  UART_CLEAR_IDLEF |
									  UART_CLEAR_OREF |
									  UART_CLEAR_NEF |
									  UART_CLEAR_FEF |
									  UART_CLEAR_PEF);

			// 清空 UART 接收数据寄存器中的残留数据.
			__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

			// 使用该实例的长期接收缓冲区和注册长度重新启动 DMA+Idle 接收.
			HAL_UARTEx_ReceiveToIdle_DMA(usart_instance->_instance.dependency.usart_handle,
										 usart_instance->_instance.output.rx_buffer.data(),
										 usart_instance->_instance.output.rx_len);

			// 禁用 DMA 半传输中断, 保持与正常接收回调一致的触发语义.
			__HAL_DMA_DISABLE_IT(usart_instance->_instance.dependency.usart_handle->hdmarx, DMA_IT_HT);

			// 一个 UART 句柄只处理一次, 恢复完成后退出遍历.
			return;
		}
	}
};

#endif
