/**
 * @file canConfig.h
 * @brief 定义 CAN 设备的句柄、收发配置和实例参数。
 */

//
// Created by 19923 on 2026/9/25.
//

#ifndef INFANTRY_H723_CANCONFIG_H
#define INFANTRY_H723_CANCONFIG_H

#include <array>
#include "main.h"

//FDCAN句柄类型
using bspCanHandleTypeDef = FDCAN_HandleTypeDef;

//FDCAN发送头文件
using bspCanTxHeaderTypeDef = FDCAN_TxHeaderTypeDef;

//FDCAN接收头文件
using bspCanRxHeaderTypeDef = FDCAN_RxHeaderTypeDef;

//FDCAN过滤器类型
using bspCanFilterTypeDef = FDCAN_FilterTypeDef;

namespace canConfig
{
	//FIFO通道选择
	enum class fifo : uint8_t
	{
		CAN_RX_FIFO = FDCAN_RX_FIFO0,//MCU
		CAN_RX_FIF1 = FDCAN_RX_FIFO1,//MCU
	};

	//接收中断类型
	enum class interrupts : uint8_t
	{
		CAN_FIFO0_IT_TYPE = FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
		CAN_FIFO1_IT_TYPE = FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
	};

	//-------------经典CAN配置-------------
	struct classicCan
	{
		static constexpr uint8_t  DATA_SIZE         = 8;
		static constexpr uint8_t  MAX_FILTER_AMOUNT = 16;
		static constexpr bool     IS_CLASSIC        = true;
		static constexpr uint32_t FD_FORMAT         = FDCAN_CLASSIC_CAN;

		//发送
		struct Input
		{
			uint8_t                        tx_len{};
			std::array<uint8_t, DATA_SIZE> tx_buffer{};
		};

		//接收
		struct Output
		{
			uint8_t                        rx_len{};
			std::array<uint8_t, DATA_SIZE> rx_buffer{};
		};

		//CAN注册的持久属性
		struct Attribute
		{
			uint32_t master_id{};
			uint32_t can_id{};
			fifo     use_fifo{};

			void (*can_callback_func)(Attribute*){};
			void*  module_handle{};
			uint8_t tx_reserved_slots{}; // 发此实例时留给其他模块的TX FIFO余量，电机默认0。
		};

		//CAN底层依赖
		struct Dependency
		{
			bspCanHandleTypeDef*   hcan{};
			bspCanTxHeaderTypeDef* tx_header{};
		};

		//CAN注册配置
		struct Config
		{
			bspCanHandleTypeDef* can_handle{};

			uint32_t master_id{};
			uint32_t can_id{};

			void (*can_callback_func)(Attribute*){};
			void*  module_handle{};

			// 本实例的反馈帧进入哪个 RX FIFO; 默认 FIFO0.
			fifo use_fifo{fifo::CAN_RX_FIFO};
			uint8_t tx_reserved_slots{};
		};
	};

	//-------------FDCAN配置-------------
	struct Fdcan
	{
		// 最大容量64，不是固定帧长；DM的实际tx_len仍为8，板间长帧可用至64。
		static constexpr uint8_t  DATA_SIZE         = 64;
		static constexpr uint8_t  MAX_FILTER_AMOUNT = 12;
		static constexpr bool     IS_CLASSIC        = false;
		static constexpr uint32_t FD_FORMAT         = FDCAN_FD_CAN;

		//发送
		struct Input
		{
			uint8_t                        tx_len{};
			std::array<uint8_t, DATA_SIZE> tx_buffer{};
		};

		//接收
		struct Output
		{
			uint8_t                        rx_len{};
			std::array<uint8_t, DATA_SIZE> rx_buffer{};
		};

		//FDCAN注册的持久属性
		struct Attribute
		{
			uint32_t master_id{};
			uint32_t can_id{};
			fifo     use_fifo{};

			void (*can_callback_func)(Attribute*){};
			void*  module_handle{};
			uint8_t tx_reserved_slots{};
		};

		//底层依赖
		struct Dependency
		{
			bspCanHandleTypeDef*   hcan{};
			bspCanTxHeaderTypeDef* tx_header{};
		};

		//注册配置
		struct Config
		{
			bspCanHandleTypeDef* can_handle{};

			uint32_t master_id{};
			uint32_t can_id{};

			void (*can_callback_func)(Attribute*){};
			void*  module_handle{};

			fifo use_fifo{};
			uint8_t tx_reserved_slots{};
		};
	};
}

using classicCan           = canConfig::classicCan;
using classicCanType       = canConfig::classicCan::Attribute;
using classicCanConfigType = canConfig::classicCan::Config;

using Fdcan           = canConfig::Fdcan;
using FdcanType       = canConfig::Fdcan::Attribute;
//FDCAN注册配置
using FdcanConfigType = canConfig::Fdcan::Config;

#endif //INFANTRY_H723_CANCONFIG_H
