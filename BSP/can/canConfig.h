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
		};
	};

	//-------------FDCAN配置-------------
	struct Fdcan
	{
		// TODO[CANFD-03 容量与帧长]：同路混发DM 8字节与板间64字节时，将FD收发容量扩为64，
		// 同时配套硬件元素大小和所有HAL接收缓冲区；classicCan仍保留8，不要一起改成64。
		// DATA_SIZE只是最大存储容量；DM每帧仍用tx_len=8，板间帧按本帧实际DLC发送。
		// 扩容后DM的Pack(std::array<uint8_t,8>&)不能直接接收此处64字节数组，见dmmotor.cpp。
		static constexpr uint8_t  DATA_SIZE         = 8;//不能用64字节模式一次性控制多个达妙电机
		static constexpr uint8_t  MAX_FILTER_AMOUNT = 12;
		static constexpr bool     IS_CLASSIC        = false;
		// TODO[CANFD-03 真正FD帧]：此处改为FDCAN_FD_CAN，并同步外设模式、DM初始化检查与电机端配置。
		// 仅把类型起名Fdcan或IS_CLASSIC设false，并不会使当前经典帧自动变成CAN FD。
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

		//FDCAN注册的持久属性
		struct Attribute
		{
			uint32_t master_id{};
			uint32_t can_id{};
			fifo     use_fifo{};

			void (*can_callback_func)(Attribute*){};
			void*  module_handle{};
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
