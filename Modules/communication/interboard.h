/**
 * @file interboard.h
 * @brief 声明双板链路启动、主题注册和链路状态接口。
 */

#pragma once

#include <cstdint>
#include "interboard_wire.h"

namespace interboard
{
	// FD最大64字节帧 = 4字节头 + 最多60字节业务；两板须使用相同协议版本。
	constexpr uint8_t kMaxPayload = wire::kMaxPayload;

	/** @brief 双板链路的最近接收时间、计数及在线标志。 */
	struct LinkStatus
	{
		uint32_t last_rx_tick;//最近有效链路接收tick；不是另一块板的时钟。
		uint16_t rx_frames;//接收帧计数
		uint16_t tx_frames;//发送帧计数
		bool  online;//在线；应用仍须检查自己的命令新鲜度，不以心跳替代命令有效性。
		uint32_t rx_dropped{}; // ISR队列满或过期帧数量。
		uint32_t tx_failures{}; // 累计发送入队失败，不表示电调执行失败。
	};

	/**
	 * @brief 注册共享CAN链路与工作任务，不独占、不停止同路其他模块。
	 * @return 启用过滤器、中断、CAN总线以及任务后，该值为真.
	 */
	bool Start();
	// 调度器启动前调用；成功仅表示本地注册完成，不代表另一块板已经在线。

	/**
	 * @brief 将本地消息中心主题中的新值发送至另一块板卡.
	 * @param id 共享主题ID，取值范围1~255；0保留用于心跳信号.
	 * @param name 本地消息中心主题名称；注册期间必须保持有效.
	 * @param length 固定业务消息长度，1至60字节；经典后端分片，FD后端单帧。
	 * @return 如果该主题在调度器启动前已注册，则为true.
	 */
	bool RegisterTxTopic(uint8_t id, char* name, uint8_t length);

	/**
	 * @brief 将另一块板卡收到的数据发布到本地消息中心主题。
	 * @param id 与对端 RegisterTxTopic 一致的主题 ID。
	 * @param name 本地消息中心主题名称，注册期间需保持有效。
	 * @param length 与对端一致的固定消息字节数。
	 * @return 调度器运行前成功注册时为 true。
	 */
	bool RegisterRxTopic(uint8_t id, char* name, uint8_t length);
} // namespace interboard
