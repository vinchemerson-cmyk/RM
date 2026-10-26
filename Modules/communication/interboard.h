/**
 * @file interboard.h
 * @brief 声明双板链路启动、主题注册和链路状态接口。
 */

#pragma once

#include <cstdint>

namespace interboard
{
	//消息最大长度
	// TODO[CANFD-07 有效负载]：区分“业务消息长度”与“CAN FD帧最大64字节”。
	// 当前最大48字节消息可继续保留并改成单FD帧；若保留2字节协议头，64字节帧最多承载62字节业务。
	// 不能只把kMaxPayload改成64而仍附加2字节头；更大消息需另设计分片或调整协议头放置方式。
	constexpr uint8_t kMaxPayload = 48;

	/** @brief 双板链路的最近接收时间、计数及在线标志。 */
	struct LinkStatus
	{
		uint32_t last_rx_tick;//上一次发送时刻
		uint16_t rx_frames;//接收帧计数
		uint16_t tx_frames;//发送帧计数
		bool  online;//在线
	};

	/**
	 * @brief 配置并启动专用CAN链路与工作任务.
	 * @return 启用过滤器、中断、CAN总线以及任务后，该值为真.
	 */
	bool Start();
	// TODO[CANFD-05 共享启动]：Start应成为共享FD总线的普通模块注册，不再独占控制器或自行Start/Stop。
	// 后端过滤器、帧路由和生命周期统一管理，避免影响同路DM电机；函数成功仍不代表链路已在线。

	/**
	 * @brief 将本地消息中心主题中的新值发送至另一块板卡.
	 * @param id 共享主题ID，取值范围1~255；0保留用于心跳信号.
	 * @param name 本地消息中心主题名称；注册期间必须保持有效.
	 * @param length 固定消息长度，1至48字节.
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
