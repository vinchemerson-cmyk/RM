/**
 * @file interboard.h
 * @brief 声明双板链路启动、主题注册和链路状态接口。
 */

#pragma once

#include <cstdint>

namespace interboard {

constexpr uint8_t kMaxPayload = 48;

    /** @brief 双板链路的最近接收时间、计数及在线标志。 */
struct LinkStatus {
    uint32_t last_rx_tick;
    uint16_t rx_frames;
    uint16_t tx_frames;
    uint8_t online;
};

/**
 * @brief 配置并启动专用CAN链路与工作任务.
 * @return 启用过滤器、中断、CAN总线以及任务后，该值为真.
 */
bool Start();

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
