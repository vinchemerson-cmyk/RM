/**
 * @file can.h
 * @brief 定义 CAN 设备封装及专用接收回调接口。
 */

//
// Created by 19923 on 2026/9/25.
//

#ifndef INFANTRY_H723_CAN_H
#define INFANTRY_H723_CAN_H
#include <array>
#include <sys/_stdint.h>

#include <cstring>
#include "canConfig.h"
#include "fdcan.h"
#include "main.h"

// 双板通信等模块可以独占一条 FDCAN 控制器，并直接处理其接收 FIFO。
using CanDedicatedRxHandler = void (*)(FDCAN_HandleTypeDef*, uint32_t);
/** @brief 注册专用接收处理器；传入空指针可清除绑定。 */
void CAN_SetDedicatedRxHandler(FDCAN_HandleTypeDef* handle, CanDedicatedRxHandler handler);
/** @brief 若句柄匹配，则转发 FIFO 接收事件。 @return 是否已由专用处理器接管。 */
bool CAN_DispatchDedicatedRx(FDCAN_HandleTypeDef* handle, uint32_t fifo);

/** @brief 按经典 CAN 或 CAN FD 配置管理静态实例表和 HAL 收发。 */
template <typename canType>
class CAN_
{
    private:
        using canAttributeType  = typename canType::Attribute;
        using canConfigType     = typename canType::Config;
        using canDependencyType = typename canType::Dependency;
        using canInputType      = typename canType::Input;
        using canOutputType     = typename canType::Output;

        struct Instance
        {
            canAttributeType  attribute{};
            canDependencyType dependency{};
            canOutputType     output{};
        };

        // 每路 FDCAN 的静态注册表, FilterIndex 直接对应表下标.
        static inline std::array<Instance, canType::MAX_FILTER_AMOUNT> _can1_instance_table{};
        static inline std::array<Instance, canType::MAX_FILTER_AMOUNT> _can2_instance_table{};
        static inline std::array<Instance, canType::MAX_FILTER_AMOUNT> _can3_instance_table{};
        static inline std::array<uint8_t, 3>                           _can_instance_amount{}; // 每路 CAN 已注册实例数.

        /**
     * @brief 将实际字节长度转换成合法 DLC 编码.
     * @param data_length 实际数据长度.
     * @return 对应 HAL DLC 编码.
     */
        static uint32_t _dlcDataLengthToCode(const uint8_t data_length)
        {
            // Classic CAN 的 0 至 8 字节长度与 HAL DLC 编码数值相同.
            if (data_length <= 8)
                return data_length;

            // FDCAN 的非线性 DLC 只能表示以下长度.
            // 9 至 12 字节会按 12 字节帧发送, 其余区间同理.
            if (data_length <= 12)
                return FDCAN_DLC_BYTES_12;
            if (data_length <= 16)
                return FDCAN_DLC_BYTES_16;
            if (data_length <= 20)
                return FDCAN_DLC_BYTES_20;
            if (data_length <= 24)
                return FDCAN_DLC_BYTES_24;
            if (data_length <= 32)
                return FDCAN_DLC_BYTES_32;
            if (data_length <= 48)
                return FDCAN_DLC_BYTES_48;
            if (data_length <= 64)
                return FDCAN_DLC_BYTES_64;

            // 发送长度超出 FDCAN 单帧上限, 禁止生成非法 DLC.
            Error_Handler();
            return FDCAN_DLC_BYTES_0;
        }

        /**
     * @brief 将 DLC 编码转换成真实数据长度.
     * @param data_length_code HAL DLC 编码.
     * @return 对应实际数据长度.
     */
        static uint8_t _dlcCodeToDataLength(const uint32_t data_length_code)
        {
            switch (data_length_code)
            {
                case FDCAN_DLC_BYTES_0:
                    return 0;
                case FDCAN_DLC_BYTES_1:
                    return 1;
                case FDCAN_DLC_BYTES_2:
                    return 2;
                case FDCAN_DLC_BYTES_3:
                    return 3;
                case FDCAN_DLC_BYTES_4:
                    return 4;
                case FDCAN_DLC_BYTES_5:
                    return 5;
                case FDCAN_DLC_BYTES_6:
                    return 6;
                case FDCAN_DLC_BYTES_7:
                    return 7;
                case FDCAN_DLC_BYTES_8:
                    return 8;
                case FDCAN_DLC_BYTES_12:
                    return 12;
                case FDCAN_DLC_BYTES_16:
                    return 16;
                case FDCAN_DLC_BYTES_20:
                    return 20;
                case FDCAN_DLC_BYTES_24:
                    return 24;
                case FDCAN_DLC_BYTES_32:
                    return 32;
                case FDCAN_DLC_BYTES_48:
                    return 48;
                case FDCAN_DLC_BYTES_64:
                    return 64;

                // HAL 返回未定义 DLC 时, 不能继续按未知长度复制数据.
                default:
                    Error_Handler();
                    return 0;
            }
        }

        /**
     * @brief 通过 Attribute 指针查找对应的完整运行实例.
     * @param attribute_instance 已注册 Attribute 指针.
     * @return 对应的静态运行实例, 查找失败返回 nullptr.
     */
        static const Instance* _getInstance(const canAttributeType* attribute_instance)
        {
            // 仅遍历实际已注册数量, 不读取注册表空槽位.
            for (uint8_t i = 0; i < _can_instance_amount[0]; i++)
            {
                if (&_can1_instance_table[i].attribute == attribute_instance)
                    return &_can1_instance_table[i];
            }

            for (uint8_t i = 0; i < _can_instance_amount[1]; i++)
            {
                if (&_can2_instance_table[i].attribute == attribute_instance)
                    return &_can2_instance_table[i];
            }

            for (uint8_t i = 0; i < _can_instance_amount[2]; i++)
            {
                if (&_can3_instance_table[i].attribute == attribute_instance)
                    return &_can3_instance_table[i];
            }

            // 外部传入非本 BSP 注册的 Attribute 指针.
            Error_Handler();
            return nullptr;
        }

    public:
        /**
         * @brief 中断入口, 根据 hfdcan 选注册表并处理对应 FIFO.
         * @param can_handle_instance 当前哪个 CAN 进入了中断.
         * @param fifox 数据进入了哪个 FIFO.
         * @note can.cpp 根据 CubeMX 的 FrameFormat 分发到 classicCan 或 Fdcan.
         *       保持"更新 Output, 执行上层回调"的既有顺序.
         */
        static void fifoxCallBack(bspCanHandleTypeDef* can_handle_instance, const uint32_t fifox)
        {
            std::array<Instance, canType::MAX_FILTER_AMOUNT>* instance_table{};
            uint8_t                                           instance_amount{};

            // 按触发中断的 FDCAN 句柄直接选择对应的静态注册表.
            if (can_handle_instance == &hfdcan1)
            {
                instance_table  = &_can1_instance_table;
                instance_amount = _can_instance_amount[0];
            } else if (can_handle_instance == &hfdcan2)
            {
                instance_table  = &_can2_instance_table;
                instance_amount = _can_instance_amount[1];
            } else if (can_handle_instance == &hfdcan3)
            {
                instance_table  = &_can3_instance_table;
                instance_amount = _can_instance_amount[2];
            } else
                return;

            // 接收缓冲区仅在本函数内使用. HAL 会先将硬件 Message RAM 数据复制到这里.
            std::array<uint8_t, canType::DATA_SIZE> rx_data_buffer{};
            bspCanRxHeaderTypeDef                   can_rx_header_instance{};

            if (HAL_FDCAN_GetRxMessage(can_handle_instance,
                                       fifox,
                                       &can_rx_header_instance,
                                       rx_data_buffer.data()) != HAL_OK)
                return;

            // FilterIndex 与同一路静态注册表下标一一对应.
            const uint32_t filter_index = can_rx_header_instance.FilterIndex;
            if (filter_index >= instance_amount)
                return;

            auto&         instance = (*instance_table)[filter_index];
            const uint8_t rx_len   = _dlcCodeToDataLength(can_rx_header_instance.DataLength);

            // 防止帧格式和模板类型不匹配时向 Output 缓冲区越界复制.
            if (rx_len > canType::DATA_SIZE)
                Error_Handler();

            // 先更新 Output. 上层回调可通过 Receive() 读取本次完整接收结果.
            instance.output.rx_len = rx_len;
            std::memcpy(instance.output.rx_buffer.data(), rx_data_buffer.data(), rx_len);

            // 保持原有顺序: 数据更新完成后再执行上层回调.
            if (instance.attribute.can_callback_func != nullptr)
                instance.attribute.can_callback_func(&instance.attribute);
        }
};
#endif //INFANTRY_H723_CAN_H
