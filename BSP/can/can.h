/**
 * @file can.h
 * @brief 定义 CAN 设备封装及专用接收回调接口。
 *
 * 提供两种接收路径:
 *   1. 常规模块 (电机等): CAN_<T>::Register() 为每个 rx_id 配置一个
 *      标准帧过滤器, FilterIndex 与注册表下标一一对应; 接收中断把
 *      整帧填进实例 output 后调用上层 can_callback_func。
 *   2. 独占控制器 (双板 interboard): CAN_SetDedicatedRxHandler() 把
 *      整个 FDCAN 控制器绑定给一个回调, HAL 回调优先派发给它,
 *      不占用过滤器表, 由该模块自行取空 FIFO。
 *
 * 模板按 classicCan (8 字节经典帧) 与 Fdcan (64 字节可变帧) 两个
 * traits 各实例化一份静态注册表; 显式实例化在 can.cpp。控制器首次
 * 注册时启动 FDCAN, 后续注册只追加过滤器。注册表与实例缓冲区均为
 * 静态存储, 中断与任务共享, 注册只允许在启动阶段进行。
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

    public:
        /**
         * @brief 单个 CAN 注册实例的持久状态.
         * @note 实例表本身保持私有, 上层只持有 Register() 返回的实例指针.
         */
        struct Instance
        {
            canAttributeType  attribute{};
            canDependencyType dependency{};
            canOutputType     output{};

            // 发送帧头随实例存放, 使同一控制器上的多个实例可以并行填帧.
            bspCanTxHeaderTypeDef tx_header{};

            // 最近一次成功接收反馈的 RTOS tick, 由上层回调按需更新, 用于掉线判定.
            uint32_t feedback_tick{};

            /**
             * @brief 用本实例的 tx_id 发送一帧, 转发到所属 CAN_ 模板的 Transmit().
             * @param input_instance 本次发送的数据.
             * @return HAL_OK 表示已放入发送 FIFO.
             */
            HAL_StatusTypeDef Transmit(const canInputType& input_instance)
            {
                return CAN_::TransmitInstance(*this, input_instance);
            }
        };

    private:
        // 每路 FDCAN 的静态注册表, FilterIndex 直接对应表下标.
        static inline std::array<Instance, canType::MAX_FILTER_AMOUNT> _can1_instance_table{};
        static inline std::array<Instance, canType::MAX_FILTER_AMOUNT> _can2_instance_table{};
        static inline std::array<Instance, canType::MAX_FILTER_AMOUNT> _can3_instance_table{};
        static inline std::array<uint8_t, 3>                           _can_instance_amount{}; // 每路 CAN 已注册实例数.
        // 每路 CAN 已占用的过滤器个数; 与实例数分开, 因为同一过滤器可以命中多个实例.
        static inline std::array<uint8_t, 3>                           _can_filter_amount{};

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
         * @brief   注册一个 CAN 实例, 配置过滤器并使能该控制器的接收中断.
         * @param   config_instance 用户配置, 包含 can_handle, tx_id, rx_id 和上层回调.
         * @return  指向已注册实例的指针, 上层可长期保存并读取其 output 与 attribute.
         * @note    FilterIndex 与实例表下标一一对应, 因此每注册一个实例就占用一个过滤器.
         * @note    同一控制器的第一个实例会启动 FDCAN; 之后每个实例只追加自己的过滤器.
         */
        static Instance* Register(const canConfigType& config_instance)
        {
            // CAN 句柄和上层回调是收发分发的必要依赖.
            if (config_instance.can_handle == nullptr || config_instance.can_callback_func == nullptr)
                Error_Handler();

            // FIFO0 与 FIFO1 的过滤器目标值不同, 不落在枚举内说明调用方传入了非法值.
            if (config_instance.use_fifo != canConfig::fifo::CAN_RX_FIFO &&
                config_instance.use_fifo != canConfig::fifo::CAN_RX_FIF1)
                Error_Handler();

            bspCanHandleTypeDef* can_handle_instance = config_instance.can_handle;

            // 按句柄选择注册表, 并记录该表当前已用数量.
            std::array<Instance, canType::MAX_FILTER_AMOUNT>* instance_table{};
            uint8_t*                                         instance_amount{};
            uint8_t*                                         filter_amount{};
            if (can_handle_instance == &hfdcan1)
            {
                instance_table  = &_can1_instance_table;
                instance_amount = &_can_instance_amount[0];
                filter_amount   = &_can_filter_amount[0];
            } else if (can_handle_instance == &hfdcan2)
            {
                instance_table  = &_can2_instance_table;
                instance_amount = &_can_instance_amount[1];
                filter_amount   = &_can_filter_amount[1];
            } else if (can_handle_instance == &hfdcan3)
            {
                instance_table  = &_can3_instance_table;
                instance_amount = &_can_instance_amount[2];
                filter_amount   = &_can_filter_amount[2];
            } else
            {
                Error_Handler();
                return nullptr;
            }

            // 注册表已满时禁止继续写入, 防止实例表越界.
            if (*instance_amount >= canType::MAX_FILTER_AMOUNT)
                Error_Handler();

            // 先占用槽位, 使中断在此之后的任意时刻都能查到本实例.
            Instance& instance = (*instance_table)[(*instance_amount)++];

            instance.attribute.tx_id               = config_instance.tx_id;
            instance.attribute.rx_id               = config_instance.rx_id;
            instance.attribute.use_fifo            = config_instance.use_fifo;
            instance.attribute.can_callback_func   = config_instance.can_callback_func;
            instance.attribute.module_handle       = config_instance.module_handle;
            instance.dependency.hcan               = can_handle_instance;
            instance.dependency.tx_header          = &instance.tx_header;

            // 只让本实例关心的反馈 ID 进入指定 FIFO, 其余标准帧在过滤器层被丢弃.
            const uint32_t filter_fifo =
                (config_instance.use_fifo == canConfig::fifo::CAN_RX_FIF1) ? FDCAN_FILTER_TO_RXFIFO1
                                                                          : FDCAN_FILTER_TO_RXFIFO0;

            bspCanFilterTypeDef filter{};
            // 每新增一个实例就占用一个新的过滤器槽位: FilterIndex 必须与 fifoxCallBack()
            // 使用的注册表下标一致, 否则反馈会被投递到错误的实例.
            // 同一过滤器命中多个实例时无法定位回调对象, 因此这里不做过滤器复用.
            //
            // 上限取 CubeMX 配置的实际过滤器数量, 而非模板的 MAX_FILTER_AMOUNT:
            // HAL_FDCAN_ConfigFilter() 对 FilterIndex >= StdFiltersNbr 只做 assert_param,
            // Release 构建下会静默写入 Message RAM 之外, 因此这里必须显式拦住.
            const uint8_t filter_limit =
                static_cast<uint8_t>(config_instance.can_handle->Init.StdFiltersNbr);
            if (*filter_amount >= filter_limit || *filter_amount >= canType::MAX_FILTER_AMOUNT)
                Error_Handler();
            filter.IdType       = FDCAN_STANDARD_ID;
            filter.FilterIndex  = static_cast<uint32_t>(*filter_amount);
            filter.FilterType   = FDCAN_FILTER_MASK;
            filter.FilterConfig = filter_fifo;
            filter.FilterID1    = config_instance.rx_id;
            filter.FilterID2    = 0x7FFU; // 标准帧 11 位掩码全 1, 表示精确匹配 rx_id.
            if (HAL_FDCAN_ConfigFilter(can_handle_instance, &filter) != HAL_OK)
                Error_Handler();
            // 过滤器写入成功后才占用槽位, 保证 FilterIndex 与注册表下标始终一一对应.
            ++(*filter_amount);

            // 未命中任何过滤器的标准帧直接丢弃, 不再进入 FIFO.
            if (HAL_FDCAN_ConfigGlobalFilter(can_handle_instance, FDCAN_REJECT, FDCAN_REJECT,
                                            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
                Error_Handler();

            const uint32_t notification = (config_instance.use_fifo == canConfig::fifo::CAN_RX_FIF1)
                                              ? FDCAN_IT_RX_FIFO1_NEW_MESSAGE
                                              : FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
            if (HAL_FDCAN_ActivateNotification(can_handle_instance, notification, 0) != HAL_OK)
                Error_Handler();

            // HAL_FDCAN_Start() 只接受 READY 状态, 对已启动的控制器返回 HAL_ERROR,
            // 因此仅当控制器尚未启动时才调用.
            if (HAL_FDCAN_GetState(can_handle_instance) != HAL_FDCAN_STATE_BUSY)
            {
                if (HAL_FDCAN_Start(can_handle_instance) != HAL_OK)
                    Error_Handler();
            }

            // 暴露本实例的 tx_id 供上层按指令帧改写.
            return &instance;
        }

        /**
         * @brief   使用给定实例的 tx_id 发送一帧数据.
         * @param   instance 目标实例, 即 Register() 返回的那个注册表项.
         * @param   input_instance 本次发送的数据, tx_len 不得超过 canType::DATA_SIZE.
         * @return  HAL_OK 表示已放入发送 FIFO; 其余返回值表示本次未发出.
         * @note    每实例持有独立帧头, 因此同一控制器上的多个实例可各自填帧.
         * @note    发送 FIFO 满时返回 HAL_ERROR 而不阻塞, 调用方可等下一周期重发.
         */
        static HAL_StatusTypeDef TransmitInstance(Instance& instance, const canInputType& input_instance)
        {
            if (input_instance.tx_len == 0 || input_instance.tx_len > canType::DATA_SIZE ||
                instance.dependency.hcan == nullptr)
                return HAL_ERROR;

            instance.tx_header.Identifier          = instance.attribute.tx_id;
            instance.tx_header.IdType              = FDCAN_STANDARD_ID;
            instance.tx_header.TxFrameType         = FDCAN_DATA_FRAME;
            instance.tx_header.DataLength          = _dlcDataLengthToCode(input_instance.tx_len);
            instance.tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
            instance.tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
            instance.tx_header.FDFormat            = canType::FD_FORMAT;
            instance.tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
            instance.tx_header.MessageMarker       = 0;

            return HAL_FDCAN_AddMessageToTxFifoQ(instance.dependency.hcan,
                                                 &instance.tx_header,
                                                 input_instance.tx_buffer.data());
        }

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
