/**
 * @file can.h
 * @brief 经典CAN/FD共享注册与收发。每个物理控制器只允许匹配其模式的一种后端。
 * 同路DM和板间消息均走CAN_<Fdcan>，统一分配过滤器；不再有整路独占接收。
 * 注册仅限调度器启动前，实例/输出缓冲区静态存储；返回nullptr表示配置或HAL操作失败。
 */
#ifndef INFANTRY_H723_CAN_H
#define INFANTRY_H723_CAN_H
#include <array>
#include <cstdint>
#include <cstring>
#include "canConfig.h"
#include "can_frame.h"
#include "fdcan.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

template <typename canType>
class CAN_
{
    using canAttributeType = typename canType::Attribute;
    using canConfigType = typename canType::Config;
    using canInputType = typename canType::Input;

public:
    struct Instance
    {
        canAttributeType attribute{};
        typename canType::Dependency dependency{};
        typename canType::Output output{};
        bspCanTxHeaderTypeDef tx_header{};
        uint32_t feedback_tick{};

        HAL_StatusTypeDef Transmit(const canInputType& input)
        { return CAN_::TransmitInstance(*this, input); }
    };

private:
    using Table = std::array<Instance, canType::MAX_FILTER_AMOUNT>;
    static inline std::array<Table, 3> tables_{};
    static inline std::array<uint8_t, 3> counts_{};

    static int BusIndex(bspCanHandleTypeDef* handle)
    {
        if (handle == &hfdcan1) return 0;
        if (handle == &hfdcan2) return 1;
        if (handle == &hfdcan3) return 2;
        return -1;
    }

    static bool ModeMatches(bspCanHandleTypeDef* handle)
    {
        if (handle == nullptr) return false;
        if constexpr (canType::IS_CLASSIC)
            return handle->Init.FrameFormat == FDCAN_FRAME_CLASSIC;
        return handle->Init.FrameFormat == FDCAN_FRAME_FD_NO_BRS ||
               handle->Init.FrameFormat == FDCAN_FRAME_FD_BRS;
    }

public:
    /** @brief 注册标准数据帧ID。重复接收ID/后端模式不符/容量不足均拒绝，不占用新槽。 */
    static Instance* Register(const canConfigType& config)
    {
        const int bus = BusIndex(config.can_handle);
        if (bus < 0 || !ModeMatches(config.can_handle) || config.can_callback_func == nullptr ||
            config.master_id > 0x7FFU || config.can_id > 0x7FFU ||
            (config.use_fifo != canConfig::fifo::CAN_RX_FIFO && config.use_fifo != canConfig::fifo::CAN_RX_FIF1) ||
            osKernelGetState() == osKernelRunning)
            return nullptr;
        auto* handle = config.can_handle;
        auto& count = counts_[bus];
        auto& table = tables_[bus];
        const auto rxSize = config.use_fifo == canConfig::fifo::CAN_RX_FIFO
            ? handle->Init.RxFifo0ElmtSize : handle->Init.RxFifo1ElmtSize;
        const auto rxCount = config.use_fifo == canConfig::fifo::CAN_RX_FIFO
            ? handle->Init.RxFifo0ElmtsNbr : handle->Init.RxFifo1ElmtsNbr;
        const uint32_t required = canType::IS_CLASSIC ? FDCAN_DATA_BYTES_8 : FDCAN_DATA_BYTES_64;
        if (count >= table.size() || count >= handle->Init.StdFiltersNbr ||
            handle->Init.TxElmtSize < required || rxSize < required || rxCount == 0 ||
            handle->Init.TxFifoQueueElmtsNbr == 0 || config.tx_reserved_slots >= handle->Init.TxFifoQueueElmtsNbr)
            return nullptr;
        for (unsigned i = 0; i < count; ++i)
            if (table[i].attribute.can_id == config.can_id) return nullptr;
        const auto state = HAL_FDCAN_GetState(handle);
        if ((count == 0 && state != HAL_FDCAN_STATE_READY) ||
            (state != HAL_FDCAN_STATE_READY && state != HAL_FDCAN_STATE_BUSY))
            return nullptr;

        // 首个注册者配置全局拒收规则。之后只追加各自过滤器，不重启/停止共享控制器。
        if (count == 0 && HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT,
                                  FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
            return nullptr;
        const unsigned slot = count;
        auto& instance = table[slot];
        instance = {};
        instance.attribute.master_id = config.master_id;
        instance.attribute.can_id = config.can_id;
        instance.attribute.use_fifo = config.use_fifo;
        instance.attribute.can_callback_func = config.can_callback_func;
        instance.attribute.module_handle = config.module_handle;
        instance.attribute.tx_reserved_slots = config.tx_reserved_slots;
        instance.dependency.hcan = handle;
        instance.dependency.tx_header = &instance.tx_header;
        bspCanFilterTypeDef filter{};
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = slot;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = config.use_fifo == canConfig::fifo::CAN_RX_FIFO
            ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
        filter.FilterID1 = config.can_id;
        filter.FilterID2 = 0x7FFU;
        if (HAL_FDCAN_ConfigFilter(handle, &filter) != HAL_OK)
        { instance = {}; return nullptr; }

        // 中断开放前先发布实例，保证FilterIndex与实例槽一致；失败则禁用新过滤器并回滚槽位。
        ++count;
        const auto notification = config.use_fifo == canConfig::fifo::CAN_RX_FIFO
            ? FDCAN_IT_RX_FIFO0_NEW_MESSAGE : FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
        if (HAL_FDCAN_ActivateNotification(handle, notification, 0) != HAL_OK ||
            (state == HAL_FDCAN_STATE_READY && HAL_FDCAN_Start(handle) != HAL_OK))
        {
            filter.FilterConfig = FDCAN_FILTER_DISABLE;
            (void)HAL_FDCAN_ConfigFilter(handle, &filter);
            --count;
            instance = {};
            return nullptr;
        }
        return &instance;
    }

    /** @brief tx_len是实际字节数，缓冲区容量不是每帧固定长度；HAL在调用期间复制至Message RAM。 */
    static HAL_StatusTypeDef TransmitInstance(Instance& instance, const canInputType& input)
    {
        if (!ModeMatches(instance.dependency.hcan) || input.tx_len == 0 || input.tx_len > canType::DATA_SIZE)
            return HAL_ERROR;
        auto* handle = instance.dependency.hcan;
        bspCanTxHeaderTypeDef header{};
        header.Identifier = instance.attribute.master_id;
        header.IdType = FDCAN_STANDARD_ID;
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = canFrame::DlcForLength(input.tx_len);
        header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        header.BitRateSwitch = !canType::IS_CLASSIC && handle->Init.FrameFormat == FDCAN_FRAME_FD_BRS
            ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
        header.FDFormat = canType::FD_FORMAT;
        header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        // HAL读取补齐至合法DLC长度的数据，统一清零尾部，不发送调用方缓冲区的旧字节。
        std::array<uint8_t, canType::DATA_SIZE> payload{};
        std::memcpy(payload.data(), input.tx_buffer.data(), input.tx_len);
        // HAL的“读TX put-index -> 写RAM -> 提交”没有互斥；共享总线多任务必须串行这一小段。
        // 临界区只覆盖最多64字节入队，绝不等待FIFO空位；本接口限任务上下文调用。
        taskENTER_CRITICAL();
        HAL_StatusTypeDef result = HAL_ERROR;
        if (HAL_FDCAN_GetTxFifoFreeLevel(handle) > instance.attribute.tx_reserved_slots)
        {
            instance.tx_header = header;
            result = HAL_FDCAN_AddMessageToTxFifoQ(handle, &header, payload.data());
        }
        taskEXIT_CRITICAL();
        return result;
    }

    /** @brief 每次从FIFO取一帧，再按标准ID/FilterIndex/帧格式路由，不按8/64字节区分模块。 */
    static void fifoxCallBack(bspCanHandleTypeDef* handle, uint32_t fifo)
    {
        // HAL按实际DLC复制；即使收到非预期长帧，也必须先有64字节接收空间再做校验。
        std::array<uint8_t, 64> payload{};
        bspCanRxHeaderTypeDef header{};
        if (HAL_FDCAN_GetRxMessage(handle, fifo, &header, payload.data()) != HAL_OK) return;
        const int bus = BusIndex(handle);
        const auto length = canFrame::LengthForDlc(header.DataLength);
        if (bus < 0 || !ModeMatches(handle) || header.FilterIndex >= counts_[bus] ||
            header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != canType::FD_FORMAT || length > canType::DATA_SIZE)
            return;
        auto& instance = tables_[bus][header.FilterIndex];
        if (header.Identifier != instance.attribute.can_id ||
            static_cast<uint32_t>(instance.attribute.use_fifo) != fifo)
            return;
        instance.output.rx_len = length;
        instance.output.rx_buffer.fill(0);
        std::memcpy(instance.output.rx_buffer.data(), payload.data(), length);
        if (instance.attribute.can_callback_func != nullptr)
            instance.attribute.can_callback_func(&instance.attribute);
    }
};
static_assert(FDCAN_DLC_BYTES_64 == 15U, "DLC helper must match this HAL's unshifted DLC constants");
#endif
