/**
 * @file interboard.cpp
 * @brief 共享CAN上的消息中心桥接：经典CAN沿用分片，CAN FD采用带版本/真实长度的单帧。
 * 两板需使用同版本协议；链路在线不代表某个控制指令仍有效，执行器须检查业务时间戳。
 */
#include "interboard.h"
#include "cmsis_os2.h"
#include "RobotConfig.hpp"
#include "can/can.h"
#include "message_center/message_center.h"
#include <algorithm>
#include <cstring>

namespace interboard
{
namespace
{
    constexpr uint32_t kHeartbeatMs = 100U, kOfflineMs = 300U, kAssemblyTimeoutMs = 50U;
    constexpr uint32_t kPollMs = 2U;
    constexpr size_t kTopicCapacity = 8U;
    constexpr unsigned kTxBudget = 2U, kMotorReservedSlots = 2U;
#if INFANTRY_INTERBOARD_FDCAN == 1
    FDCAN_HandleTypeDef* const canHandle = &hfdcan1;
#elif INFANTRY_INTERBOARD_FDCAN == 2
    FDCAN_HandleTypeDef* const canHandle = &hfdcan2;
#elif INFANTRY_INTERBOARD_FDCAN == 3
    FDCAN_HandleTypeDef* const canHandle = &hfdcan3;
#else
#error "INFANTRY_INTERBOARD_FDCAN must be 1, 2 or 3"
#endif
#if INFANTRY_BOARD_GIMBAL
    constexpr uint32_t kTxId = 0x301U, kRxId = 0x302U;
#else
    constexpr uint32_t kTxId = 0x302U, kRxId = 0x301U;
#endif

    using ClassicBus = CAN_<classicCan>;
    using FdBus = CAN_<Fdcan>;
    ClassicBus::Instance* classicInstance{};
    FdBus::Instance* fdInstance{};
    bool fdMode{}, startAttempted{};
    osThreadId_t taskHandle{};
    osMessageQueueId_t rxQueue{};
    volatile uint32_t rxDrops{};
    uint32_t staleDrops{}; // 仅任务写入，与ISR队列溢出计数分开，避免跨上下文自增丢失。
    LinkStatus status{};
    Publisher_t* statusPublisher{};
    char statusTopicName[] = "interboard/link";
    constexpr osThreadAttr_t taskAttrs{
        .name = "InterboardTask", .stack_size = 2048U, .priority = osPriorityAboveNormal,
    };

    struct Topic
    {
        uint8_t id{}, length{}, tx_sequence{};
        Publisher_t* publisher{};
        Subscriber_t* subscriber{};
        wire::ClassicAssembly assembly{};
        std::array<uint8_t, kMaxPayload> pending_value{};
        uint32_t pending_tick{};
        uint8_t pending_sequence{}, pending_offset{};
        bool pending{};
    };
    std::array<Topic, kTopicCapacity> txTopics{}, rxTopics{};
    size_t txCount{}, rxCount{}, nextTxTopic{};

    uint32_t Ticks(uint32_t milliseconds)
    {
        return static_cast<uint32_t>((static_cast<uint64_t>(milliseconds) * osKernelGetTickFreq() + 999U) / 1000U);
    }
    bool Expired(uint32_t now, uint32_t sample, uint32_t milliseconds)
    { return now - sample >= Ticks(milliseconds); }

    /** @brief ISR只复制并排队，不操作消息中心；统一CAN后端已完成取帧、ID及帧格式校验。 */
    template <class Instance> void QueueRx(Instance* instance, bool fd)
    {
        if (instance == nullptr || rxQueue == nullptr || instance->output.rx_len > 64U) return;
        wire::Frame frame{};
        frame.fd = fd;
        frame.length = instance->output.rx_len;
        frame.received_tick = osKernelGetTickCount();
        std::copy_n(instance->output.rx_buffer.begin(), frame.length, frame.data.begin());
        if (osMessageQueuePut(rxQueue, &frame, 0U, 0U) != osOK) ++rxDrops;
    }
    void OnClassicRx(classicCanType*) { QueueRx(classicInstance, false); }
    void OnFdRx(FdcanType*) { QueueRx(fdInstance, true); }

    bool CanSend()
    {
        // 限制板间突发，并保留FIFO余量给电机；不代表具备硬实时带宽或比赛功率保证。
        return HAL_FDCAN_GetTxFifoFreeLevel(canHandle) > kMotorReservedSlots;
    }

    bool Send(const wire::Frame& frame)
    {
        if (!CanSend()) return false;
        HAL_StatusTypeDef result = HAL_ERROR;
        if (fdMode && fdInstance != nullptr && frame.fd)
        {
            Fdcan::Input input{};
            input.tx_len = frame.length;
            std::copy_n(frame.data.begin(), frame.length, input.tx_buffer.begin());
            result = fdInstance->Transmit(input);
        }
        else if (!fdMode && classicInstance != nullptr && !frame.fd && frame.length <= 8U)
        {
            classicCan::Input input{};
            input.tx_len = frame.length;
            std::copy_n(frame.data.begin(), frame.length, input.tx_buffer.begin());
            result = classicInstance->Transmit(input);
        }
        if (result != HAL_OK) { ++status.tx_failures; return false; }
        ++status.tx_frames;
        return true;
    }

    void MarkOnline(uint32_t sampleTick, bool heartbeat)
    {
        const bool wasOffline = !status.online;
        status.last_rx_tick = sampleTick;
        status.online = true;
        status.rx_dropped = rxDrops + staleDrops;
        ++status.rx_frames;
        if (wasOffline || heartbeat) PubPushMessage(statusPublisher, &status);
    }

    void Process(const wire::Frame& frame)
    {
        const uint32_t now = osKernelGetTickCount();
        // 不让任务队列中积压的旧帧刷新在线状态；使用ISR到达时刻，而不是处理时刻。
        if (frame.fd != fdMode || Expired(now, frame.received_tick, kOfflineMs)) { ++staleDrops; return; }
        if (frame.fd)
        {
            wire::PayloadView payload{};
            if (!wire::DecodeFd(frame, payload)) return;
            if (payload.topic == 0) { MarkOnline(frame.received_tick, true); return; }
            for (size_t i = 0; i < rxCount; ++i)
            {
                auto& topic = rxTopics[i];
                if (topic.id != payload.topic) continue;
                if (payload.length != topic.length) return;
                MarkOnline(frame.received_tick, false);
                // PubPushMessage立即复制；payload指向本地frame，无需延长其生命周期。
                PubPushMessage(topic.publisher, const_cast<uint8_t*>(payload.bytes));
                return;
            }
        }
        else
        {
            if (wire::IsClassicHeartbeat(frame)) { MarkOnline(frame.received_tick, true); return; }
            if (frame.length < 3U || frame.length > 8U || frame.data[0] == 0U) return;
            for (size_t i = 0; i < rxCount; ++i)
            {
                auto& topic = rxTopics[i];
                if (topic.id != frame.data[0]) continue;
                bool complete = false;
                if (!topic.assembly.Accept(frame, topic.length, Ticks(kAssemblyTimeoutMs), complete)) return;
                MarkOnline(frame.received_tick, false);
                if (complete) PubPushMessage(topic.publisher, topic.assembly.bytes.data());
                return;
            }
        }
    }

    void TransmitTopics(uint32_t now, unsigned& budget)
    {
        // 轮转访问，防止高频主题独占；经典长消息分多轮发送，不要求FIFO一次装下所有分片。
        for (size_t visit = 0; visit < txCount && budget != 0U; ++visit)
        {
            auto& topic = txTopics[nextTxTopic++ % txCount];
            if (topic.pending && Expired(now, topic.pending_tick, kAssemblyTimeoutMs)) topic.pending = false;
            if (!topic.pending)
            {
                if (!SubGetMessage(topic.subscriber, topic.pending_value.data())) continue;
                topic.pending = true;
                topic.pending_tick = now;
                topic.pending_offset = 0U;
                topic.pending_sequence = topic.tx_sequence++;
            }
            while (topic.pending && budget != 0U && CanSend())
            {
                wire::Frame frame{};
                const uint8_t remaining = topic.length - topic.pending_offset;
                const uint8_t count = fdMode ? topic.length : std::min(remaining, wire::kClassicChunk);
                const bool encoded = fdMode
                    ? wire::EncodeFd(topic.id, topic.pending_sequence, topic.pending_value.data(), count, frame)
                    : wire::EncodeClassic(topic.id, topic.pending_sequence, topic.pending_offset / wire::kClassicChunk,
                                          topic.pending_value.data() + topic.pending_offset, count, frame);
                if (!encoded) { topic.pending = false; break; }
                --budget;
                if (!Send(frame)) break; // 保留本片，短期重试；超过pending超时后丢弃，不无限追逐旧值。
                topic.pending_offset += count;
                if (topic.pending_offset == topic.length) topic.pending = false;
            }
        }
    }

    void Task(void*)
    {
        uint32_t heartbeatTick = osKernelGetTickCount() - Ticks(kHeartbeatMs);
        for (;;)
        {
            wire::Frame frame{};
            while (osMessageQueueGet(rxQueue, &frame, nullptr, 0U) == osOK) Process(frame);
            const uint32_t now = osKernelGetTickCount();
            unsigned budget = kTxBudget;
            if (Expired(now, heartbeatTick, kHeartbeatMs) && CanSend())
            {
                const bool encoded = fdMode ? wire::EncodeFd(0,0,nullptr,0,frame)
                                            : wire::EncodeClassic(0,0,0,nullptr,0,frame);
                --budget;
                if (encoded && Send(frame)) heartbeatTick = now;
            }
            status.rx_dropped = rxDrops + staleDrops;
            if (status.online && Expired(now, status.last_rx_tick, kOfflineMs))
            {
                status.online = false;
                for (auto& topic : rxTopics) topic.assembly.assembling = false;
                PubPushMessage(statusPublisher, &status);
            }
            TransmitTopics(now, budget);
            osDelay(Ticks(kPollMs));
        }
    }

    bool RegisterTopic(std::array<Topic, kTopicCapacity>& topics, size_t& used,
                       uint8_t id, char* name, uint8_t length, bool transmit)
    {
        if (taskHandle == nullptr || osKernelGetState() != osKernelReady ||
            id == 0 || name == nullptr || length == 0 || length > kMaxPayload || used >= topics.size())
            return false;
        size_t nameLength = 0;
        while (nameLength <= MAX_TOPIC_NAME_LEN && name[nameLength] != '\0') ++nameLength;
        if (nameLength == 0 || nameLength > MAX_TOPIC_NAME_LEN) return false;
        for (size_t i = 0; i < used; ++i) if (topics[i].id == id) return false;
        auto& topic = topics[used];
        topic.id = id; topic.length = length;
        if (transmit) topic.subscriber = SubRegister(name, length);
        else topic.publisher = PubRegister(name, length);
        if (transmit ? topic.subscriber == nullptr : topic.publisher == nullptr) return false;
        ++used;
        return true;
    }
}

bool Start()
{
    if (taskHandle != nullptr) return true;
    if (startAttempted || osKernelGetState() != osKernelReady || osKernelGetTickFreq() == 0U ||
        canHandle->Init.TxFifoQueueElmtsNbr <= kMotorReservedSlots)
        return false;
    startAttempted = true; // 失败后不重复分配任务/消息中心资源；不自动停掉共享控制器。
    rxQueue = osMessageQueueNew(16U, sizeof(wire::Frame), nullptr);
    if (rxQueue == nullptr) return false;
    statusPublisher = PubRegister(statusTopicName, sizeof(LinkStatus));
    if (statusPublisher == nullptr)
    { osMessageQueueDelete(rxQueue); rxQueue = nullptr; return false; }
    fdMode = canHandle->Init.FrameFormat != FDCAN_FRAME_CLASSIC;
    if (fdMode)
    {
        FdcanConfigType config{};
        config.can_handle = canHandle; config.master_id = kTxId; config.can_id = kRxId;
        config.can_callback_func = OnFdRx;
        config.tx_reserved_slots = kMotorReservedSlots;
        fdInstance = FdBus::Register(config);
        if (fdInstance == nullptr)
        { osMessageQueueDelete(rxQueue); rxQueue = nullptr; return false; }
    }
    else
    {
        classicCanConfigType config{};
        config.can_handle = canHandle; config.master_id = kTxId; config.can_id = kRxId;
        config.can_callback_func = OnClassicRx;
        config.tx_reserved_slots = kMotorReservedSlots;
        classicInstance = ClassicBus::Register(config);
        if (classicInstance == nullptr)
        { osMessageQueueDelete(rxQueue); rxQueue = nullptr; return false; }
    }
    taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
    // 已注册实例仍属于共享总线；任务创建失败不停止总线、不破坏DM过滤器。
    if (taskHandle == nullptr)
    {
        const auto queue = rxQueue;
        rxQueue = nullptr; // 先令ISR停止排队，再释放队列；实例回调仍安全存在。
        osMessageQueueDelete(queue);
        return false;
    }
    return true;
}

bool RegisterTxTopic(uint8_t id, char* name, uint8_t length)
{ return RegisterTopic(txTopics, txCount, id, name, length, true); }
bool RegisterRxTopic(uint8_t id, char* name, uint8_t length)
{ return RegisterTopic(rxTopics, rxCount, id, name, length, false); }
}
