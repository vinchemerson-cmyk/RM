/**
 * @file rc.cpp
 * @brief 解析 DJI DT7 遥控器 (DBUS) 帧并维护掉线状态。
 *
 * 物理链路: DT7 遥控器 --2.4G--> DR16 接收机 --DBUS--> UART5 DMA+Idle。
 * UART 配置为 100000 波特率、9 位数据 + 偶校验 (等价 8E1), 与 SBUS 一致。
 *
 * DBUS 帧格式 (25 字节, 每 14ms 一帧):
 *   [0]      0x0F 起始字节;
 *   [1..22]  16 通道 x 11 位小端比特流。本工程使用:
 *              通道 0/1 = 右摇杆水平/垂直, 通道 2/3 = 左摇杆垂直/水平,
 *              通道 4/5 = 左/右拨杆三档 (364/1024/1684),
 *              通道 6 = 键鼠合并 11 位位段 (位序见 ApplyFrame);
 *   [23]     帧标志字节;
 *   [24]     结束字节 (个别固件不为 0x00, 只用于辅助定位, 不做强校验)。
 *
 * 线程模型:
 *   - ApplyFrame() 在 UART 中断内执行, 只做位运算与标量写入;
 *     last_update_tick 最后写入, 配合 release/acquire fence 保证
 *     任务侧要么读到旧帧要么读到完整新帧;
 *   - Task 仅轮询 IsOnline() 维护在线状态; 缓冲区被写满说明流中已
 *     丢失帧边界, 此时重启接收链路丢弃残留数据。
 */

#include "rc.h"

#include <atomic>

#include "cmsis_os2.h"
#include "usart.h"
#include "usart/usart.hpp"

namespace
{
    /** @brief 是否已经创建过遥控任务。 */
    osThreadId_t taskHandle = nullptr;
    usart*       uartInstance = nullptr;

    constexpr osThreadAttr_t taskAttrs = {
        .name       = "RCTask",
        .stack_size = 2048U,
        .priority   = osPriorityAboveNormal,
    };

    /**
 * @brief 从 11 位紧凑通道编码中取出第 index 个通道。
 * @return 通道原始值, 范围 0..2047。
 * @note DBUS 把 16 个通道按 11 位小端依次排布, 因此可能跨越字节边界。
 * @note 只解析前 4 个通道, 但按通用公式计算, 便于复核。
 */
    uint16_t DecodeChannel(const uint8_t* payload, const uint8_t index)
    {
        const uint16_t bit_offset = static_cast<uint16_t>(index) * 11U;
        const auto  byte_index = static_cast<uint8_t>(bit_offset / 8U);
        const auto  bit_index  = static_cast<uint8_t>(bit_offset % 8U);

        // 先取 22 位窗口, 避免 11 位数据跨字节时漏掉高位。
        uint16_t low  = static_cast<uint16_t>(payload[byte_index]) |
                       static_cast<uint16_t>(payload[byte_index + 1U]) << 8U;
        uint32_t value = static_cast<uint32_t>(low) |
                         static_cast<uint32_t>(payload[byte_index + 2U]) << 16U;
        return static_cast<uint16_t>((value >> bit_index) & 0x07FFU);
    }

    /**
 * @brief 把原始通道值映射为左右拨杆状态。
 * @note DBUS 的拨杆三档原始值约为 364 / 1024 / 1684。
 */
    rc::Left_Switch_e DecodeLeftSwitch(const uint16_t raw)
    {
        if (raw < 700U)
            return rc::Left_Switch_e::UP;
        if (raw > 1300U)
            return rc::Left_Switch_e::DOWN;
        return rc::Left_Switch_e::MIDDLE;
    }

    rc::Right_Switch_e DecodeRightSwitch(const uint16_t raw)
    {
        if (raw < 700U)
            return rc::Right_Switch_e::UP;
        if (raw > 1300U)
            return rc::Right_Switch_e::DOWN;
        return rc::Right_Switch_e::MIDDLE;
    }

    /**
 * @brief 解析一帧并更新全局遥控状态。
 * @note 本函数在 UART 中断内执行, 只做位运算和标量赋值, 不含任何阻塞调用。
 * @note last_update_tick 最后写入, 任务侧据此判断本次状态已完整可见。
 */
    void ApplyFrame(const uint8_t* frame)
    {
        auto& [channels, switch_left, switch_right, keyboard_mouse_raw, keyboard, mouse, last_update_tick] = rc::RC;
        const uint8_t* payload = frame + 1U;

        for (uint8_t i = 0; i < rc::CHANNEL_COUNT; ++i)
            channels[i] = DecodeChannel(payload, i);

        switch_left  = DecodeLeftSwitch(DecodeChannel(payload, 4U));
        switch_right = DecodeRightSwitch(DecodeChannel(payload, 5U));

        // 键盘与鼠标合并在一个 16 位字段中 (SBUS 通道 6)。
        const uint16_t keyboard_mouse = DecodeChannel(payload, 6U);
        keyboard_mouse_raw       = keyboard_mouse;

        // 位序以 DR16 常见固件为准; 若实测不符, 可直接比对 keyboard_mouse_raw 定位。
        keyboard.w     = (keyboard_mouse & (1U << 0U)) != 0U;
        keyboard.s     = (keyboard_mouse & (1U << 1U)) != 0U;
        keyboard.a     = (keyboard_mouse & (1U << 2U)) != 0U;
        keyboard.d     = (keyboard_mouse & (1U << 3U)) != 0U;
        keyboard.shift = (keyboard_mouse & (1U << 4U)) != 0U;
        keyboard.ctrl  = (keyboard_mouse & (1U << 5U)) != 0U;
        keyboard.q     = (keyboard_mouse & (1U << 6U)) != 0U;
        keyboard.e     = (keyboard_mouse & (1U << 7U)) != 0U;

        mouse.left   = (keyboard_mouse & (1U << 8U)) != 0U;
        mouse.right  = (keyboard_mouse & (1U << 9U)) != 0U;
        mouse.middle = (keyboard_mouse & (1U << 10U)) != 0U;
        // 鼠标位移为 11 位有符号补码, 静止时读数为 1024。
        mouse.x = static_cast<int16_t>((keyboard_mouse >> 11U) & 0x07FFU) - 1024;
        mouse.y = static_cast<int16_t>((keyboard_mouse >> 13U) & 0x07FFU) - 1024;

        std::atomic_thread_fence(std::memory_order_release);
        last_update_tick = osKernelGetTickCount();
    }

    /**
 * @brief 在本次接收缓冲区内扫描所有合法帧; 跨缓冲区边界残缺的帧会被丢弃。
 * @return 是否至少解析出一帧。
 * @note 接收缓冲区每次回调都从下标 0 重新取得数据, 因此可以整段扫描。
 */
    bool ParseBuffer(const uint8_t* buffer, const uint32_t size)
    {
        bool decoded = false;
        // 起始字节之后仍需 22 个通道字节, 因此最大起点为 size - FRAME_LENGTH。
        if (size < rc::FRAME_LENGTH)
            return false;

        for (uint32_t offset = 0; offset + rc::FRAME_LENGTH <= size; ++offset)
        {
            if (buffer[offset] != rc::FRAME_HEADER)
                continue;
            // 只校验结束字节, 起始字节已足够定位; 结束字节偶有偏差时不影响通道解析。
            if (buffer[offset + rc::FRAME_LENGTH - 1U] != rc::FRAME_TAIL)
                continue;

            ApplyFrame(&buffer[offset]);
            decoded = true;
            offset += rc::FRAME_LENGTH - 1U; // 跳过本帧, 继续寻找后续帧。
        }
        return decoded;
    }

    /**
 * @brief UART5 接收回调: 解析新到达的数据。
 * @note 该回调由 BSP 在 IDLE 或 DMA 完成中断中调用。
 */
    void OnReceive(usart* instance)
    {
        if (instance == nullptr)
            return;

        const auto& rx = instance->Receive();
        uint32_t    size = rx.rx_size;
        if (size > rx.rx_buffer.size())
            size = rx.rx_buffer.size();
        if (size == 0)
            return;

        ParseBuffer(rx.rx_buffer.data(), size);

        // 接收缓冲区被填满说明流中已找不到帧边界 (例如遥控器刚上电或波特率不符),
        // 此时重启接收链路以丢弃残留数据, 避免解析永久停摆。
        if (size >= rx.rx_buffer.size())
            instance->restartRx();
    }

    /**
 * @brief 轮询掉线状态; 遥控数据本身由中断更新, 任务只负责超时判定。
 */
    void Task(void*)
    {
        bool online = false;
        for (;;)
        {
            const bool now_online = rc::IsOnline();
            if (now_online != online)
                online = now_online;
            osDelay(rc::TASK_PERIOD_MS);
        }
    }
} // namespace

rc::RC_Data_s rc::RC{};

float rc::Normalize(const uint16_t raw)
{
    if (raw <= CHANNEL_MIN)
        return NORMALIZED_MIN;
    if (raw >= CHANNEL_MAX)
        return NORMALIZED_MAX;

    const auto span = static_cast<float>(CHANNEL_MAX - CHANNEL_MIN);
    return (static_cast<float>(raw) - static_cast<float>(CHANNEL_MIN)) / span * 2.0F - 1.0F;
}

float rc::RightHorizontal()
{
    return Normalize(RC.channels[0]);
}

float rc::RightVertical()
{
    return Normalize(RC.channels[1]);
}

float rc::LeftVertical()
{
    return Normalize(RC.channels[2]);
}

float rc::LeftHorizontal()
{
    return Normalize(RC.channels[3]);
}

bool rc::IsOnline()
{
    const uint32_t tick = RC.last_update_tick;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (tick == 0)
        return false; // 从未收到过合法帧。
    return osKernelGetTickCount() - tick <= OFFLINE_TIMEOUT_MS;
}

void rc::CreateRCTask()
{
    if (taskHandle != nullptr)
        return;

    taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
    if (taskHandle == nullptr)
        return; // 任务创建失败时不阻塞启动流程, 掉线判定会保持离线。
}

void rc::Init()
{
    // 首次调用完成 UART5 注册, 之后重复调用只保证任务存在。
    if (uartInstance == nullptr)
    {
        const usartConfig::Config config = {
            .usart_handle        = &huart5,
            .tx_mode             = usartConfig::NO_TRANSFER, // 遥控链路只收不发。
            .rx_data_len         = usartConfig::RX_BUF_SIZE,
            .usart_callback_func = OnReceive,
            .module_instance     = &uartInstance,
        };        uartInstance = usart::Register(config);
    }

    CreateRCTask();
}
