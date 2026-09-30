/**
 * @file rc.h
 * @brief DJI DT7 遥控器 (DR16 接收机, DBUS 协议) 的解析与访问接口。
 *
 * @note 接收链路: DT7 -> DR16 接收机 -> UART5 (100000 波特率, 9 位数据, 偶校验)。
 * @note 该串口配置等价于 8 位数据 + 偶校验位, 与 SBUS 协议一致, 不要按 8N1 使用。
 * @note 掉线判定由本模块维护, 其他模块应通过 rc::IsOnline() 获取安全状态。
 */

#ifndef INFANTRY_H723_RC_H
#define INFANTRY_H723_RC_H

#include <cstdint>

namespace rc
{
    /** @brief DBUS 单帧长度: 1 起始 + 22 通道 + 1 标志 + 1 结束。 */
    static constexpr uint8_t FRAME_LENGTH = 25;

    /** @brief DBUS 起始字节。 */
    static constexpr uint8_t FRAME_HEADER = 0x0FU;

    /** @brief DBUS 结束字节; 取值有时不为 0, 因此解析时不做强校验。 */
    static constexpr uint8_t FRAME_TAIL = 0x00U;

    /** @brief DBUS 帧中的通道数量。 */
    static constexpr uint8_t CHANNEL_COUNT = 4;

    /** @brief 摇杆通道的原始值范围。 */
    static constexpr uint16_t CHANNEL_MIN = 364U;
    static constexpr uint16_t CHANNEL_MAX = 1684U;

    /** @brief 归一化后的输出范围。 */
    static constexpr float NORMALIZED_MIN = -1.0F;
    static constexpr float NORMALIZED_MAX = 1.0F;

    /** @brief 超过该时间没有收到合法帧即判定遥控掉线, 单位毫秒。 */
    static constexpr uint32_t OFFLINE_TIMEOUT_MS = 100U;

    /** @brief 遥控任务轮询周期, 单位毫秒。 */
    static constexpr uint32_t TASK_PERIOD_MS = 1U;

    /** @brief 遥控器右侧拨杆 (控制模式常用)。 */
    enum class Right_Switch_e : uint8_t
    {
        UP = 0,
        DOWN = 1,
        MIDDLE = 2,
    };

    /** @brief 遥控器左侧拨杆。 */
    enum class Left_Switch_e : uint8_t
    {
        UP = 0,
        DOWN = 1,
        MIDDLE = 2,
    };

    /**
 * @brief 遥控器解析出的全部状态。
 * @note channels 下标对应 SBUS 通道: 0=右摇杆水平, 1=右摇杆垂直, 2=左摇杆垂直, 3=左摇杆水平。
 * @note switch_left/switch_right 直接由原始通道值映射, 因此是枚举而非数值。
 */
    struct RC_Data_s
    {
        uint16_t channels[CHANNEL_COUNT]{};

        Left_Switch_e  switch_left{};
        Right_Switch_e switch_right{};

        // 键盘与鼠标合并成的 16 位原始值, 便于逐位核对接线/固件差异。
        uint16_t keyboard_mouse_raw{};

        // 键盘按键状态, 与遥控器上的丝印一致。
        struct
        {
            bool w : 1;
            bool s : 1;
            bool a : 1;
            bool d : 1;
            bool shift : 1;
            bool ctrl : 1;
            bool q : 1;
            bool e : 1;
        } keyboard{};

        // 鼠标按键与相对位移。
        struct
        {
            bool  left : 1;
            bool  right : 1;
            bool  middle : 1;
            int16_t x;
            int16_t y;
        } mouse{};

        uint32_t last_update_tick{}; // 最近一次收到合法帧的 RTOS tick。
    };

    extern RC_Data_s RC; // 全局遥控状态, 由遥控任务解析更新。

    /**
     * @brief 把摇杆原始值映射到 [-1, 1]。
     * @param raw 通道原始值。
     * @return 归一化结果, 超出标定范围时截断到边界。
     */
    float Normalize(uint16_t raw);

    /** @brief 读取右摇杆水平通道的归一化值。 */
    float RightHorizontal();

    /** @brief 读取右摇杆垂直通道的归一化值。 */
    float RightVertical();

    /** @brief 读取左摇杆垂直通道的归一化值。 */
    float LeftVertical();

    /** @brief 读取左摇杆水平通道的归一化值。 */
    float LeftHorizontal();

    /** @brief 遥控是否在线; 掉线时其他模块必须停止运动输出。 */
    bool IsOnline();

    /** @brief 注册 UART5 接收并创建遥控解析任务; 重复调用不重复创建。 */
    void Init();

    /** @brief 兼容 InfantryInit() 中按模块名创建任务的调用风格。 */
    void CreateRCTask();
} // namespace rc

#endif // INFANTRY_H723_RC_H
