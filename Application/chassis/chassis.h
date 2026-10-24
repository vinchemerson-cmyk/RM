//
// Created by 19923 on 2026/10/12.
//
#pragma once
#include <cstdint>

namespace chassis
{
    enum class Wheel : uint8_t { FrontLeft, FrontRight, BackLeft, BackRight };
    struct WheelControlSnapshot
    {
        uint32_t control_tick{}, control_sequence{};
        float control_dt_s{};
        float target_speed_dps{}, actual_speed_dps{}; // 输出轴 °/s。
        float command_current_raw{}; // PI逻辑坐标电流指令码，未做安装方向翻转及量化。
        float feedback_current_a{};
        bool output_enabled{}, feedback_seen{};
        uint32_t feedback_tick{}, tx_attempts{}, tx_failures{};
    };

    // 调度器启动前显式调用；硬件未确认/配置留空返回false，无电机注册。
    // 当前不由Robot.cpp调用；局部注册或任务创建失败后不自动重试。
    bool Init();
    // 任务上下文读取；无效轮号返回零快照，不自动发送串口或消息中心。
    WheelControlSnapshot GetWheelControlSnapshot(Wheel wheel);
}


