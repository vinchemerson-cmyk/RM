//
// Created by 19923 on 2026/10/12.
//
#pragma once
#include <cstdint>
#include "communication/encoder_telemetry.h"

namespace chassis
{
    enum class Wheel : uint8_t { FrontLeft, FrontRight, BackLeft, BackRight }; // 与配置数组FL/FR/BL/BR一一对应。
    /** @brief 单轮诊断数据的值副本；默认零表示尚未发布，不能把零速当作反馈在线。 */
    struct WheelControlSnapshot
    {
        uint32_t control_tick{}, control_sequence{}; // 控制周期tick / 快照递增编号；二者都不是持续累加的秒数。
        float control_dt_s{}; // 本次控制实际间隔，单位s，可用于观察调度延迟。
        float target_speed_dps{}, actual_speed_dps{}; // 输出轴目标/实际角速度，°/s，已从转子单位换算。
        float command_current_raw{}; // PI逻辑坐标指令码，非A；未包含最终安装方向翻转和CAN量化。
        float feedback_current_a{}; // 驱动解码的反馈电流，A，正方向已随电机逻辑坐标统一。
        bool output_enabled{}, feedback_seen{}; // 软件输出使能 / 是否曾收过首帧；seen为真不代表当前未超时。
        uint32_t feedback_tick{}, tx_attempts{}, tx_failures{}; // 最近反馈tick / 累计发送入队尝试 / 累计入队失败。
    };

    // RTOS已初始化但调度器尚未启动时显式调用；成功表示四轮已注册且任务已创建，不表示电机已运动。
    // Robot.cpp 按 ENABLE_CHASSIS 装配本模块；硬件未确认时返回 false，调用方记录日志后继续启动。
    // 部分注册或任务创建失败后不自动重试，已经占用的驱动池不会自动回收。
    bool Init();
    // 任务上下文读取；临界区拷贝完整结构，无效轮号返回零快照，不自动发送串口或消息中心。
    // 使用者应结合feedback_seen及feedback_tick自行判断数据有效性，不长期持有内部数据引用。
    WheelControlSnapshot GetWheelControlSnapshot(Wheel wheel);
    // 供 UART7 遥测读取的四轮快照（FL/FR/BL/BR × 目标/实际/反馈电流）；未初始化时返回全零。
    serialTasks::ChassisSnapshot GetTelemetrySnapshot();
}


