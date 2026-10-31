# 双板隔离详细修改文档

> 配套文档：[dual-board-isolation.md](dual-board-isolation.md)（设计与角色矩阵）
> 工程：RoboMaster 步兵 H723（`infantry`）　分支：`全向`　编写日期：2026-10-30
> 本文只给出“怎么改”，**没有修改任何源码、CMake 或仓库文件**。所有行号以 2026-10-30 的工作树为准；实际编辑时以函数名和上下文定位。

## 0. 改动总表

| # | 文件 | 操作 | 目的 |
|---|---|---|---|
| 1 | `Application/chassis/ChassisConfig.h` | 修改 | 定义 `kWheelMaxSpeedDps` 占位值；修正过期注释 |
| 2 | `Application/chassis/chassis.cpp` | 修改 | 补 `<limits>`；轮速上限校验；新增底盘遥测快照；修正注释 |
| 3 | `Application/chassis/chassis.h` | 修改 | 声明 `GetTelemetrySnapshot()`；修正注释 |
| 4 | `Application/shoot/ShootConfig.hpp` | 修改 | 关闭拨弹盘，解除 `0x201` 重复 ID 的启动停机 |
| 5 | `Modules/communication/encoder_telemetry.h` | 修改 | 新增底盘十二通道快照类型与格式 |
| 6 | `Modules/communication/encoder_serial_task.h` | 修改 | 新增格式选择与底盘入口 |
| 7 | `Modules/communication/encoder_serial_task.cpp` | 修改 | 同一 UART7 任务支持云台/底盘两种数据源 |
| 8 | `Application/RobotConfig.hpp` | 修改 | 由板卡角色派生模块开关 |
| 9 | `Application/Robot.cpp` | 修改 | 按角色装配；底盘非致命；UART7 按角色路由 |
| 10 | `CMakePresets.json` | 修改 | 四个显式角色预设 |
| 11 | `README.md`、`Application/README.md`、`Application/shoot/README.md` | 修改 | 构建命令、角色说明、UART7 描述 |

没有新增源文件；唯一新增文档是本文与配套设计文档。

---

## 1. 前置约束

### 1.1 工作树现状

当前工作树已有未提交修改：

```
Application/chassis/ChassisConfig.h
Application/chassis/chassis.cpp
Application/chassis/chassis_control.h
Modules/motor/dji/dji.cpp
```

这些是本次改动之前就存在的本地工作，**不要**使用 `git checkout -- .`、`git reset --hard` 或 `git clean` 回滚；需要撤销时只手动撤销本文列出的具体编辑。

### 1.2 语言与工具链

- 工程使用 C++17：`gimbal.cpp`、`shoot.cpp` 已在用 `if constexpr`，GCC 默认 `gnu++17`，不需要改 CMake 标准。
- 工具链 `cmake/gcc-arm-none-eabi.cmake` 已启用 `-ffunction-sections -fdata-sections` 与 `--gc-sections`，未装配模块可在链接阶段被回收。
- `BSP/usart/usartConfig.hpp:22`：`TX_BUF_SIZE = 256`，UART7 单行长度必须小于该值。

### 1.3 实施原则

- 先解除编译/启动阻塞，再做角色装配，最后改预设与文档。
- 每一步都保留可构建状态，避免一次性大改后难以定位错误。
- 不确认的硬件数值一律不用猜测值填充；占位值必须被校验挡住。

---

## 2. 推荐实施顺序与检查点

| 阶段 | 内容 | 检查点 |
|---|---|---|
| A | 第 3.1～3.4 节：编译、启动阻塞与底盘遥测适配 | 用现有 `Debug` 预设（角色=1）构建通过 |
| B | 第 3.5～3.7 节：UART7 类型与任务扩展 | 再次构建 `Debug` 通过；默认参数保持旧调用兼容 |
| C | 第 3.8～3.9 节：角色档案与装配入口 | `Debug`（云台角色）构建通过 |
| D | 第 3.10 节：四个角色预设 | `Debug-Gimbal` 与 `Debug-Chassis` 均构建通过 |
| E | 第 3.11 节：文档更新 | 构建命令与实际预设一致 |

> 阶段 A 结束时，`kWheelMaxSpeedDps` 已定义、拨弹盘已关闭，云台板镜像可以先恢复“可编译 + 可启动到调度器”的状态。

---

## 3. 逐文件修改

### 3.1 `Application/chassis/ChassisConfig.h`

#### 3.1.1 新增轮速占位值

位置：`kChassisGeometry` 定义之前（当前第 52～55 行注释之后）。

```cpp
// 输出轴最大轮速占位值：实物未测量前必须保持 0，ValidateConfiguration() 会拒绝注册；
// 不得据此推算底盘能力，确认后替换为实测值。
inline constexpr float kWheelMaxSpeedDps = 0.0F;
```

注意：

- 这里的 `0.0F` 不是硬件参数，只是让几何表通过编译。
- `kChassisGeometry` 中四个 `max_output_speed_dps` 都引用该值；不要分别填四个猜测值。
- 后续在 `ValidateConfiguration()` 中新增 `> 0` 校验（见 3.2.2），防止把占位值带进已确认硬件的构建。

#### 3.1.2 修正第 18 行注释

修改前：

```cpp
// 原设计为实物参数确认开关；当前chassis.cpp把校验入口注释了，因此此值暂不参与运行拦截。
```

修改后：

```cpp
// 实物参数确认开关；chassis::Init() 会调用 ValidateConfiguration()，
// 此值为 false 时不注册任何底盘电机，也不创建底盘任务。
```

#### 3.1.3 修正第 54 行注释

修改前：

```cpp
// kFL/FR/BL/BRDirectionDeg、kWheelRadiusM、kWheelMaxSpeedDps须有对应定义，不能仅凭变量名推断数值。
```

修改后：

```cpp
// 轮位方向与轮径沿用现有配置；kWheelMaxSpeedDps 当前为零占位，必须实测后替换。
```

---

### 3.2 `Application/chassis/chassis.cpp`

#### 3.2.1 补 `<limits>`

当前 `chassis.cpp` 第 239 行使用 `std::numeric_limits<int16_t>`，但文件与包含链都没有 `<limits>`。在文件头 include 区末尾补：

```cpp
#include <limits>
```

修改后的 include 区：

```cpp
#include "chassis.h"
#include "bsp_log.h"
#include "ChassisConfig.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "safety/safety.h"
#include <limits>
```

#### 3.2.2 在 `ValidateConfiguration()` 中校验轮速上限

位置：轮子循环内，`gear_ratio` 检查之后、`peakRotorSpeed` 计算之前。

```cpp
if (!std::isfinite(kChassisGeometry[i].max_output_speed_dps) ||
    kChassisGeometry[i].max_output_speed_dps <= 0.0F) return false;
```

原因：`WheelVelocityPlanner::Configure()` 不校验这一项，`Update()` 会把它作为限速比例的分子（`chassis_control.h:199-201`）。若将来只把 `kChassisHardwareConfirmed` 改成 true 而忘记替换 0，四轮目标会被静默缩放为 0，表现为“配置成功但底盘不动”。

当前 `kChassisHardwareConfirmed=false` 会在进入该循环之前返回，所以新增校验不会改变现在的启动行为。

#### 3.2.3 新增底盘遥测快照

位置：文件末尾，`GetWheelControlSnapshot()` 之后。

```cpp
serialTasks::ChassisSnapshot chassis::GetTelemetrySnapshot()
{
    serialTasks::ChassisSnapshot snapshot{};
    constexpr Wheel kTelemetryOrder[kWheelCount]{
        Wheel::FrontLeft, Wheel::FrontRight, Wheel::BackLeft, Wheel::BackRight,
    };
    for (unsigned i = 0; i < kWheelCount; ++i)
    {
        const WheelControlSnapshot wheel = GetWheelControlSnapshot(kTelemetryOrder[i]);
        snapshot.wheels[i].target_speed_dps   = wheel.target_speed_dps;
        snapshot.wheels[i].actual_speed_dps   = wheel.actual_speed_dps;
        snapshot.wheels[i].feedback_current_a = wheel.feedback_current_a;
    }
    return snapshot;
}
```

说明：

- 四次 `GetWheelControlSnapshot()` 各自进入短临界区，10ms 周期调用开销可接受；没有新增共享状态。
- 底盘未初始化或 `Init()` 失败时，四轮快照本身为零，本函数自然返回全零。
- 不复制 `control_tick` 等字段：遥测格式只打印 12 个 float，避免 `decimal()` 对大 tick 值做 ±1e6 截断后显示为 0。

#### 3.2.4 修正过期注释

文件头当前为：

```cpp
/** @file chassis.cpp @brief 四全向轮控制与快照接口；当前被注释的初始化校验见Init说明。 */
```

改为：

```cpp
/** @file chassis.cpp @brief 四全向轮控制、配置校验与快照接口。 */
```

`ValidateConfiguration()` 前的 212～215 行注释（“下列完整配置校验目前被手动注释……”）改为：

```cpp
// ValidateConfiguration() 由 Init() 在注册电机前调用：
// 确认硬件开关/CAN 模式、计算 periodTicks、检查摇杆映射和 PI、
// 核对四轮 ID/减速比/轮速上限，最后调用 planner.Configure 缓存几何。
```

`Init()` 内 258～259 行注释（“配置校验入口当前被注释……”）改为：

```cpp
// kChassisHardwareConfirmed=false 时 ValidateConfiguration() 直接返回 false，
// 不会注册任何电机；确认硬件并填入实测参数后再改为 true。
```

---

### 3.3 `Application/chassis/chassis.h`

#### 3.3.1 引入遥测类型并声明接口

修改后的完整文件建议如下：

```cpp
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
    bool Init();
    // 任务上下文读取；临界区拷贝完整结构，无效轮号返回零快照，不自动发送串口或消息中心。
    // 使用者应结合feedback_seen及feedback_tick自行判断数据有效性，不长期持有内部数据引用。
    WheelControlSnapshot GetWheelControlSnapshot(Wheel wheel);
    // 供 UART7 遥测读取的四轮快照（FL/FR/BL/BR × 目标/实际/反馈电流）；
    // 底盘未初始化时返回全零，不表示电机在线。
    serialTasks::ChassisSnapshot GetTelemetrySnapshot();
}
```

注意：`chassis.h` 位于 `Application/chassis/`，`communication/encoder_telemetry.h` 能通过 `Modules` 包含路径找到；`gimbal.h` 已经是同样的包含方式。

---

### 3.4 `Application/shoot/ShootConfig.hpp`

#### 3.4.1 关闭拨弹盘

修改前：

```cpp
/** @brief 三电机测试阶段不注册或驱动拨弹盘。 */
inline constexpr bool             kEnableFeeder      = true;  //测拨弹盘是否启用
inline constexpr uint32_t         kFeederFeedbackId  = 0x201U; // todo 拨弹盘电机	ID
```

修改后：

```cpp
/** @brief 拨弹盘反馈 ID 未确认；与左摩擦轮同为 0x201，DJI 驱动会拒绝重复 ID，暂不注册。 */
inline constexpr bool             kEnableFeeder      = false;
inline constexpr uint32_t         kFeederFeedbackId  = 0x201U; // todo 确认后改为实际 ID，再启用 kEnableFeeder
```

影响：

- `kControlConfig.enable_feeder` 引用该常量，会同步变为 false。
- `shoot.cpp:107-109` 的 `if constexpr (kEnableFeeder)` 分支不注册第三个电机；左右摩擦轮仍按 `0x201/0x202` 注册。
- 单发、连发、堵转反转暂不可用；摩擦轮仍可转动。确认拨弹盘实际 ID 后改回 `true` 并填入真实 ID。

---

### 3.5 `Modules/communication/encoder_telemetry.h`

#### 3.5.1 增加 `<array>`

```cpp
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
```

#### 3.5.2 新增底盘快照类型

位置：`EncoderProvider` 定义之后、`decimal` lambda 之前。

```cpp
/** @brief 单轮 UART7 遥测值；不含时间戳，避免大 tick 被 decimal() 截断。 */
struct WheelTelemetrySample
{
    float target_speed_dps{};
    float actual_speed_dps{};
    float feedback_current_a{};
};

/** @brief 底盘四轮快照，轮序固定为 FL/FR/BL/BR，与 chassis::Wheel 一致。 */
struct ChassisSnapshot
{
    std::array<WheelTelemetrySample, 4U> wheels{};
};

using ChassisProvider = ChassisSnapshot (*)();
```

#### 3.5.3 新增十二通道格式化函数

位置：`FormatPitchControlTelemetryLine()` 之后、命名空间结束之前。

```cpp
/** @brief 底盘四轮十二通道：FL/FR/BL/BR × 目标轮速、实际轮速、反馈电流。 */
inline size_t FormatChassisTelemetryLine(char* buffer, size_t capacity,
                                         const ChassisSnapshot& sps, uint32_t /*now*/)
{
    if (buffer == nullptr || capacity == 0U) return 0U;

    char v[12][24]{};
    const float values[]{
        sps.wheels[0].target_speed_dps, sps.wheels[0].actual_speed_dps, sps.wheels[0].feedback_current_a,
        sps.wheels[1].target_speed_dps, sps.wheels[1].actual_speed_dps, sps.wheels[1].feedback_current_a,
        sps.wheels[2].target_speed_dps, sps.wheels[2].actual_speed_dps, sps.wheels[2].feedback_current_a,
        sps.wheels[3].target_speed_dps, sps.wheels[3].actual_speed_dps, sps.wheels[3].feedback_current_a
    };
    static_assert(sizeof(values) / sizeof(values[0]) == 12U, "Chassis telemetry requires 12 channels");
    for (size_t i = 0; i < 12U; ++i) decimal(values[i], v[i]);
    const int count = std::snprintf(buffer, capacity,
                                    "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
                                    v[0], v[1], v[2], v[3], v[4], v[5],
                                    v[6], v[7], v[8], v[9], v[10], v[11]);
    if (count <= 0 || static_cast<size_t>(count) >= capacity)
    {
        buffer[0] = '\0';
        return 0U;
    }
    return static_cast<size_t>(count);
}
```

长度核算：

- `decimal()` 最多输出 12 个字符（符号 + 7 位整数 + 小数点 + 3 位小数）。
- 12 列共 144 字符，加 11 个逗号、1 个换行、1 个结尾 NUL，最坏约 157 字节。
- `TX_BUF_SIZE=256`，余量约 99 字节；格式化失败返回 0，该周期不发。

栈核算：

- `v` 占 288 字节；原 yaw/pitch 格式的 `v` 占 192 字节。
- UART7 任务栈为 2048 字节，另有 `usartConfig::Input`（含 256 字节缓冲）。构建后检查 `.su` 中 `EncoderSerialTask` 的栈用量。

---

### 3.6 `Modules/communication/encoder_serial_task.h`

修改后的完整文件建议如下：

```cpp
/** @file encoder_serial_task.h @brief UART7 FireWater 遥测任务入口；按角色输出云台或底盘快照。 */
#pragma once
#include "encoder_telemetry.h"

namespace serialTasks
{
    enum class EncoderTelemetryFormat : uint8_t { Pitch, Yaw };

    /** @brief 创建 UART7 TX-only 任务；provider 提供同步反馈快照，UART7 不得再注册回环任务。 */
    void CreateEncoderSerialTask(EncoderProvider provider,
                                 EncoderTelemetryFormat format = EncoderTelemetryFormat::Pitch);

    /** @brief 创建 UART7 底盘遥测任务；与 CreateEncoderSerialTask 共用同一个任务句柄。 */
    void CreateChassisSerialTask(ChassisProvider provider);
}
```

说明：

- 默认参数 `Pitch` 保持旧调用兼容；现有唯一调用点在 `Robot.cpp`。
- 两个创建函数只能成功一个：第二个调用会因 `taskHandle != nullptr` 直接返回。

---

### 3.7 `Modules/communication/encoder_serial_task.cpp`

建议整文件替换为：

```cpp
/** @file encoder_serial_task.cpp @brief UART7 DMA 每 10ms 输出最新云台或底盘诊断快照。 */
#include "encoder_serial_task.h"
#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"
#include "usart/usart.hpp"

namespace
{
    osThreadId_t                       taskHandle       = nullptr;
    serialTasks::EncoderProvider       feedbackProvider = nullptr;
    serialTasks::EncoderTelemetryFormat telemetryFormat =
        serialTasks::EncoderTelemetryFormat::Pitch;
    serialTasks::ChassisProvider       chassisProvider  = nullptr;

    constexpr osThreadAttr_t taskAttrs{
        .name = "EncoderSerialTask", .stack_size = 2048U, .priority = osPriorityNormal,
    };
    constexpr uint32_t kPrintPeriodMs = 10U; // 串口打印周期
    static_assert(usartConfig::TX_BUF_SIZE >= 256U, "UART control telemetry requires a 256-byte buffer");

    /** @brief 在任务中注册 UART7；BSP 将临时文本复制至 AXI SRAM 长期 DMA 缓冲。 */
    void Task(void*)
    {
        constexpr usartConfig::Config config{
            .usart_handle        = &huart7,
            .tx_mode             = usartConfig::TRANSFER_DMA,
            .rx_data_len         = 0U,
            .usart_callback_func = nullptr,
            .module_instance     = &taskHandle,
        };
        auto* uart = usart::Register(config);
        for (;;)
        {
            usartConfig::Input tx{};
            const uint32_t     now = osKernelGetTickCount();
            if (chassisProvider != nullptr)
            {
                const auto snapshot = chassisProvider();
                tx.tx_len = static_cast<uint32_t>(serialTasks::FormatChassisTelemetryLine(
                    reinterpret_cast<char*>(tx.tx_buffer.data()), tx.tx_buffer.size(), snapshot, now));
            }
            else
            {
                const auto   snapshot = feedbackProvider();
                const size_t length =
                    telemetryFormat == serialTasks::EncoderTelemetryFormat::Yaw
                        ? serialTasks::FormatYawControlTelemetryLine(
                              reinterpret_cast<char*>(tx.tx_buffer.data()), tx.tx_buffer.size(), snapshot, now)
                        : serialTasks::FormatPitchControlTelemetryLine(
                              reinterpret_cast<char*>(tx.tx_buffer.data()), tx.tx_buffer.size(), snapshot, now);
                tx.tx_len = static_cast<uint32_t>(length);
            }
            if (tx.tx_len != 0U)
                (void)uart->Transmit(tx); // BUSY/ERROR 不阻塞，下一周期发送最新快照。
            osDelay(kPrintPeriodMs);
        }
    }
}

void serialTasks::CreateEncoderSerialTask(EncoderProvider provider, EncoderTelemetryFormat format)
{
    if (taskHandle != nullptr) return;
    if (provider == nullptr) Error_Handler();
    feedbackProvider = provider;
    telemetryFormat  = format;
    taskHandle       = osThreadNew(Task, nullptr, &taskAttrs);
    if (taskHandle == nullptr) Error_Handler();
}

void serialTasks::CreateChassisSerialTask(ChassisProvider provider)
{
    if (taskHandle != nullptr) return;
    if (provider == nullptr) Error_Handler();
    chassisProvider = provider;
    taskHandle      = osThreadNew(Task, nullptr, &taskAttrs);
    if (taskHandle == nullptr) Error_Handler();
}
```

注意：

- `Task()` 只注册一次 UART7；两种数据源共用同一个任务。
- `chassisProvider` 非空时优先走底盘格式；否则按 `telemetryFormat` 选择 yaw/pitch。
- 两个入口都保留空 provider 与任务创建失败的 `Error_Handler()` 语义。
- 不要把这个任务改成周期性 `osDelayUntil` 之外的忙循环；10ms 周期沿用现状。

---

### 3.8 `Application/RobotConfig.hpp`

建议整文件替换为：

```cpp
/**
 * @file RobotConfig.hpp
 * @brief 整车编译期配置: 板卡角色、CAN 资源分配和各功能模块的启停开关。
 *
 * 两块板 (云台板/底盘板) 共用本工程, 烧录前必须确认 INFANTRY_BOARD_GIMBAL:
 *   - 取 1: 云台板, 装配遥控/安全/云台/发射; 双板链路发送 0x301 / 接收 0x302。
 *   - 取 0: 底盘板, 装配安全/底盘; 双板链路发送 0x302 / 接收 0x301。
 *   两块板必须使用相反的值, 否则双板链路收不到对方心跳, 永远显示离线。
 *
 * 角色差异在本文件集中定义; 模块源码不得再按板卡角色复制逻辑。
 *
 * FDCAN 资源分配 (STM32H723 共 3 路 FDCAN, 具体模式见 board_can_config.h):
 *   云台板: FDCAN1 FD (云台 DM), FDCAN2 Classic (摩擦轮), FDCAN3 FD (空闲), 板间 FDCAN1。
 *   底盘板: FDCAN1 Classic (底盘 DJI), FDCAN2 FD (板间), FDCAN3 Classic (空闲)。
 */

//
// Created by 19923 on 2026/9/24.
//
#pragma once
#ifndef ROBOTINIT_HPP
#define ROBOTINIT_HPP
#include <cstdint>
#include "board_can_config.h"

namespace InfantryInitialization
{
    enum class BoardRole : std::uint8_t { Gimbal, Chassis };
    inline constexpr BoardRole kBoardRole =
#if INFANTRY_BOARD_GIMBAL
        BoardRole::Gimbal;
#else
        BoardRole::Chassis;
#endif

    // ---------- 两板共有 ----------
    inline constexpr bool ENABLE_USB_SERIAL    = true;  // USB CDC 虚拟串口
    inline constexpr bool ENABLE_IMU           = true;  // BMI088 六轴采样与姿态解算
    inline constexpr bool ENABLE_SAFETY        = true;  // 本地输出总闸门
    inline constexpr bool ENABLE_INTERBOARD    = false; // 板间业务本阶段关闭
    inline constexpr bool ENABLE_ENCODER_UART7 = true;  // 云台 yaw / 底盘四轮遥测
    inline constexpr bool ENABLE_BUZZER        = false; // 两板共享开关; 当前无裁判系统模块

    // ---------- 按角色装配 ----------
    inline constexpr bool ENABLE_RC      = (kBoardRole == BoardRole::Gimbal);  // DR16 仅云台板
    inline constexpr bool ENABLE_GIMBAL  = (kBoardRole == BoardRole::Gimbal);
    inline constexpr bool ENABLE_SHOOT   = (kBoardRole == BoardRole::Gimbal);
    inline constexpr bool ENABLE_CHASSIS = (kBoardRole == BoardRole::Chassis);

    static_assert(!ENABLE_GIMBAL || (ENABLE_RC && ENABLE_SAFETY),
                  "Gimbal requires remote control and output safety");
    static_assert(!ENABLE_SHOOT || (ENABLE_RC && ENABLE_SAFETY),
                  "Shooter requires remote control and output safety");
    static_assert(ENABLE_GIMBAL == (kBoardRole == BoardRole::Gimbal),
                  "Gimbal role mismatch");
    static_assert(ENABLE_SHOOT == (kBoardRole == BoardRole::Gimbal),
                  "Shooter role mismatch");
    static_assert(ENABLE_CHASSIS == (kBoardRole == BoardRole::Chassis),
                  "Chassis role mismatch");
    static_assert(ENABLE_RC == (kBoardRole == BoardRole::Gimbal),
                  "DR16 belongs to the gimbal board");
    static_assert(ENABLE_SAFETY, "Both boards keep a local output gate");
}

#endif
```

说明：

- `board_can_config.h` 在宏未由 CMake 提供时给出默认值，因此 `#if INFANTRY_BOARD_GIMBAL` 在 IDE 单独预处理时也不会报错。
- `ENABLE_BUZZER` 不按角色禁用：当前没有裁判系统模块，蜂鸣器只是 TIM12 任务；保留共享开关。
- 角色一致性断言是“自证式”的，但能在后续有人手动改开关时立即报错，防止再出现“底盘板装配云台”。

---

### 3.9 `Application/Robot.cpp`

#### 3.9.1 栈预算不变

文件顶部的 `kApplicationStackBytes` 仍按 `ENABLE_*` 求和，不需要修改。两角色都满足：

- 云台板：`4096 + 2048 + 3072 + 2048 + 1024 + 2048 + 3072 + 2048 = 19456` 字节，加 8KB 余量 ≤ 32KB。
- 底盘板：`4096 + 2048 + 3072 + 1024 + 2048 + 2048 = 14336` 字节，加 8KB 余量 ≤ 32KB。

#### 3.9.2 替换 `InfantryInit()`

建议将 `InfantryInit()` 整体替换为：

```cpp
/** @brief 创建已启用的任务并注册通信接口；重复调用只装配一次。 */
void InfantryInit(void)
{
    static bool initialized = false;
    if (initialized)
    {
        return;
    }
    BSPLogInit();

    if constexpr (InfantryInitialization::ENABLE_USB_SERIAL)
    {
        serialTasks::CreateUsbSerialTask();
    }
    if constexpr (InfantryInitialization::ENABLE_IMU)
    {
        imu::CreateTask();
    }
    // 双板链路与同路设备统一注册；各模块都须在调度器启动前完成装配，不独占或重启总线。
    if constexpr (InfantryInitialization::ENABLE_INTERBOARD)
    {
        if (!interboard::Start())
        {
            Error_Handler();
        }
    }
    if constexpr (InfantryInitialization::ENABLE_BUZZER)
    {
        appTasks::CreateBuzzerTask();
    }
    if constexpr (InfantryInitialization::ENABLE_RC)
    {
        rc::Init();
    }
    if constexpr (InfantryInitialization::ENABLE_SAFETY)
    {
        safety::Init();
    }

    // 云台任务运行时每周期查询遥控和急停状态。
    if constexpr (InfantryInitialization::ENABLE_GIMBAL)
    {
        gimbal::Init();
    }

    if constexpr (InfantryInitialization::ENABLE_CHASSIS)
    {
        // 底盘硬件未确认时 Init() 返回 false；底盘板仍须启动 IMU/UART7/USB，不能整板停机。
        if (!chassis::Init())
        {
            LOGERROR("[robot] chassis not assembled; hardware/configuration incomplete");
        }
    }

    if constexpr (InfantryInitialization::ENABLE_SHOOT)
    {
        shoot::Init();
    }

    if constexpr (InfantryInitialization::ENABLE_ENCODER_UART7)
    {
        if constexpr (InfantryInitialization::kBoardRole == InfantryInitialization::BoardRole::Gimbal)
        {
            // yaw 快照在控制任务中每周期发布；pitch 快照当前从未写入，不能再作为数据源。
            serialTasks::CreateEncoderSerialTask(gimbal::GetYawControlSnapshot,
                                                 serialTasks::EncoderTelemetryFormat::Yaw);
        }
        else
        {
            serialTasks::CreateChassisSerialTask(chassis::GetTelemetrySnapshot);
        }
    }

    initialized = true;
}
```

说明：

- 所有模块闸门改为 `if constexpr`。非模板函数中被丢弃的分支仍会做语义检查，但不会生成调用；配合链接器 `--gc-sections`，未装配模块可被回收。
- `gimbal::Init()`、`shoot::Init()`、`interboard::Start()` 保持致命失败：云台/发射/链路是核心执行路径，不允许半初始化。
- `chassis::Init()` 非致命：底盘板在硬件参数未确认时必须能启动其余模块；日志由 `LOGERROR` 输出到 RTT。
- UART7 不再使用 `GetPitchControlSnapshot`，云台板改为 `GetYawControlSnapshot` + `Yaw` 格式；底盘板使用 `GetTelemetrySnapshot`。

---

### 3.10 `CMakePresets.json`

建议整文件替换为：

```json
{
  "version": 3,
  "configurePresets": [
    {
      "name": "default",
      "hidden": true,
      "generator": "Ninja",
      "binaryDir": "${sourceDir}/build/${presetName}",
      "toolchainFile": "${sourceDir}/cmake/gcc-arm-none-eabi.cmake",
      "cacheVariables": {}
    },
    {
      "name": "Debug-Gimbal",
      "inherits": "default",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug",
        "INFANTRY_BOARD_GIMBAL": "1"
      }
    },
    {
      "name": "Release-Gimbal",
      "inherits": "default",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Release",
        "INFANTRY_BOARD_GIMBAL": "1"
      }
    },
    {
      "name": "Debug-Chassis",
      "inherits": "default",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug",
        "INFANTRY_BOARD_GIMBAL": "0"
      }
    },
    {
      "name": "Release-Chassis",
      "inherits": "default",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Release",
        "INFANTRY_BOARD_GIMBAL": "0"
      }
    }
  ],
  "buildPresets": [
    { "name": "Debug-Gimbal", "configurePreset": "Debug-Gimbal" },
    { "name": "Release-Gimbal", "configurePreset": "Release-Gimbal" },
    { "name": "Debug-Chassis", "configurePreset": "Debug-Chassis" },
    { "name": "Release-Chassis", "configurePreset": "Release-Chassis" }
  ]
}
```

说明：

- `CMakeLists.txt:28` 的 `set(INFANTRY_BOARD_GIMBAL "1" CACHE STRING ...)` 没有 `FORCE`，预设传入的 `-D` 在首次配置时生效。
- `binaryDir` 继承自 `default`，`${presetName}` 会解析成最终预设名，因此四个预设各自使用 `build/<presetName>`。
- 旧的 `Debug` / `Release` 预设被移除；现有 `build/Debug`、`build/Release` 目录仍保留，可手动删除或继续直接 `cmake --build build/Debug`，但不建议与新预设混用。
- CMakeLists 不需要改：`CMakeLists.txt:120` 已把 `INFANTRY_BOARD_GIMBAL` 注入 `stm32cubemx` INTERFACE target，并传递到所有 C/C++ 单元。

---

### 3.11 文档更新

#### 3.11.1 根 `README.md`

“构建与烧录 → 命令”段（当前第 55～61 行）改为：

```bash
# 配置 + 构建 (CMakePresets.json 提供四个角色预设)
cmake --preset Debug-Gimbal
cmake --build build/Debug-Gimbal

# 底盘板
cmake --preset Debug-Chassis
cmake --build build/Debug-Chassis

# 产物: build/Debug-Gimbal/infantry_H723.elf
#       build/Debug-Chassis/infantry_H723.elf
```

烧录命令（当前第 67 行）改为：

```bash
openocd -f stm32h7_daplink.cfg -c "program build/Debug-Gimbal/infantry_H723.elf verify reset exit"
```

底盘板烧录时把路径替换为 `build/Debug-Chassis/infantry_H723.elf`；两块板不要烧同一份 ELF。

第 74 行关于四电机测试和 UART7 的描述改为：

```markdown
当前云台板：FDCAN1 的 yaw/pitch 使用 LQI 位置控制，FDCAN2 双摩擦轮；UART7 输出 yaw 八通道同步控制诊断。
底盘板：装配底盘、IMU、UART7、USB；底盘硬件未确认时 `chassis::Init()` 非致命失败，UART7 四轮十二通道输出全零。
板间速度指令与底盘运动属于下一阶段。
```

#### 3.11.2 `Application/README.md`

- 第 24 行 chassis 状态：`尚未实现` → `已装配；硬件未确认时不注册电机/任务`。
- 第 27～33 行的“RobotConfig.hpp — 编译期配置”改为说明“角色决定装配集合”，并给出第 3 节角色矩阵的摘要。
- 第 41 行 UART7 描述改为：

```markdown
`ENABLE_ENCODER_UART7=true` 时：云台板输出 yaw 八通道同步控制诊断；底盘板输出四轮十二通道
（FL/FR/BL/BR × 目标轮速、实际轮速、反馈电流）。底盘未初始化时十二通道为 0。
```

> 本文件里 GM6020、六通道等历史描述与本次隔离无关，建议后续单独清理，不要在同一提交里混入大规模文档重写。

#### 3.11.3 `Application/shoot/README.md`

第 116～117 行的构建命令改为：

```bash
cmake --build --preset Debug-Gimbal --parallel
cmake --build --preset Release-Gimbal --parallel
```

因为 shoot 只在云台板装配。

---

## 4. 构建与验证

### 4.1 四预设构建

```bash
cmake --preset Debug-Gimbal
cmake --build build/Debug-Gimbal --parallel

cmake --preset Release-Gimbal
cmake --build build/Release-Gimbal --parallel

cmake --preset Debug-Chassis
cmake --build build/Debug-Chassis --parallel

cmake --preset Release-Chassis
cmake --build build/Release-Chassis --parallel
```

四个都必须配置、编译、链接成功。若只验证编译速度，至少完成 `Debug-Gimbal` 与 `Debug-Chassis`；符号回收检查使用 Release。

### 4.2 角色定义检查

```bash
grep "INFANTRY_BOARD_GIMBAL" build/Debug-Gimbal/CMakeCache.txt
grep "INFANTRY_BOARD_GIMBAL" build/Debug-Chassis/CMakeCache.txt
```

预期分别为 `STRING=1` 与 `STRING=0`。

```bash
grep -n "INFANTRY_BOARD_GIMBAL" build/Debug-Gimbal/compile_commands.json
grep -n "INFANTRY_BOARD_GIMBAL" build/Debug-Chassis/compile_commands.json
```

预期 `Application/Robot.cpp` 与 `Core/Src/fdcan.c` 的编译命令分别带 `-DINFANTRY_BOARD_GIMBAL=1/0`。

### 4.3 符号回收检查

工具链已启用 `--gc-sections`，在 Release 镜像上检查：

```bash
# 底盘镜像不应出现云台/发射初始化
arm-none-eabi-nm --demangle build/Release-Chassis/infantry_H723.elf | grep -E "gimbal::Init|shoot::Init"

# 云台镜像不应出现底盘初始化与底盘遥测
arm-none-eabi-nm --demangle build/Release-Gimbal/infantry_H723.elf | grep -E "chassis::Init|chassis::Task|chassis::GetTelemetrySnapshot"
```

预期两条命令都无输出。注意：

- `gimbal.cpp`、`shoot.cpp`、`chassis.cpp` 仍会被编译；被丢弃的只是没有引用的函数/数据。
- 底盘镜像中 `chassis::Init` 是可达的，因此 `chassis::Task` 会保留；它不会在 `kChassisHardwareConfirmed=false` 时被创建。
- 若符号仍存在，先确认 Release 链接命令包含 `--gc-sections`，再检查是否有其他文件引用该函数。

### 4.4 任务栈与堆检查

构建后查看 `*.su`（`-fstack-usage`）：

- 云台镜像应包含 `GimbalTask`、`ShootTask`、`EncoderSerialTask`、`SafetyTask`、`ImuTask`，不包含 `ChassisTask`。
- 底盘镜像应包含 `ChassisTask`、`EncoderSerialTask`、`SafetyTask`、`ImuTask`，不包含 `GimbalTask`、`ShootTask`。
- `EncoderSerialTask` 的栈用量因新增 `char v[12][24]` 约增加 96 字节；任务栈 2048 字节，需确认剩余余量。
- 两角色的 `static_assert(configTOTAL_HEAP_SIZE >= ...)` 均应在编译期通过。

### 4.5 源码静态检查

```bash
# 各只有一处 UART7 创建调用
grep -n "CreateEncoderSerialTask\|CreateChassisSerialTask" Application/Robot.cpp

# 底盘 Init 返回值被消费
grep -n "chassis::Init" Application/Robot.cpp

# CubeMX 钩子仍在（CMake 配置阶段也会强制）
grep -n "ApplyBoardCanConfiguration" Core/Src/fdcan.c
```

预期：

- `CreateEncoderSerialTask` 与 `CreateChassisSerialTask` 各一处，且都在 `if constexpr` 角色分支内。
- `chassis::Init` 只出现在 `if (!chassis::Init())` 中，没有裸调用。
- `fdcan.c` 仍有 `ApplyBoardCanConfiguration(&hfdcan1, 0U)`、`(&hfdcan2, 1U)`、`(&hfdcan3, 2U)` 三条。

### 4.6 明确不做的验证

不烧录、不连接 DR16、不接底盘电机、不验证 CAN 波形、不验证 UART7 实际接线。上述检查只证明“两套镜像能编译、装配关系正确、未装配模块被链接器回收”，不能证明硬件行为。

---

## 5. 风险与回滚

| 风险 | 说明 | 处理 |
|---|---|---|
| 误用整体回滚 | 工作树已有 `ChassisConfig.h`、`chassis.cpp`、`chassis_control.h`、`dji.cpp` 的未提交修改 | 只手动撤销本文列出的编辑；禁止 `git reset --hard` / `git checkout -- .` |
| 角色镜像烧错板 | 底盘镜像烧到云台板不会主动报错，只会表现为 CAN 通信失败 | 烧录前核对预设名、ELF 路径与板卡标签 |
| 底盘板静止 | 无 RC、无板间命令、硬件未确认 | 本阶段预期；UART7 四轮数据为 0 不是故障 |
| 占位轮速进入实车 | `kWheelMaxSpeedDps=0` 若未被替换，规划器会把四轮目标缩放为 0 | 新增的 `> 0` 校验会拒绝注册；确认硬件前不要改 `kChassisHardwareConfirmed` |
| 拨弹盘不可用 | `kEnableFeeder=false` 关闭单发/连发/堵转反转 | 确认拨弹盘实际 ID 后改回，并保留左右轮 `0x201/0x202` |
| 底盘板无 RC 时 `ENABLE_SAFETY=true` | 安全任务运行但永不 arm；不是故障 | 作为第二阶段接入板间安全状态的本地闸门 |
| UART7 行被截断 | 12 通道最坏约 157 字节；未来加列可能超过 256 | 格式化函数截断时返回 0；加列前重新核算 `TX_BUF_SIZE` |
| `if constexpr` 非模板丢弃分支 | 仍会语义检查，被引用函数必须有声明 | `gimbal.h`/`chassis.h` 已声明；不要删掉未装配模块的头文件包含 |
| 旧 `Debug`/`Release` 预设被移除 | IDE/脚本中的旧命令会失效 | 同步更新 README、IDE 运行配置与 CI 脚本 |

---

## 6. 不在本次改动范围

- `Core/Src/main.c`、`Core/Src/freertos.c`、`Core/Inc/board_can_config.h`：外设初始化与 CAN 模式表保持现状。
- `Modules/communication/interboard.*`：不注册业务话题、不启用 `ENABLE_INTERBOARD`。
- `Modules/motor/dji/*`、`Modules/motor/dm/*`：驱动行为不变；本次只修拨弹盘配置。
- `Application/gimbal/*` 的控制逻辑、`Application/chassis/chassis_control.h` 的规划算法、`Modules/safety/*`、`Modules/rc/*`：逻辑不变，只改装配与数据出口。
- 底盘硬件参数、拨弹盘 ID、蜂鸣器/裁判系统启用、板间速度/安全转发。

---

## 7. 一句话总结

先让代码重新可编译、镜像可启动（占位轮速 + 关闭拨弹盘），再把“哪些模块被装配”从运行时判断提升为角色档案，最后用四个预设把两套镜像固定下来；本阶段结束时，底盘板能启动但不会动，这是下一阶段板间转发要解决的问题。
