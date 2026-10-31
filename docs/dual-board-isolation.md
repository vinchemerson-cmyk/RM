# 双板隔离设计：云台板与底盘板

> 工程：RoboMaster 步兵 H723（`infantry`）
> 分支：`全向`　核对日期：2026-10-30
> 文档性质：设计说明。本文描述“应该怎样隔离”，不表示相关改动已经落地；第 1 节的现状均以当前工作树为准。

## 结论

两板共用一套源码时，隔离的核心不是 `#if` 掉几个外设初始化，而是让**板卡角色决定装配哪些应用模块**。

推荐的分层如下：

1. **编译期角色**：`INFANTRY_BOARD_GIMBAL` 已在 `CMakeLists.txt` 中校验并注入所有 C/C++ 编译单元，继续作为唯一角色来源。
2. **装配层**：`RobotConfig.hpp` 由角色派生 `ENABLE_RC / ENABLE_GIMBAL / ENABLE_SHOOT / ENABLE_CHASSIS`，`Robot.cpp` 只装配本角色拥有的模块。
3. **总线层**：`board_can_config.h` 继续负责每板 CAN1/2/3 的模式、时序与板间控制器，不承担模块启停。
4. **遥测层**：UART7 按角色选择数据源——云台板输出 yaw 快照，底盘板输出四轮快照。
5. **构建层**：每个角色独立配置目录与预设，避免同一份 CMake cache 在两块板之间来回切换。

本阶段只做上述**模块装配隔离**。底盘运动指令由云台板经板间链路转发属于第二阶段，本次不实现；因此底盘板在本阶段保持静止是预期行为。

---

## 1. 现状

### 1.1 角色宏已经存在，但只作用在 CAN 层

`Core/Inc/board_can_config.h:37-55` 根据角色选择不同的 CAN 模式表和板间控制器：

| 控制器 | 云台板（`=1`） | 底盘板（`=0`） |
|---|---|---|
| FDCAN1 | FD | CLASSIC |
| FDCAN2 | CLASSIC | FD |
| FDCAN3 | FD | CLASSIC |
| 板间链路所在控制器 | FDCAN1 | FDCAN2 |

`Modules/communication/interboard.cpp:32-35` 同时按角色交换心跳 ID：云台板发 `0x301` / 收 `0x302`，底盘板发 `0x302` / 收 `0x301`。

这些配置是正确的，但它们只决定“总线怎么工作”，不决定“板上装配什么”。

### 1.2 应用层装配没有隔离

`Application/RobotConfig.hpp:32-50` 中除 `ENABLE_INTERBOARD` 与 `ENABLE_BUZZER` 外，其余开关在两块板上都相同：

```
ENABLE_USB_SERIAL / ENABLE_IMU / ENABLE_RC / ENABLE_SAFETY
ENABLE_GIMBAL / ENABLE_SHOOT / ENABLE_CHASSIS / ENABLE_ENCODER_UART7 = true
```

`Application/Robot.cpp:54-95` 因此会在两块板上都调用 `rc::Init()`、`safety::Init()`、`gimbal::Init()`、`chassis::Init()`、`shoot::Init()`。这会带来以下问题：

1. **底盘板在 `gimbal::Init()` 停机**
   云台 DM4310 通过 `kGimbalCanHandle = &hfdcan1` 注册，而底盘板的 FDCAN1 是 CLASSIC。DM 驱动显式要求 FD 模式（`Modules/motor/dm/dmmotor.cpp:114-115`），注册返回空，`gimbal::Init()` 在调度器启动前调用 `Error_Handler()`（`Application/gimbal/gimbal.cpp:566-568`）。

2. **即使跳过云台，底盘板的发射也会停机**
   `shoot::Init()` 使用 `hfdcan2`（`Application/shoot/ShootConfig.hpp:9`），底盘板的 FDCAN2 是 FD；DJI 驱动要求 CLASSIC（`Modules/motor/dji/dji.cpp:288`）。注册失败同样触发 `Error_Handler()`。

3. **云台板装配了底盘模块**
   `chassis::Init()` 被调用，返回值被忽略（`Application/Robot.cpp:81-84`）。当前 `kChassisHardwareConfirmed=false`，`ValidateConfiguration()`（`Application/chassis/chassis.cpp:260`）会记录日志并返回 false，所以暂时不会注册电机；但装配关系本身是错的。即使硬件确认，底盘配置还要求 CAN1 为 CLASSIC（`chassis.cpp:219`），在云台板上也永远无法通过。

4. **底盘板没有遥控输入**
   DR16 只接云台板。底盘板若装配 RC，`rc::IsOnline()` 永远为 false；即使底盘电机注册成功，`chassis::Task` 的 `remoteValid` 也恒为 false，输出始终被禁止。

### 1.3 当前工作树的两个硬阻塞

| 类型 | 位置 | 现象 |
|---|---|---|
| 编译失败 | `Application/chassis/ChassisConfig.h:63/70/77/84` | `kWheelMaxSpeedDps` 未定义，当前无法生成可执行文件 |
| 启动停机 | `Application/shoot/ShootConfig.hpp:11-14` | 拨弹盘 `kFeederFeedbackId = 0x201` 与左摩擦轮 `kLeftFeedbackId = 0x201` 重复；DJI 驱动拒绝重复反馈 ID（`dji.cpp:297-300`），`shoot::Init()` 在 `shoot.cpp:109-110` 调用 `Error_Handler()` |

第一项是纯编译问题；第二项只在启用拨弹盘时出现，当前 `kEnableFeeder=true`，所以云台板镜像也会在调度器启动前停机。

### 1.4 UART7 当前数据源是错误的

`Application/Robot.cpp:94` 把 `gimbal::GetPitchControlSnapshot` 交给 UART7 任务。但 `gimbal.cpp` 只发布 `yawControlSnapshot`，从未写入 `aim_pitchControlSnapshot`（`gimbal.cpp:542-548` 返回的是零快照）。因此 UART7 的 pitch 八通道长期输出 0，不能作为诊断依据。

---

## 2. 隔离目标

### 本阶段要做

- 用板卡角色决定应用模块的装配集合，而不是运行时判断。
- 让底盘板在没有 DR16、没有板间命令的前提下仍能正常启动 IMU、UART7、USB CDC。
- 让云台板只装配云台与发射，不再触碰底盘。
- 给两个角色提供明确的构建入口，避免烧错镜像。
- 修复当前阻塞编译和启动的最小问题，不涉及新功能。

### 本阶段不做

- 板间速度指令、安全状态转发、业务话题注册。
- 底盘实际运动；底盘板在本阶段保持静止。
- 底盘硬件参数确认（电机 ID、减速比、轮速上限）。
- 拨弹盘 ID 确认与发射流程恢复。
- 蜂鸣器/裁判系统启用（当前没有裁判系统模块）。
- 按角色裁剪 `main.c` 的 CubeMX 外设初始化；隔离只发生在模块注册层。

---

## 3. 角色矩阵

| 开关 | 云台板（`INFANTRY_BOARD_GIMBAL=1`） | 底盘板（`=0`） | 理由 |
|---|---|---|---|
| `ENABLE_USB_SERIAL` | true | true | 两板都保留 USB CDC；底盘板用于调试 |
| `ENABLE_IMU` | true | true | 底盘板保留姿态输出 |
| `ENABLE_RC` | true | false | DR16 只接云台板 |
| `ENABLE_SAFETY` | true | true | 本地输出总闸门；底盘板无 RC 时恒禁止，保留为第二阶段接入板间安全状态的入口 |
| `ENABLE_GIMBAL` | true | false | 底盘板没有 DM 云台电机，也不应占用 FD 的 FDCAN1 |
| `ENABLE_SHOOT` | true | false | 底盘板没有摩擦轮/拨弹盘 |
| `ENABLE_CHASSIS` | false | true | 底盘板装配底盘模块；硬件未确认时 `Init()` 非致命失败 |
| `ENABLE_ENCODER_UART7` | true | true | 云台板输出 yaw 八通道；底盘板输出四轮十二通道 |
| `ENABLE_INTERBOARD` | false | false | 本阶段不启用板间业务 |
| `ENABLE_BUZZER` | 共享开关（当前 false） | 共享开关（当前 false） | 不按角色禁用，保留底盘板能力；硬件未确认前不启用 |

矩阵之外的两个约束：

- `ENABLE_SAFETY` 在两块板都保留。它不依赖遥控初始化即可安全默认：`safety.cpp` 中 `offline=true`、`armed=false`，未收到遥控时 `ShouldOutput()` 恒为 false（`Modules/safety/safety.cpp:64-108`）。
- `ENABLE_BUZZER` 不做角色限制。当前没有裁判系统模块，蜂鸣器任务只依赖 TIM12；保持共享开关，后续在任一板上确认硬件后都可启用。

---

## 4. 推荐设计

> 本节是设计建议，尚未写入代码。

### 4.1 编译期角色档案（`Application/RobotConfig.hpp`）

在 `InfantryInitialization` 中引入角色枚举，由它派生模块开关：

```cpp
enum class BoardRole : uint8_t { Gimbal, Chassis };
inline constexpr BoardRole kBoardRole =
#if INFANTRY_BOARD_GIMBAL
    BoardRole::Gimbal;
#else
    BoardRole::Chassis;
#endif

inline constexpr bool ENABLE_USB_SERIAL = true;
inline constexpr bool ENABLE_IMU        = true;
inline constexpr bool ENABLE_SAFETY     = true;
inline constexpr bool ENABLE_INTERBOARD = false;
inline constexpr bool ENABLE_ENCODER_UART7 = true;
inline constexpr bool ENABLE_BUZZER     = false;  // 共享开关

inline constexpr bool ENABLE_RC      = (kBoardRole == BoardRole::Gimbal);
inline constexpr bool ENABLE_GIMBAL  = (kBoardRole == BoardRole::Gimbal);
inline constexpr bool ENABLE_SHOOT   = (kBoardRole == BoardRole::Gimbal);
inline constexpr bool ENABLE_CHASSIS = (kBoardRole == BoardRole::Chassis);
```

同时保留现有 `ENABLE_GIMBAL/SHOOT => RC && SAFETY` 断言，并补充角色一致性断言：

```cpp
static_assert(ENABLE_GIMBAL  == (kBoardRole == BoardRole::Gimbal));
static_assert(ENABLE_SHOOT   == (kBoardRole == BoardRole::Gimbal));
static_assert(ENABLE_CHASSIS == (kBoardRole == BoardRole::Chassis));
static_assert(ENABLE_RC      == (kBoardRole == BoardRole::Gimbal));
static_assert(ENABLE_SAFETY);
```

`RobotConfig.hpp` 已经包含 `board_can_config.h`，后者在宏未定义时提供默认值，因此不需要额外包含。

### 4.2 装配入口（`Application/Robot.cpp`）

- 把模块开关从运行时 `if` 改为 `if constexpr`，让未装配模块的调用不出现在生成代码中；工具链已开启 `-ffunction-sections -fdata-sections` 与 `--gc-sections`，未引用的模块函数可被链接器回收。
- 云台板：`safety::Init()`、`gimbal::Init()`、`shoot::Init()` 保持致命失败语义——云台/发射是核心执行机构，不允许“半个模块”运行。
- 底盘板：`chassis::Init()` 失败必须**非致命**，否则硬件配置未完成时整板无法启动：

```cpp
if constexpr (InfantryInitialization::ENABLE_CHASSIS)
{
    if (!chassis::Init())
        LOGERROR("[robot] chassis not assembled; hardware/configuration incomplete");
}
```

这样底盘板仍可启动 IMU、UART7、USB；`chassis::Init()` 内部已经记录了具体原因。

### 4.3 UART7 遥测路由

替换当前永远为零的 pitch 数据源：

```cpp
if constexpr (InfantryInitialization::ENABLE_ENCODER_UART7)
{
    if constexpr (InfantryInitialization::kBoardRole == InfantryInitialization::BoardRole::Gimbal)
        serialTasks::CreateEncoderSerialTask(gimbal::GetYawControlSnapshot,
                                             serialTasks::EncoderTelemetryFormat::Yaw);
    else
        serialTasks::CreateChassisSerialTask(chassis::GetTelemetrySnapshot);
}
```

云台板直接使用已经每周期发布的 yaw 快照（`gimbal::GetYawControlSnapshot`），解决当前全零问题。

底盘板需要一个新的十二通道格式：四轮 FL/FR/BL/BR 各输出“目标轮速、实际轮速、反馈电流”，共 12 列。数据来自 `chassis::GetWheelControlSnapshot()`，在 `ChassisConfig` 未确认、底盘任务未创建时自然全为零。

长度核算：每个数值经现有 `decimal()` 最多 12 个字符（含符号与三位小数），12 列 + 11 个逗号 + 换行 + 结尾 NUL 最坏约 157 字节，低于 UART7 的 `TX_BUF_SIZE = 256`（`BSP/usart/usartConfig.hpp:22`）。格式化函数应在截断时返回 0，与现有 yaw/pitch 格式保持一致。

接口上建议：

- `encoder_telemetry.h` 增加 `WheelTelemetrySample`、`ChassisSnapshot`、`ChassisProvider` 和 `FormatChassisTelemetryLine()`。
- `encoder_serial_task.h` 增加 `EncoderTelemetryFormat { Pitch, Yaw }`，`CreateEncoderSerialTask` 增加默认参数保持兼容；另加 `CreateChassisSerialTask(ChassisProvider)`。
- 两个创建函数共用同一个 UART7 任务句柄与注册流程。

### 4.4 构建预设（`CMakePresets.json`）

当前只有 `Debug` / `Release`，且两者都依赖 CMake 的默认 `INFANTRY_BOARD_GIMBAL=1`，角色是隐式的。建议改为四个显式预设：

| 预设 | `CMAKE_BUILD_TYPE` | `INFANTRY_BOARD_GIMBAL` | 产物目录 |
|---|---|---|---|
| `Debug-Gimbal` | Debug | 1 | `build/Debug-Gimbal` |
| `Release-Gimbal` | Release | 1 | `build/Release-Gimbal` |
| `Debug-Chassis` | Debug | 0 | `build/Debug-Chassis` |
| `Release-Chassis` | Release | 0 | `build/Release-Chassis` |

`CMakeLists.txt:28` 是无 `FORCE` 的 cache 变量，预设传入的 `-DINFANTRY_BOARD_GIMBAL` 在首次配置时生效；`CMakeLists.txt:120` 会把定义传递给所有通过 `stm32cubemx` 链接的 C/C++ 编译单元。每个预设独立目录，避免切换角色时复用旧 cache。

### 4.5 最小修复建议

1. **`kWheelMaxSpeedDps` 占位**
   在 `ChassisConfig.h` 的几何表前定义 `inline constexpr float kWheelMaxSpeedDps = 0.0F;`，注释明确“未测量占位、不得据此推算能力”。0 不是实物值，只用于让代码通过编译；注册仍被 `kChassisHardwareConfirmed=false` 阻止。
   同时建议在 `ValidateConfiguration()` 的轮子循环中增加 `max_output_speed_dps > 0` 检查。原因是 `WheelVelocityPlanner::Configure` 不校验该项，`Update` 把它当作限速比例的分子（`chassis_control.h:199-201`）；若将来只把 `kChassisHardwareConfirmed` 改成 true 而忘记替换 0，四轮目标会被静默缩放为 0，表现为“配置成功但底盘不动”。

2. **拨弹盘反馈 ID 冲突**
   `kEnableFeeder` 改为 `false`，直到拨弹盘实际反馈 ID 确认。`kControlConfig.enable_feeder` 引用该常量，会同步关闭；左右摩擦轮仍按 `0x201/0x202` 注册。代价是单发/连发/堵转反转暂不可用，仅摩擦轮可转。

3. **显式包含 `<limits>`**
   `chassis.cpp:239` 使用 `std::numeric_limits<int16_t>`，但 `chassis.cpp`/`chassis_control.h`/`pid.h` 都没有包含 `<limits>`，目前依赖传递包含。建议在 `chassis.cpp` 显式补上，避免更换包含顺序后编译失败。

---

## 5. 底盘板在本阶段的行为

底盘板镜像启动后会执行：

1. `BSPLogInit()`、USB CDC 任务、IMU 任务、UART7 任务。
2. `safety::Init()` 创建安全任务，但没有 RC，`offline` 保持 true，`ShouldOutput()` 恒为 false。
3. `chassis::Init()` 调用 `ValidateConfiguration()`；`kChassisHardwareConfirmed=false`，记录日志并返回 false，不注册电机、不创建底盘任务。
4. 主循环继续运行；UART7 十二通道输出全零（因为没有底盘快照）。

即使将来填好底盘硬件参数、`chassis::Init()` 成功：

- `chassis::Task` 的 `remoteValid` 仍恒为 false（没有 RC）；
- `safety::ShouldOutput()` 仍恒为 false（没有遥控在线）；
- 底盘不会输出任何电流。

这是本阶段的**预期行为**，不是故障。底盘要动起来，必须等第二阶段把云台板的速度指令和安全状态送到底盘板，并重新定义 `chassis::Task` 的输入来源与超时策略。

---

## 6. 第二阶段边界

以下内容明确留给下一阶段，本设计不为它们预埋业务代码：

- 板间速度指令话题：云台板 → 底盘板（vx、vy、yaw 或轮速目标，二选一后固定语义）。
- 安全状态转发：使能、急停、遥控在线、超时；底盘板必须能在链路中断时独立撤输出。
- `ENABLE_INTERBOARD` 按角色启用；云台板使用 FDCAN1，底盘板使用 FDCAN2，双方 FD 时序一致。
- 底盘板 `chassis::Task` 的输入切换：从本地 RC 改为板间指令，保留本地 `safety::ShouldOutput()` 作为最终闸门。
- 底盘硬件参数确认与实车整定。

---

## 7. 验收清单（若按本设计落地）

> 仅构建与静态检查，不烧录、不接硬件。

1. 四个预设均能配置、编译、链接：
   `cmake --preset Debug-Gimbal && cmake --build build/Debug-Gimbal --parallel`，其余三个同理。
2. `build/*/CMakeCache.txt` 中 `INFANTRY_BOARD_GIMBAL` 分别为 1/0；`compile_commands.json` 中 `Robot.cpp` 与 `fdcan.c` 的 `-DINFANTRY_BOARD_GIMBAL` 与角色一致。
3. 在 Release 镜像上用 `arm-none-eabi-nm --demangle` 检查：
   - 底盘镜像无 `gimbal::Init`、`shoot::Init`；
   - 云台镜像无 `chassis::Init`、`chassis::Task`、`chassis::GetTelemetrySnapshot`。
4. 检查 `*.su` 栈用量：云台镜像含 GimbalTask/ShootTask 而无 ChassisTask，底盘镜像相反；两角色的 `configTOTAL_HEAP_SIZE` 静态断言仍成立。
5. 源码检查：`Robot.cpp` 各只有一处 `CreateEncoderSerialTask` / `CreateChassisSerialTask`，且都在角色 `if constexpr` 内；`chassis::Init()` 返回值被消费并记录；`Core/Src/fdcan.c` 仍保留三条 `ApplyBoardCanConfiguration`。
6. 文档与代码一致性：上述行号与行为在改动后重新核对。

---

## 8. 风险与注意事项

| 风险 | 说明 | 处理 |
|---|---|---|
| 角色镜像不可运行时识别 | 把底盘镜像烧到云台板（或反之）不会主动报错，只表现为 CAN 通信失败 | 烧录前核对预设、ELF 路径与板卡标签 |
| 底盘板长期静止 | 无 RC、无板间命令、硬件未确认 | 明确为本阶段预期；第二阶段实现转发后再验收运动 |
| `kWheelMaxSpeedDps=0` 被误当真实值 | 规划器不校验正数，可能静默输出 0 | 增加 `> 0` 校验；提高 `kChassisHardwareConfirmed` 前必须替换为实测值 |
| 拨弹盘暂时不可用 | `kEnableFeeder=false` 会关闭单发/连发 | 确认反馈 ID 后改回，并保留左右轮 `0x201/0x202` |
| 底盘板 `ENABLE_SAFETY=true` 但无 RC | 安全任务运行但永不 arm | 作为第二阶段接入板间安全状态的本地闸门；当前不影响启动 |
| `if constexpr` 的丢弃分支仍会做语义检查 | 非模板函数中，被丢弃分支必须可编译 | 所有模块源码保持双角色可编译；当前满足 |
| 文档与工作树漂移 | 本文描述的是推荐设计，当前代码仍是“全模块装配 + 编译失败” | 落地时同步更新本文与源码注释 |

---

## 9. 关键文件索引

| 文件 | 作用 |
|---|---|
| `Application/RobotConfig.hpp` | 角色枚举与 `ENABLE_*` 模块开关（建议修改点） |
| `Application/Robot.cpp` | `InfantryInit()` 装配入口、UART7 数据源选择（建议修改点） |
| `Application/chassis/ChassisConfig.h` | `kWheelMaxSpeedDps` 占位、底盘硬件参数与校验入口 |
| `Application/chassis/chassis.cpp` | `Init()` 非致命语义、`ValidateConfiguration()`、`GetTelemetrySnapshot()` 建议位置 |
| `Application/chassis/chassis.h` | `WheelControlSnapshot` 与底盘遥测适配接口 |
| `Application/shoot/ShootConfig.hpp` | 拨弹盘开关与反馈 ID（建议关闭拨弹盘） |
| `Application/shoot/shoot.cpp` | 电机注册与 `Error_Handler()` 启动路径 |
| `Application/gimbal/gimbal.cpp` | 云台 `Init()`、yaw/pitch 快照发布与读取 |
| `Modules/motor/dm/dmmotor.cpp` | DM 要求 FD 模式的校验 |
| `Modules/motor/dji/dji.cpp` | DJI 要求 CLASSIC 模式、重复反馈 ID 拒绝 |
| `Modules/communication/encoder_telemetry.h` | UART7 遥测格式（建议新增底盘十二通道） |
| `Modules/communication/encoder_serial_task.{h,cpp}` | UART7 发送任务与 provider 接口 |
| `Modules/safety/safety.cpp` | 安全默认状态与 `ShouldOutput()` |
| `Core/Inc/board_can_config.h` | 两板 CAN 模式表与板间控制器 |
| `CMakeLists.txt` | 角色 cache 变量、注入与 CubeMX 钩子校验 |
| `CMakePresets.json` | 建议新增四个角色预设 |
| `BSP/usart/usartConfig.hpp` | `TX_BUF_SIZE=256`，决定遥测行长上限 |

---

## 附：当前状态一句话总结

角色宏只切了 CAN 表，应用层仍把两块板当同一台车；本设计的全部工作就是把角色宏提升为“装配边界”，并顺手修掉两个会让镜像根本跑不起来的硬阻塞。
