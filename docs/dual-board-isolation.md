# 双板隔离与板间控制架构（当前实现）

> 工程：RoboMaster 步兵 H723（`infantry`）
> 分支：`全向`　核对日期：2026-11-01
> 本文描述当前代码的实际装配与控制链路，不是待实施计划。构建验证见第 8 节；尚未烧录实车。

## 1. 一句话结论

板卡角色决定装配哪些模块；**云台板产生底盘速度指令与安全状态，经板间链路转发；底盘板只消费命令，不再依赖本地 DR16 或本地安全任务**。底盘运动的安全链因此跨越两块板，任何一侧超时都会在底盘侧停机。

## 2. 角色矩阵

| 开关 | 云台板（`INFANTRY_BOARD_GIMBAL=1`） | 底盘板（`=0`） | 说明 |
|---|---|---|---|
| `ENABLE_USB_SERIAL` | true | true | 两板保留 USB CDC |
| `ENABLE_IMU` | true | true | 底盘板保留姿态 |
| `ENABLE_RC` | true | false | DR16 只接云台板 |
| `ENABLE_SAFETY` | true | false | 安全状态由云台板产生并转发；底盘板不再运行本地安全任务 |
| `ENABLE_GIMBAL` | true | false | 云台 DM 电机只在云台板注册 |
| `ENABLE_SHOOT` | true | false | 摩擦轮/拨弹盘只在云台板注册 |
| `ENABLE_CHASSIS` | false | true | 底盘 M3508 只在底盘板注册 |
| `ENABLE_ENCODER_UART7` | true | true | 云台板输出折叠 pitch 快照；底盘板输出四轮十二通道 |
| `ENABLE_INTERBOARD` | true | true | 两板都启动板间任务 |
| `ENABLE_BUZZER` | 共享开关（当前 false） | 共享开关（当前 false） | 不按角色禁用 |

`RobotConfig.hpp` 由 `BoardRole` 派生 RC/SAFETY/GIMBAL/SHOOT/CHASSIS，并用 `static_assert` 锁定角色与开关的一致性。`Robot.cpp` 使用 `if constexpr` 装配；云台/发射/链路失败是致命的，底盘 `Init()` 失败只记录日志。

## 3. CAN 资源

| 控制器 | 云台板 | 底盘板 |
|---|---|---|
| FDCAN1 | FD，板间链路（`INFANTRY_INTERBOARD_FDCAN=1`） | CLASSIC，四轮 M3508 |
| FDCAN2 | CLASSIC，摩擦轮/拨弹盘 | FD，板间链路（`INFANTRY_INTERBOARD_FDCAN=2`） |
| FDCAN3 | FD，云台 DM（`kGimbalCanHandle=&hfdcan3`） | CLASSIC，空闲 |

- 板间心跳：云台板发 `0x301` / 收 `0x302`；底盘板发 `0x302` / 收 `0x301`。
- 板间发送保留 **4 个 TX FIFO 槽**给同路电机；每 2ms 最多发 2 帧（含心跳）。
- FD 单帧格式为 4 字节头（topic/sequence/length/version `0xA1`）+ 最多 60 字节业务；经典 CAN 走 6 字节分片兼容路径。

## 4. 板间主题

消息中心是**最新值邮箱**，不提供确认/重放；执行层必须自行检查超时。

### 4.1 底盘速度指令（topic 1，10 字节）

主题名：`chassis/command`。发送侧云台板，接收侧底盘板。

| 字段 | 类型 | 单位/含义 |
|---|---|---|
| `com_vx_mmps` | int16 | 车体前向速度，mm/s（m/s × 1000） |
| `com_vy_mmps` | int16 | 车体左向速度，mm/s |
| `com_yaw_ddps` | int16 | 车体偏航角速度，0.1 °/s（°/s × 10） |
| `mode` | uint8 | `chassis::Mode` |
| `reserved` | uint8 | 发送方置 0，显式填充 |
| `sequence` | uint16 | 发送侧递增，仅用于诊断丢帧 |

`chassis::Mode`：`Disabled=0`、`NoFollow=1`、`FollowGimbal=2`、`Spin=3`、`Direct=4`。
命令超时：`kCommandTimeoutMs = 100ms`。

### 4.2 安全状态（topic 2，4 字节）

主题名：`chassis/safety`。云台板周期性发布，底盘板订阅。

| 字段 | 类型 | 含义 |
|---|---|---|
| `flags` | uint16 | 见下表 |
| `sequence` | uint16 | 发送侧递增 |

| 位 | 名称 | 来源 |
|---|---|---|
| bit0 | `kSafetyOutputEnabled` | 云台板 `safety::ShouldOutput()` |
| bit1 | `kSafetyRcOnline` | 云台板 `rc::IsOnline()` |
| bit2 | `kSafetyEStopped` | 云台板急停状态 |
| bit3 | `kSafetyBothDown` | 双拨杆 DOWN |
| bit4 | `kSafetyRunRequested` | 底盘运行拨杆请求 |

安全状态超时：`kSafetyTimeoutMs = 100ms`。

### 4.3 时序

- 心跳 100ms；300ms 无有效链路帧判离线。
- 云台板 `GimbalTask` 每 10 个 tick（1kHz 下 10ms）发布一次命令和安全状态。
- 底盘板 1kHz 任务每周期 `SubGetMessage()` 取最新值。
- 命令或安全状态超过 100ms 未刷新，底盘立即撤输出。

## 5. 数据流

### 5.1 云台板：产生命令与安全状态

`gimbal.cpp` 在 `GimbalTask` 内：

1. 读取 RC，按 `kChassisForward/Left/YawChannel` 归一化、去死区，映射到 `kChassisMotionLimits`。
2. `outputEnabled = kChassisOutputMode && safety::ShouldOutput()`；`runRequested = 左拨杆 == kChassisRunSwitch`。
3. `SelectChassisMode()` 当前返回 `NoFollow`；Spin/FollowGimbal/Direct 的切换条件仍为注释状态。
4. 量化后 `PubPushMessage(chassis/command)`；同时把输出许可、RC 在线、急停、双 DOWN、运行请求打包为 `SafetyState` 发布。
5. 只有 `interboard::RegisterTxTopic()` 和 `PubRegister()` 都成功的主题才会发布；失败只记录日志。

这意味着：**云台任务不运行（例如云台初始化失败）时，底盘收不到任何命令或安全状态，会在 100ms 内停机。**

### 5.2 底盘板：消费命令与安全状态

`chassis.cpp` 的 `Task()` 每周期：

1. `SubGetMessage()` 读取命令和安全状态，记录本地到达 tick。
2. `commandFresh` / `safetyFresh` 分别检查 100ms 超时。
3. 安全状态必须同时满足：
   - 置位 `OutputEnabled | RcOnline | RunRequested`；
   - 清零 `EStopped | BothDown`。
4. `ModeTargets()` 生成 planner 输入：
   - `Disabled`：停机；
   - `NoFollow` / `Direct`：使用命令中的 vx/vy/yaw；
   - `Spin`：平移仍用命令，yaw = `kSpinBiasDps + kSpinAmplitudeDps*sin(2π·f·t)`；
   - `FollowGimbal`：尚未实现，返回 false 并停机。
5. 任一条件失败（时间异常、命令/安全超时、模式未实现、CAN 入队失败、四轮反馈未就绪）→ `StopWheels()`。
6. 发送四轮速度目标并发布轮速快照。

底盘板**不再读取 RC**，也不再调用 `safety::ShouldOutput()`；它的全部运动许可来自云台板转发的安全状态。

## 6. 底盘配置（当前值）

`Application/chassis/ChassisConfig.h`：

- `kChassisHardwareConfirmed = true`；
- 四轮 M3508：反馈 ID `0x201..0x204`，`reverse=true`，减速比 17；
- 输出轴最大轮速 50 °/s；
- 平移上限 5 m/s，yaw 上限 90 °/s，平移加速度 3 m/s²，yaw 加速度 45 °/s²；
- Spin：偏置 6 °/s，幅值 3 °/s，频率 0.2Hz（6±3 °/s）。

`Application/gimbal/GimbalConfig.hpp`：

- 云台 CAN 已切到 FDCAN3；
- aim pitch 反馈 `0x205`，yaw 反馈 `0x206`；
- folding pitch master `3`、反馈 `0x206`，但 `kEnableFoldingPitch=false`，当前不注册、不控制；
- 折叠 pitch 反馈 ID 与 yaw 相同，启用前必须重新分配。

`Modules/rc/rc.h`：遥控掉线判定为 **1000ms**。

## 7. UART7

- 云台板：`Robot.cpp` 当前传入 `gimbal::GetFoldingPitchMotorFeedbackSnapshot`。折叠 pitch 处于禁用状态，该快照目前不会被写入，因此云台板 UART7 会输出零；旧的 aim pitch 行仍保留为注释。
- 底盘板：`GetTelemetrySnapshot()` 输出 FL/FR/BL/BR ×（目标轮速、实际轮速、反馈电流），共 12 列；未初始化时为 0。

## 8. 构建与验证

```bash
cmake --preset Debug-Gimbal
cmake --build build/Debug-Gimbal --parallel

cmake --preset Debug-Chassis
cmake --build build/Debug-Chassis --parallel
```

- `CMakeLists.txt` 使用 C++20，并 vendored `Middlewares/Third_Party/CMSIS-DSP-1.18.0`；Kalman 滤波器头文件已加入工程，但尚未接入控制回路。
- 本次核对使用 `build/verify-gimbal` / `build/verify-chassis` 完成链接验证；未烧录、未接实车、未验证 CAN 波形、UART7 接线或 DR16 行为。

## 9. 已知问题与风险

| 问题 | 影响 | 处理方向 |
|---|---|---|
| 拨弹盘 `kEnableFeeder=true`，反馈 ID `0x201` 与左摩擦轮重复 | 云台板 `shoot::Init()` 在调度器启动前 `Error_Handler()`，整板无法启动 | 确认拨弹盘实际 ID 或暂时关闭 `kEnableFeeder` |
| 折叠 pitch 禁用，UART7 却取其快照 | 云台板 UART7 当前为 0，不能作为诊断依据 | 启用折叠 pitch 前先实现快照发布并核对反馈 ID |
| `FollowGimbal` 模式未实现 | 底盘收到 mode=2 会停机 | 在命令中补充云台 yaw 反馈/目标后再实现 |
| 安全状态只由云台板产生 | 板间链路中断或云台任务停摆时，底盘在 100ms 后停机 | 这是当前失效安全策略；如需本地降级，需另行设计 |
| 折叠 pitch 反馈 ID 与 yaw 相同（0x206） | 同时注册时 DM 驱动会拒绝重复 ID | 启用折叠 pitch 前重新分配 |
| 未做实物验证 | 电机方向、CAN 时序、轮速 PI、UART7 接线均未确认 | 按 `ChassisConfig.h`/`GimbalConfig.hpp` 逐项上板核对 |

## 10. 关键文件

| 文件 | 作用 |
|---|---|
| `Application/RobotConfig.hpp` | 角色与模块开关 |
| `Application/Robot.cpp` | 装配入口、UART7 路由 |
| `Application/RobotCommand.hpp` | 底盘命令、模式、安全状态、主题 ID/超时 |
| `Application/gimbal/gimbal.cpp` | 命令/安全状态发布、云台控制 |
| `Application/chassis/chassis.cpp` | 命令/安全状态订阅、模式分发、四轮控制 |
| `Application/chassis/ChassisConfig.h` | 底盘硬件、运动限制、Spin 参数 |
| `Application/gimbal/GimbalConfig.hpp` | 云台 CAN、电机 ID、折叠 pitch 配置 |
| `Modules/communication/interboard.cpp` | 板间任务、心跳、TX FIFO 保留槽 |
| `Modules/communication/interboard.md` | 线路格式与主题说明 |
| `Core/Inc/board_can_config.h` | 两板 CAN 模式表与板间控制器 |
| `CMakePresets.json` | 四个角色构建预设 |
