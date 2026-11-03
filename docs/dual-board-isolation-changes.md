# 双板隔离与板间控制：实现记录

> 工程：RoboMaster 步兵 H723（`infantry`）
> 分支：`全向`　核对日期：2026-11-01
> 本文记录**实际落地**的改动与当前偏差；早期“按文件逐步修改”的草案已被本记录取代。架构说明见 [dual-board-isolation.md](dual-board-isolation.md)。

## 1. 提交索引

| 提交 | 内容 |
|---|---|
| `c47d0f4` `feat(board)` | 角色档案、`if constexpr` 装配、UART7 分流、四个构建预设、初版文档 |
| `2b20f41` `feat(chassis)` | 8 字节 `chassis::Command` 与底盘 RX 主题注册 |
| `2dfc313` `build(deps)` | vendored CMSIS-DSP 1.18.0 |
| `a8c9495` `feat(algorithm)` | C++20、CMSIS-DSP 链接、Kalman 滤波器头文件 |
| `ea647c1` `refactor(command)` | 底盘字段加 `com_` 前缀，新增 `gimbal::Command` 模式字段 |
| `5787104` `feat(gimbal)` | 云台 CAN 切到 FDCAN3，区分 aim/folding pitch 快照 |
| `3fde07b` `feat(chassis)` | 底盘硬件参数启用、电机 ID/减速比/运动限制、控制代码整理 |
| `a06459c` `fix(rc)` | 掉线判定 100ms → 1000ms，清理 DBUS 死代码 |
| `45aeea1` `fix(interboard)` | 电机 TX FIFO 保留槽 2 → 4，ISR 丢帧计数加临界区 |
| `2835010` `底盘不受遥控器控制` | 底盘改为命令驱动的基础改动 |
| `eeefba5` `feat(board)` | 底盘速度指令 + 安全状态经板间转发；底盘不再读 RC/本地安全 |
| `fed5f36` `fix(gimbal)` | 云台板 UART7 切换到折叠 pitch 反馈快照 |
| `51f0d60` `style(bsp,rc)` | IMU/UART/DBUS 注释与接口整理 |

## 2. 与最初方案的偏差

最初方案是“只做模块装配隔离，底盘保持静止”。实际代码已经推进到“云台板转发速度与安全状态，底盘板消费命令”：

| 项目 | 最初方案 | 当前实现 |
|---|---|---|
| 底盘运动 | 本阶段不动，第二阶段再接 | 已接入命令驱动，NoFollow/Direct/Spin 可用 |
| 底盘命令 | 8 字节：3×int16 + uint16 | **10 字节**：3×int16 + mode + reserved + uint16 |
| 安全状态 | 未定义 | 独立 topic 2，4 字节 flags + sequence |
| `ENABLE_SAFETY` | 两板都保留 | 仅云台板；底盘消费转发状态 |
| UART7 云台源 | 方案建议 yaw，实际曾用 aim pitch | 当前用 **folding pitch** 快照（折叠 pitch 仍禁用） |
| `kWheelMaxSpeedDps` | 建议 0 占位并校验 | 已由真实配置取代：`kChassisHardwareConfirmed=true`，轮速上限 50 °/s |
| 拨弹盘 | 建议关闭 | 仍为 `kEnableFeeder=true`，`0x201` 与左摩擦轮重复（未修复） |
| 板间 TX 保留槽 | 2 | 4 |
| RC 掉线 | 100ms | 1000ms |
| 云台 CAN | FDCAN1 | FDCAN3 |

## 3. 当前文件职责

### 3.1 角色与装配

- `Application/RobotConfig.hpp`
  - `BoardRole` 派生 RC/SAFETY/GIMBAL/SHOOT/CHASSIS；
  - `ENABLE_SAFETY` 仅在云台板为 true；`static_assert` 锁定“安全状态由云台板产生”；
  - `ENABLE_INTERBOARD=true`，`ENABLE_BUZZER=false`（共享开关）。
- `Application/Robot.cpp`
  - `if constexpr` 装配；
  - 云台/发射/链路失败致命，底盘失败记录日志；
  - UART7：云台板 `GetFoldingPitchMotorFeedbackSnapshot`，底盘板 `GetTelemetrySnapshot`。

### 3.2 板间协议

`Application/RobotCommand.hpp`：

- `chassis::Mode`：`Disabled/NoFollow/FollowGimbal/Spin/Direct`；
- `chassis::Command`：10 字节，静态断言锁定布局；
- `chassis::SafetyState`：4 字节；
- command topic `1` / name `chassis/command` / timeout 100ms；
- safety topic `2` / name `chassis/safety` / timeout 100ms；
- safety flags：OutputEnabled / RcOnline / EStopped / BothDown / RunRequested。

### 3.3 云台板发布

`Application/gimbal/gimbal.cpp`：

- `GimbalTask` 每 10 个 tick 发布一次命令和安全状态；
- 命令由 RC 归一化、去死区、按 `kChassisMotionLimits` 缩放后量化；
- `SelectChassisMode()` 当前只返回 `NoFollow`；Spin/FollowGimbal/Direct 的条件仍为注释；
- `gimbal::Init()` 注册 TX 主题并保存 publisher；
- 发布只在 `interboard::Start()` 成功后可用。

### 3.4 底盘板消费

`Application/chassis/chassis.cpp`：

- `Init()` 注册 command/safety 两个 RX 主题并保存 subscriber；
- `Task()` 取最新命令/安全状态，检查 100ms 新鲜度；
- 安全许可 = `(flags & (OutputEnabled|RcOnline|RunRequested)) == 全部` 且 `(flags & (EStopped|BothDown)) == 0`；
- `ModeTargets()`：Disabled 停机；NoFollow/Direct 直通；Spin 使用 6±3 °/s 正弦；FollowGimbal 未实现；
- 任一条件失败 → `StopWheels()`；四轮反馈未就绪同样停机；
- 不再包含 `rc/rc.h` 或 `safety/safety.h`。

### 3.5 配置

- `Application/chassis/ChassisConfig.h`
  - `kChassisHardwareConfirmed=true`；
  - M3508 `0x201..0x204`、reverse、17:1、输出轴 50 °/s；
  - 平移 5 m/s、yaw 90 °/s、加速度 3 / 45；
  - Spin 偏置 6、幅值 3、频率 0.2Hz。
- `Application/gimbal/GimbalConfig.hpp`
  - CAN3；aim pitch `0x205`，yaw `0x206`；
  - folding pitch master 3 / feedback `0x206`，`kEnableFoldingPitch=false`。
- `Modules/rc/rc.h`：`OFFLINE_TIMEOUT_MS=1000`。
- `Modules/communication/interboard.cpp`：`kMotorReservedSlots=4`，`rxDrops` 在临界区内累加。

### 3.6 构建与依赖

- `CMakeLists.txt`：C++20；`INFANTRY_CMSIS_DSP_PATH`；`add_subdirectory(.../CMSIS-DSP-1.18.0/Source)`；链接 `CMSISDSP`。
- `CMakePresets.json`：`Debug/Release-Gimbal`、`Debug/Release-Chassis`。
- `Modules/algorithm/KalmanFilter/`：已加入头文件，尚未被控制回路调用。

## 4. 验证

```bash
cmake --preset Debug-Gimbal
cmake --build build/Debug-Gimbal --parallel
cmake --preset Debug-Chassis
cmake --build build/Debug-Chassis --parallel
```

本次核对在 `build/verify-gimbal` 与 `build/verify-chassis` 完成：

- 两套角色均配置、编译、链接通过；
- 底盘板包含 `RegisterRxTopic`/`SubRegister`/`chassis::Init`，云台板包含 `interboard::Start` 与云台发布路径；
- 未烧录、未接电机、未验证 DR16/CAN/UART7 实物行为。

## 5. 已知问题

| 问题 | 影响 | 建议 |
|---|---|---|
| 拨弹盘 `kEnableFeeder=true` 且 ID 与左摩擦轮重复 | 云台板 `shoot::Init()` 在调度器启动前 `Error_Handler()`，整板无法启动 | 确认实际 ID 或暂时关闭拨弹盘 |
| 折叠 pitch 禁用但 UART7 取其快照 | 云台板 UART7 为 0，不能诊断 | 启用前实现快照发布并核对 ID |
| 折叠 pitch 反馈 ID 与 yaw 同为 `0x206` | 同时注册会被 DM 驱动拒绝 | 重新分配反馈 ID |
| `FollowGimbal` 未实现 | mode=2 时底盘停机 | 补充云台 yaw 反馈/目标 |
| 安全状态只来自云台板 | 链路/云台任务中断 → 底盘 100ms 后停机 | 当前失效安全策略；需要本地降级时另行设计 |
| 底盘轮速/方向/PI 未实测 | 可能反向、振荡或限幅不足 | 按 `ChassisConfig.h` 上板逐项核对 |
| UART7 云台数据为零 | 折叠 pitch 未运行时无有效数据 | 先确认使用 aim pitch 还是 folding pitch |

## 6. 后续优先级

1. 解决拨弹盘 `0x201` 重复，保证云台板能启动到调度器。
2. 确认云台板 UART7 到底要观察 aim pitch 还是 folding pitch；若继续 folding，先启用并发布快照。
3. 实现 `FollowGimbal`（需要云台 yaw 角度/目标进入命令）。
4. 上板核对底盘四轮方向、减速比、轮速 PI 与 Spin 参数。
5. 评估安全状态是否需要在底盘板保留本地降级路径。
