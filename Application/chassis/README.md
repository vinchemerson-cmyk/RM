# 底盘控制（已实现）

底盘模块只在底盘板装配（`INFANTRY_BOARD_GIMBAL=0`），使用 FDCAN1 Classic 控制四台 M3508。

## 当前配置

- 反馈 ID：FL `0x201`、FR `0x202`、BL `0x203`、BR `0x204`，全部 `reverse=true`，减速比 17；
- 输出轴最大轮速 50 °/s；平移上限 5 m/s，yaw 上限 90 °/s；
- 平移加速度 3 m/s²，yaw 加速度 45 °/s²；
- Spin：偏置 6 °/s、幅值 3 °/s、频率 0.2Hz；
- 参数集中在 `ChassisConfig.h`，`kChassisHardwareConfirmed=true` 才会注册电机。

## 指令与安全

底盘板**不读 DR16、不运行本地安全任务**，由云台板经板间链路发布：

- topic 1 `chassis/command`（10 字节）：vx/vy（mm/s）、yaw（0.1 °/s）、mode、reserved、sequence；
- topic 2 `chassis/safety`（4 字节）：OutputEnabled/RcOnline/EStopped/BothDown/RunRequested。

`chassis::Init()` 在调度器启动前注册两个 RX 主题；`Task()` 每周期检查 100ms 新鲜度。
安全许可要求 `OutputEnabled|RcOnline|RunRequested` 全部置位，且 `EStopped|BothDown` 全部清零。
任一条件失败、模式未实现、CAN 入队失败或四轮反馈未就绪都会 `StopWheels()`。

模式：`Disabled` 停机；`NoFollow`/`Direct` 直通命令速度；`Spin` 使用正弦自旋；`FollowGimbal` 尚未实现。

## UART7

`GetTelemetrySnapshot()` 输出 FL/FR/BL/BR ×（目标轮速、实际轮速、反馈电流），共 12 列；未初始化时为 0。

## 构建

```bash
cmake --preset Debug-Chassis
cmake --build build/Debug-Chassis --parallel
```

## 已知限制

- 底盘硬件参数尚未实车核对，首次上板先检查四轮方向、轮速与电流限幅；
- 命令/安全状态完全依赖板间链路，链路或云台任务中断时底盘在 100ms 内停机；
- `FollowGimbal` 需要云台 yaw 反馈/目标，当前收到该模式会停机。
