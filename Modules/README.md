# Modules 层文档

Modules 层是"能力"层: 提供与应用无关的驱动、算法和服务, 不含整车业务逻辑。每个模块只依赖 BSP 和更底层的 Modules, 应用层 (`Application/`) 通过各模块的命名空间接口使用它们。

## 任务与中断总览

### FreeRTOS 任务 (均由各模块 Init 时创建)

| 任务名 | 优先级 | 周期/阻塞点 | 栈 | 源文件 | 说明 |
|---|---|---|---|---|---|
| GimbalTask | High | 1ms | 2048 | `Application/gimbal/gimbal.cpp` | 云台两轴控制 (属应用层, 列此备查) |
| SafetyTask | AboveNormal | 1ms | 1024 | `Modules/safety/safety.cpp` | 急停判定 + 掉线监控 |
| RCTask | AboveNormal | 1ms 轮询 | 2048 | `Modules/rc/rc.cpp` | 仅做超时判定, 解析在 UART 中断内 |
| InterboardTask | AboveNormal | 2ms 轮询 | 2048 | `Modules/communication/interboard.cpp` | 双板帧处理、心跳、话题转发 |
| UartSerialTask | Normal | 阻塞于队列 | 2048 | `Modules/communication/uart_serial_task.cpp` | UART7 回环 |
| UsbSerialTask | Normal | 约 10ms | 2048 | `Modules/communication/usb_serial_task.cpp` | USB CDC 姿态角发送 |
| ImuTask | AboveNormal | 每轮采样后间隔约 2ms | 3072 | `Modules/imu/ImuTask.cpp` | BMI088 六轴采样与姿态解算 |
| BuzzerTask | Normal | 1s 鸣响 / 1s 静音 | 1024 | `Modules/BUZZER/buzzer_task.cpp` | 周期提示音 |
| Bmi088Heater | BelowNormal | ~1.3s (温度寄存器刷新率) | 2048 | `Modules/imu/Bmi088Heater.cpp` | IMU 加热闭环 (当前未接入整车初始化) |

### 中断上下文 (ISR 内只做解析/复制/入队, 禁止阻塞)

| 中断源 | 处理函数 | 下游 |
|---|---|---|
| FDCAN1 RX FIFO0 | `dji.cpp` `MotorFeedbackCallback` | 直接更新电机反馈结构 (含 total_angle 解卷绕) |
| FDCAN2 RX FIFO0 | `interboard.cpp` `OnCanRx` | 帧入 osMessageQueue, 交给 InterboardTask |
| UART5 DMA+Idle | `rc.cpp` `OnReceive` | DBUS 帧解析, 标量写入 `rc::RC` |
| UART7 DMA+Idle | `uart_serial_task.cpp` `OnReceive` | 数据分包入队 |
| USB CDC OUT | `usbd_cdc_if.c` `CDC_Receive_HS` | 重新准备接收缓冲区，不回传收到的数据 |
| SPI2 DMA 完成/错误 | `Bmi088.cpp` HAL 回调 | 释放信号量唤醒等待任务 |

---

## algorithm/pid — 通用 PID

`pid.h` / `pid.cpp`。与 HAL、设备完全无关, 可在宿主机上单独测试 (`tests/pid_test.cpp`)。

- **算法**: P + I + D(作用于测量值)。微分取测量而非误差, 目标阶跃不产生微分冲击。
- **抗饱和**: 积分独立限幅 (`integral_min/max`) + 条件积分 — 输出饱和且误差与饱和同向时暂停积分, 反向后自动恢复。
- **参数校验**: `Configure()` 拒绝 NaN、输出区间不含 0 等非法配置; "全零参数"在电机场景表示该环停用 (见 `motor_common.cpp`)。
- **失败安全**: `Update()` 遇到未配置/NaN/dt≤0 时返回 0 并清空状态。
- 约定: `dt_s` 单位秒; 电机角度环单位 度/°每秒, 电流环为指令码, 加热环为 °C/占空比。

## motor/ — 电机驱动

### dji.cpp / dji.h — DJI 电调驱动 (核心)

支持 GM6020 (0x1FF/0x2FF 电压指令, 反馈 0x205+)、M3508/M2006 (0x200 电流指令, 反馈 0x201+)。

- **反馈路径**: CAN 中断 → `MotorFeedbackCallback` 校验 8 字节反馈帧，解析机械角、转速、电流及可用时的温度 → 写入 `motor->feedback`。M2006/C610 的末两字节为保留字节，`temperature = 0` 表示没有温度反馈。回调内完成:
  - 单圈角度 → 度; RPM → °/s (`MOTOR_RPM_TO_DEGREES_PER_SECOND`);
  - **连续角度** `total_angle`: 用 `MotorEncoderDelta` 对相邻反馈解卷绕累加;
  - **重锚机制**: 首帧或 >100ms 无反馈时, `total_angle` 重置为当前单圈角、`angle_epoch` 递增; 若电机正跑角度环则自动 `Disable`, 逼迫上层重新同步位置目标。这是位置模式安全性的基石。
- **控制路径**: `DJI_Motor_Control(motor_list, count, dt_s)` 按型号分组, 逐电机算 16 位输出并打包进指令帧 (一帧最多 4 台, 槽位由反馈 ID 决定); 掉线/未使能电机按零输出打包, 不影响同帧其他电机。
- **实例管理**: 16 槽静态池, 无动态分配; CAN 回调经 `module_handle` 直达实例。
- 上层职责: 每周期写 `motor_settings.speed`/`.angle` (或 `.current`), 并用 Enable/Disable 控制输出; 反馈 ID 必须唯一且在型号区间内。

### motor_common.h — 共享类型

闭环组合 (`Closeloop_Type_e`)、方向/反馈源/前馈开关、`Motor_Control_Setting_s` (上层每周期写入的期望值)、`Motor_Feedback_s` (含 `angle_epoch`/`angle_initialized`/`last_update_tick`)、`Motor_Controller_InitPids()` (三环 PID 装配, 全零参数 = 该环停用)。

### angle_target.h / encoder_angle.h — 位置目标 (纯头文件, 可宿主机测试)

- `MotorEncoderDelta(prev, cur)`: 13 位编码器跨 0/8191 边界的最短增量 (±4096 内); 两次反馈间转动超过半圈则无法判定 (驱动层用超时重锚兜底)。
- `MotorAngleTarget`: 位置模式的目标管理器。
  - `Sync()`: 把目标对齐当前实测角度 (启动不自寻零), 校验零点标定与行程, 记录 `angle_epoch`;
  - `Step()`: 按摇杆速率积分目标, 非连续轴限幅在 `[zero+min, zero+max]`;
  - `Contains()`: 运行中持续校验是否越出行程, 越界由上层 Disable。
- `MotorAngleLimits`: 行程标定结构 (`calibrated`/`zero_raw`/`min_deg`/`max_deg`/`continuous`), `IsConfigured()` 保证未实测的行程无法被启用。

### other_motors.h — 非 DJI 电机 (预留)

达妙 (DM)、MIT 模式、LK9025、HT04 的参数结构。协议与 DJI 不同, 接入时需另写驱动, 不复用 `DJI_Motor_Control()`。当前仅 `DMMotor_Controller_InitPids()` 有实现。

## rc — 遥控器 (DT7 / DR16 / DBUS)

- **链路**: UART5, 100000 波特, 9 位数据+偶校验 (等价 8E1, 与 SBUS 一致, 勿按 8N1 使用)。BSP 注册 DMA+Idle 接收。
- **解析**: 帧定位 = 起始字节 0x0F + 结束字节辅助校验; 11 位小端比特流解 16 通道; 通道 0-3 摇杆 (364..1684 → [-1,1]), 通道 4/5 拨杆三档, 通道 6 键鼠合并位段。解析在 **UART 中断内**完成, `last_update_tick` 最后写入 + release/acquire fence 保证任务侧读到完整帧。
- **掉线**: `IsOnline()` = 100ms 内收到过合法帧; 全局状态 `rc::RC` 供所有模块读取。
- **自恢复**: 接收缓冲区被写满说明帧边界丢失 (上电瞬间/波特率不符), 自动重启接收链路。
- 上层接口: `RightHorizontal/RightVertical/LeftVertical/LeftHorizontal()`、`RC.switch_left/switch_right`、`IsOnline()`。

## safety — 急停与输出使能

- **触发**: 左右拨杆**同时** DOWN, 经 3ms 去抖 (状态跳变重新计数, 延迟固定)。
- **解除**: 任一拨杆离开 DOWN 组合即立即恢复, 无需额外操作。
- **掉线**: 遥控失联直接撤销输出, 恢复收帧后自动重新判定。
- **接口**: `ShouldOutput()` 是所有执行器输出的总开关 (上电首次判定前返回 false); `IsEStopped()`/`BothDown()` 供状态展示。
- 上电默认: 拨杆不在 DOWN 组合即为放行状态。

## communication/ — 通信

### interboard — 双板 CAN 链路 (详见 [interboard.md](communication/interboard.md))

- 独占一路 FDCAN (`INFANTRY_INTERBOARD_FDCAN` 选择), 经 `CAN_SetDedicatedRxHandler` 接管接收, 不占用 CAN_ 模板过滤器表。
- 协议: 云台→底盘 0x301, 底盘→云台 0x302; 字节 0 = 话题 ID (0 = 心跳), 字节 1 = 序号(高4位)+分片号(低4位), 每片负载 ≤6 字节; 话题定长 ≤48 字节。
- 接收: 按序号+分片连续+50ms 限时重组; 无应用层确认/重发, 依赖消息中心"最新值邮箱"容忍丢帧。
- 链路状态: 收帧即在线, 300ms 无帧离线; 经 `interboard/link` 话题 (LinkStatus) 发布, **应用须自行实现失联保护**。
- 话题注册: 必须在 `interboard::Start()` 之后、调度器启动之前; 两板同 ID 同长度, 一端 Tx 一端 Rx。

### message_center — 板内发布/订阅 (第三方, 来自 NeoZeng 的 RM 框架)

C 实现, 最新值邮箱 (队列深度 1): `PubRegister/PubPushMessage` 发布, `SubRegister/SubGetMessage` 订阅。发布快于消费时覆盖旧值 — 使用方必须意识到读到的是"最新值"而非"每条消息"。全局话题表最多 12 个, 话题名 ≤32 字符。interboard 的跨板话题即构建在它之上。

### usb_serial_task / uart_serial_task — 串口链路自测

USB CDC 在主机完成配置后，约每 10ms 发送一行 `Roll=+0.00,Pitch=+0.00,Yaw=+0.00`（单位：度）。没有最近 100ms 的有效姿态时，每秒发送一次 `IMU not ready`；收到的 CDC 数据不回传。UART7 仍使用"收 → 队列 → 原样发回"验证链路。

## imu/ — 惯性测量单元

### Bmi088 — 芯片驱动 (部分实现)

SPI2 共总线 + 双片选 (ACC/GYRO), DMA 传输, 信号量同步, 互斥锁保护总线; 加速度计读时序多一个哑字节。收发缓冲区置于 `.axi_bss` (DMA 不可访问 DTCM 栈, 见 BSP 文档内存布局节)。
已实现: 芯片 ID 识别、温度读取、加速度计 ±3g/800Hz 与陀螺仪 ±2000°/s/1000Hz 配置、三轴六字节连续读取及物理单位换算。`ImuTask` 调用四元数互补估计器输出 Roll/Pitch/Yaw；Yaw 以上电朝向为零，没有磁力计或外部航向源时会漂移。芯片轴与整车轴的对应关系需上机核对。

### Bmi088Heater — 加热温控 (已实现, 未接入整车)

PID (0~1 占空比输出) → TIM3 CH4 PWM → 加热片; 目标温度必须低于截止温度 (≤85°C), 读温失败/过温即停机。`Start()` 会阻塞等待约 1.3s 首笔温度, 只能在任务中调用。接入整车时需在 `RobotConfig.hpp` 增加开关并在 `InfantryInit()` 调用 `bmi088Heater::CreateTask()`。

## BUZZER / LED — 人机交互

- **BUZZER**: BSP 层 TIM12 CH2 PWM (`BuzzerOn/Off/SetDuty`), 任务层周期鸣响 (1s/1s)。`ENABLE_BUZZER` 当前为 false。
- **LED**: 规划保留在 [LED/README.md](LED/README.md)，目前没有任务实现或初始化开关。

## 模块开发约定

1. **ISR 最小化**: 中断回调只做解析、标量写入或入队; 跨上下文数据用"最后写 tick + fence"或消息队列同步, 不自旋。
2. **失败安全**: 一切校验失败时输出收敛到零 (PID 返回 0、电机 Disable), 宁可不动不可乱动。
3. **静态分配**: 任务栈/实例池/队列容量全部编译期确定, 运行期无 malloc。
4. **参数集中**: PID 初值、ID、限幅等放各 `*Config.hpp` 或头文件常量, 实车整定只改一处。
