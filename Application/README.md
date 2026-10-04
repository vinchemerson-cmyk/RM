# Application 层文档

Application 层是"整车业务"层: 决定**这台机器人要做什么**。它向下调用 `Modules/` 提供的驱动与算法, 通过 `BSP/` 访问硬件, 自身不直接操作寄存器。与 CubeMX 生成代码 (`Core/`) 的唯一边界是 `Robot.h` 暴露的 C 接口 `InfantryInit()`。

```
Core/Src/main.c        外设初始化 (CubeMX 生成)
  └─ freertos.c        MX_FREERTOS_Init(): 创建 defaultTask, 调用 InfantryInit()
       └─ Robot.cpp    InfantryInit(): 按 RobotConfig.hpp 开关装配各模块
            └─ osKernelStart() 后, 各模块任务并行运行
```

## 目录

| 文件/目录 | 职责 | 状态 |
|---|---|---|
| `Robot.h` / `Robot.cpp` | 整车初始化入口, 装配各模块任务 | 已实现 |
| `RobotConfig.hpp` | 板卡角色、FDCAN 分配、模块启停开关 (编译期) | 已实现 |
| `gimbal/` | 云台控制及编码器快照 | pitch单调速度环（端点方向保护），yaw连续位置控制 |
| `chassis/` | 底盘控制规划，见目录内 README | 尚未实现 |
| `shoot/` | 两台 M3508 摩擦轮；拨弹代码保留但不注册电机 | 双轮测试启用，拨弹关闭 |

## RobotConfig.hpp — 编译期配置

烧录前必须核对的两件事:

1. **板卡角色**: `INFANTRY_BOARD_GIMBAL` — 云台板为 `1`, 底盘板为 `0`, 两块板必须相反, 否则双板链路永远离线 (互相收不到心跳)。
2. **FDCAN 分配** (H723 共 3 路):

| 控制器 | 用途 | 配置位置 |
|---|---|---|
| FDCAN1 | pitch=0x205 (ID1)速度调试、yaw=0x206 (ID2)位置控制；Classic CAN 1Mbps | `GimbalConfig.hpp` |
| FDCAN2 | 左摩擦=0x201/右摩擦=0x202 (用户确认 ID1/ID2)，Classic CAN 1Mbps | `ShootConfig.hpp`；双板通信关闭 |
| FDCAN3 | 空闲, 留给底盘/发射/裁判系统 | — |

`ENABLE_ENCODER_UART7=true`启用20Hz串口pitch目标位置、实际位置与反馈电流FireWater三通道输出，`ENABLE_UART_SERIAL=false`关闭回环。`kEncoderCalibrationOnly=false`、`kEnableYaw=true`、`kEnablePitch=true`允许两轴运动；`kReadYawEncoder=true`保留编码器采集。FDCAN1使用两个标准过滤器、FDCAN2使用两个；RTOS堆32KB。pitch零位/限位已写入，yaw经用户确认可连续旋转。操作见 `shoot/README.md`。

## gimbal — 云台控制 (已实现)

### 数据流

```
DT7 遥控器 ──2.4G──> DR16 ──DBUS──> UART5 中断 (rc 模块解析)
                                        │ rc::RightHorizontal/RightVertical
                                        ▼
   GimbalTask (1kHz, osPriorityHigh)
   ├─ dt 计算 (钳位到 (0, 0.05s], 调度抖动不进 PID)
   ├─ outputAllowed = !kEncoderCalibrationOnly && rc::IsOnline() && safety::ShouldOutput()
   ├─ UpdateAxis(yaw)   连续多圈位置目标──┤
   ├─ UpdateAxis(pitch) ──写 motor_settings──┤
   └─ DJI_Motor_Control(): PID 计算 + 打包 0x1FE 电流帧 ──> FDCAN1 ──> 双轴GM6020
```

### 控制模式

由`GimbalConfig.hpp`决定：yaw/pitch位置模式均为true，两轴运动启用。

- **标定模式（已关闭）**: `kEncoderCalibrationOnly=true`时两轴始终发送零电流，可手动读取编码器。位置模式请求和PID配置保留但不执行运动。
- **位置模式（pitch当前）**: 安全区内右摇杆上下按`kPitchTargetRateDps`推动有界角度目标，松杆保持目标；进入端点内侧10°保护区后临时切换速度环，只允许摇杆向水平零位退回，松杆或向外拨时Disable，不会自动回正。退回安全区后重新同步当前位置并恢复位置保持。
- **位置模式（yaw当前）**: 摇杆按`kYawTargetRateDps`推动多圈角度目标，角度环串速度环。解锁保持当前位置，不自动回零；所有yaw参数不变。

### 安全行为 (每个周期判定)

| 条件 | 行为 |
|---|---|
| 遥控失联 (`rc::IsOnline()` == false) | 两轴 Disable, 发送零输出 |
| 急停 (`safety::ShouldOutput()` == false, 双拨杆 DOWN) | 同上 |
| 编码器标定模式 | 两轴无条件Disable并发送零电流，UART7继续输出反馈 |
| 电机无反馈/反馈超时 | 两轴位置模式均需首帧定位，之后反馈超时不停机 |
| 位置模式首帧建立角度基准 (`angle_epoch` 变化) | 同步当前角度，不自动寻找水平零位 |
| pitch进入顶点内侧10°保护区或越界 | 位置模式临时使用速度环，只接受摇杆朝零位退回；向外及零目标撤输出，不保证制动距离 |
| pitch回到安全区 | 同步当前位置，恢复位置保持，不追逐旧目标 |

重新使能的语义: 所有目标状态被清空, 位置目标从**当前实测角度**重新开始, 不会追逐失联前的旧目标; PID 历史被 `DJI_Motor_Enable()` 清零。

### 位置标定状态与后续流程

pitch零位893、当前代码配置上顶点120、下顶点1490；位置目标限制在两端各内收10°的行程内。当前位置进入保护区后只允许手动向内退回，无法保证惯性制动距离。编码器计数增大对应向下。yaw零位3741，经用户确认无限位且线束允许连续旋转，已设continuous=true并启用。`static_assert`检查非连续轴标定。机构变化时须重新核对：

1. 记录机械零位 (云台正前) 时两轴的 13 位编码器原始值, 填入 `zero_raw`, 置 `calibrated = true`;
2. 实测相对零位的最小/最大可动角度 (必须落在零点 ±180° 内), 填入 `min_deg` / `max_deg`;
3. yaw 若确认可以连续旋转 (滑环), 可改用 `continuous = true` 而不做行程限位; pitch 有机械限位, **禁止**使用 continuous;
4. 方向不对时改 `kYawMotorReverse` / `kPitchMotorReverse`, 而不是改 PID 符号。

### 实车整定参数 (均在 GimbalConfig.hpp)

| 参数 | yaw | pitch | 说明 |
|---|---|---|---|
| 速度环 Kp / Ki / Kd | 以`GimbalConfig.hpp`为准 | 以`GimbalConfig.hpp`为准 | 两轴参数仍需实车整定 |
| 电流输出 / 积分限幅 | 以`GimbalConfig.hpp`为准 | 以`GimbalConfig.hpp`为准 | 电机内部电流环必须开启，协议最大电流为±3A |
| 角度环 Kp / Ki | 12.0 / 0 | 10.0 / 0 | 两轴当前均执行角度环串速度环 |
| 角度环输出速度限幅 | ±240°/s | ±90°/s | 均为角度环输出，不是摇杆目标变化率 |
| 摇杆目标 | 位置变化率180°/s | 位置变化率60°/s | 两轴松杆保持角度目标 |

## chassis — 底盘 (规划中)

规划见 [chassis/README.md](chassis/README.md)。落地时应按实车接线核对电机 ID，并在双板链路及安全模块初始化后接入控制任务。

## shoot — 发射机构

操作、硬件默认值及堵转处理见 [shoot/README.md](shoot/README.md)。已实现摩擦轮、单发、连发及有限堵转反转，裁判热量管理尚未接入。

## 约定

- 所有执行器输出必须以 `safety::ShouldOutput()` 为总开关, 不得绕过;
- "必须与实车一致"的硬件事实 (CAN ID、转向、行程、PID) 一律集中在各 `*Config.hpp`, 禁止散落在逻辑代码中;
- 应用层每个模块一个 FreeRTOS 任务, 任务内禁止长时间阻塞, 共享数据的读写遵循各模块头注释声明的线程模型。
