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
| `gimbal/` | 云台 yaw/pitch 两轴控制 (GM6020 ×2) | 已实现 |
| `chassis/` | 底盘控制规划，见目录内 README | 尚未实现 |
| `shoot/` | 发射机构规划，见目录内 README | 尚未实现 |

## RobotConfig.hpp — 编译期配置

烧录前必须核对的两件事:

1. **板卡角色**: `INFANTRY_BOARD_GIMBAL` — 云台板为 `1`, 底盘板为 `0`, 两块板必须相反, 否则双板链路永远离线 (互相收不到心跳)。
2. **FDCAN 分配** (H723 共 3 路):

| 控制器 | 用途 | 配置位置 |
|---|---|---|
| FDCAN1 | 云台电机 (GM6020 yaw=0x205, pitch=0x206) | `GimbalConfig.hpp` `kGimbalCanHandle` |
| FDCAN2 | 双板通信 interboard 独占 (0x301/0x302) | `RobotConfig.hpp` `INFANTRY_INTERBOARD_FDCAN` |
| FDCAN3 | 空闲, 留给底盘/发射/裁判系统 | — |

`InfantryInitialization` 命名空间下只保留已实现模块的启停开关，每个开关控制 `InfantryInit()` 中的一个分支。遥控和云台分别使用 `ENABLE_RC`、`ENABLE_GIMBAL`；底盘、发射和 LED 待实现时再增加开关。

## gimbal — 云台控制 (已实现)

### 数据流

```
DT7 遥控器 ──2.4G──> DR16 ──DBUS──> UART5 中断 (rc 模块解析)
                                        │ rc::RightHorizontal/RightVertical
                                        ▼
   GimbalTask (1kHz, osPriorityHigh)
   ├─ dt 计算 (钳位到 (0, 0.05s], 调度抖动不进 PID)
   ├─ outputAllowed = rc::IsOnline() && safety::ShouldOutput()
   ├─ UpdateAxis(yaw)   ──写 motor_settings──┐
   ├─ UpdateAxis(pitch) ──写 motor_settings──┤
   └─ DJI_Motor_Control(): PID 计算 + 打包 0x1FF 帧 ──> FDCAN1 ──> GM6020 ×2
```

### 控制模式

由 `GimbalConfig.hpp` 的 `kYawPositionMode` / `kPitchPositionMode` 决定, 当前**均为 false (纯速度模式)**:

- **速度模式**: 右摇杆值 (归一化到 [-1,1]) 直接乘以 `kXxxMaxSpeedDps` 得到角速度目标, 电机跑"速度环+电流环"。yaw 打满 400°/s, pitch 打满 300°/s。
- **位置模式** (代码已就绪, 待标定开启): 摇杆按 `kXxxTargetRateDps` 的最大变化率推动角度目标, 角度环输出速度目标串速度环。目标被 `MotorAngleLimits` (相对编码器零点的行程) 限幅。

### 安全行为 (每个周期判定)

| 条件 | 行为 |
|---|---|
| 遥控失联 (`rc::IsOnline()` == false) | 两轴 Disable, 发送零输出 |
| 急停 (`safety::ShouldOutput()` == false, 双拨杆 DOWN) | 同上 |
| 电机反馈超时 (> `kGimbalFeedbackTimeoutMs` = 100ms 无反馈) | 仅该轴 Disable |
| 位置模式下反馈重锚 (`angle_epoch` 变化, 见 dji 驱动) | 目标自动同步当前角度, 电机 Disable 后重新使能 |
| 位置模式下实测角度越出行程 | 该轴 Disable, 目标复位 |

重新使能的语义: 所有目标状态被清空, 位置目标从**当前实测角度**重新开始, 不会追逐失联前的旧目标; PID 历史被 `DJI_Motor_Enable()` 清零。

### 位置模式开启前的标定流程 (TODO)

`kYawAngleLimits` / `kPitchAngleLimits` 当前全是未标定值, `static_assert` 保证没标定就不可能开启位置模式。开启流程:

1. 记录机械零位 (云台正前) 时两轴的 13 位编码器原始值, 填入 `zero_raw`, 置 `calibrated = true`;
2. 实测相对零位的最小/最大可动角度 (必须落在零点 ±180° 内), 填入 `min_deg` / `max_deg`;
3. yaw 若确认可以连续旋转 (滑环), 可改用 `continuous = true` 而不做行程限位; pitch 有机械限位, **禁止**使用 continuous;
4. 方向不对时改 `kYawMotorReverse` / `kPitchMotorReverse`, 而不是改 PID 符号。

### 实车整定参数 (均在 GimbalConfig.hpp)

| 参数 | yaw | pitch | 说明 |
|---|---|---|---|
| 速度环 Kp / Ki / Kd | 14 / 0.05 / 0 | 12 / 0.05 / 0 | 电压指令输出, 限幅 ±25000 |
| 角度环 Kp (预置) | 3.0 | 3.0 | 输出为角速度目标 (°/s), 限幅 ±120 / ±80 |
| 目标变化率 | 60°/s | 40°/s | 位置模式摇杆打满时角度目标的最大变化速度 |

## chassis — 底盘 (规划中)

规划见 [chassis/README.md](chassis/README.md)。落地时应按实车接线核对电机 ID，并在双板链路及安全模块初始化后接入控制任务。

## shoot — 发射机构 (规划中)

规划见 [shoot/README.md](shoot/README.md)。摩擦轮、拨弹盘和热量管理尚未实现。

## 约定

- 所有执行器输出必须以 `safety::ShouldOutput()` 为总开关, 不得绕过;
- "必须与实车一致"的硬件事实 (CAN ID、转向、行程、PID) 一律集中在各 `*Config.hpp`, 禁止散落在逻辑代码中;
- 应用层每个模块一个 FreeRTOS 任务, 任务内禁止长时间阻塞, 共享数据的读写遵循各模块头注释声明的线程模型。
