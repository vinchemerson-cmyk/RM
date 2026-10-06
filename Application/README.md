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
| `gimbal/` | 云台控制及编码器快照 | 两轴 LQI，当前手填 K，可选上电求解 |
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
   └─ LQI 电流计算 ──> DJI_Motor_Control(): 打包 0x1FE ──> FDCAN1 ──> 双轴GM6020
```

### 控制模式

由`GimbalConfig.hpp`决定：yaw/pitch位置模式均为true，两轴使用 LQI。当前 `kPitchLqiGainSource`、`kYawLqiGainSource` 为 `Manual`，直接使用手填 K，不受零占位模型影响；切到 `BootSolve` 后须填写模型和 Q/R，求解失败的轴保持零电流。

- **标定模式（已关闭）**: `kEncoderCalibrationOnly=true`时两轴始终发送零电流，可手动读取编码器。位置模式请求和PID配置保留但不执行运动。
- **位置模式（pitch当前）**: 安全区内右摇杆上下按`kPitchTargetRateDps`推动有界角度目标，LQI 计算电流，松杆保持目标；进入端点内侧10°保护区后临时使用 K_speed 速度反馈，只允许摇杆向水平零位退回，松杆或向外拨时Disable。退回安全区后重新同步当前位置并恢复位置保持。
- **位置模式（yaw当前）**: 摇杆按`kYawTargetRateDps`推动多圈角度目标，LQI 计算电流；解锁保持当前位置，不自动回零。

### LQI 参数预留

`GimbalConfig.hpp` 提供 `kPitchLqiConfig`、`kYawLqiConfig`。`gimbal::Init()` 在调度器启动前按每轴 `GainSource` 装载 K：`Manual` 直接装载 `k_angle/k_speed/k_integral`，不执行求解、不使用 A/B 或 Q/R；`BootSolve` 调用一次 `LQI_Solve`，成功后将结果装载到独立的 `LQIController`，覆盖手填 K。非法手填 K/限幅和求解失败均使对应轴保持禁用，不在两种模式之间自动回退。1kHz 任务不重新求解、不执行原云台 PID。摩擦轮控制不变。

配置包含三个反馈增益 K、Q/R、采样周期、电流输出范围及积分状态范围。当前周期 1ms、电流范围 ±3A；手填 K 和 Q/R 保留用户当前取值，积分范围仍为零。手填模式调 K，求解模式调模型与 Q/R，修改配置后编译烧录并重启。

pitch 的 Manual K 按保留的原 PID 比例增益换算：`k_angle=角度Kp*速度Kp/kDjiGm6020CommandUnitsPerAmp`，`k_speed=速度Kp/kDjiGm6020CommandUnitsPerAmp`。yaw 手填增益调为与 pitch 相同，两轴均为 `k_angle=0.1`、`k_speed=0.01`，单位分别为 A/度、A/(度/秒)。pitch 只匹配原串级 PID 未限幅的比例反馈，yaw 不再匹配旧 yaw PID；不恢复原角度环速度限幅和 pitch 速度积分。LQI 积分限幅仍为零。

单位约定：角度为度，速度为度/秒，积分为度*秒，输出为 A。定义 `e = 实际角度 - 目标角度`、`z = integral(e * dt)`，定点保持的控制律为 `I = -k_angle*e - k_speed*实际速度 - k_integral*z`。离线模型必须沿用这些单位与积分符号；若求解工具使用 `目标 - 实际` 的积分状态，导入前须转换积分增益符号。填入模型和 Q/R 后可调用 `LQI_Solve` 计算 K，仍需另行接入运行代码；GM6020 发送前使用 `kDjiGm6020CommandUnitsPerAmp` 将 A 换算为指令码。

实车模型入口为 `kPitchLqiModel` / `kYawLqiModel`，当前 A/B 全零，须先辨识并离散化后填写。模型为 `x[k+1]=A*x[k]+B*I[k]`，其中 `x=[角度,速度]`。求解器增加 `z[k+1]=z[k]+Ts*e[k]`，使用 `Q=diag(q_angle,q_speed,q_integral)`、`R=r_current`、`N=0` 解离散 Riccati 方程，不做连续模型离散化。

```cpp
LQI_Solve_Result_s result{};
auto config = kPitchLqiConfig; // 运行时副本，不修改 constexpr 配置。
if (LQI_Solve(kPitchLqiModel, config, result) == LQI_Solve_Status::Success)
{
    config.k_angle = result.gain[0];
    config.k_speed = result.gain[1];
    config.k_integral = result.gain[2];
}
```

求解器返回迭代次数、相对 Riccati 残差与 P，并检查 float K 的闭环稳定性。失败时对应轴保持零电流，RTT 打印轴名、状态和迭代次数，不影响其他模块。状态编号：0成功、1非法输入、2数值异常、3未收敛、4非稳定解。配置的积分限幅目前为零，模型和权重填好后还须设置非零积分范围才能消除静差。

`LQIController` 使用更新前的 `z[k]` 计算电流，再更新 `z[k+1]`，与求解器离散时序一致；包含积分限幅和条件积分抗饱和。失联、急停、异常周期、反馈建立新基准时清空积分。pitch 保护区内只接受向内摇杆，临时使用已求出 K 的速度反馈项，清空积分和位置目标；松杆/向外时零输出，回到安全区后重新同步当前位置。UART7 的位置/电流三个通道继续保留，保护区无位置目标时 CH1 为零。

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
4. 方向不对时核对 `kYawMotorReverse` / `kPitchMotorReverse`，模型 A/B 与反馈、电流须使用方向处理后的同一逻辑坐标。

### 实车整定参数 (均在 GimbalConfig.hpp)

| 参数 | yaw | pitch | 说明 |
|---|---|---|---|
| 增益来源 | `kYawLqiGainSource` | `kPitchLqiGainSource` | 当前 Manual；可切到 BootSolve |
| 模型 A/B | `kYawLqiModel` | `kPitchLqiModel` | 当前零占位，仅 BootSolve 模式需要 |
| Q/R 与积分限幅 | `kYawLqiConfig` | `kPitchLqiConfig` | Q/R 为占位权重，积分范围目前为零 |
| K | 当前直接使用手填值 | 当前直接使用手填值 | BootSolve 则上电求解；原 PID 增益不再使用 |
| 电流输出范围 | ±3A | ±3A | GM6020 内部电流环必须开启 |
| 摇杆目标 | 位置变化率180°/s | 位置变化率60°/s | 两轴松杆保持角度目标 |

启动求解在调度器启动前使用 MSP；Debug 栈用量报告显示求解调用链超过原 1KB 预留，因此链接脚本与 CubeMX 的主栈配置同步增加至 4KB。GimbalTask 的 2KB 任务栈不承担 Riccati 求解。

## chassis — 底盘 (规划中)

规划见 [chassis/README.md](chassis/README.md)。落地时应按实车接线核对电机 ID，并在双板链路及安全模块初始化后接入控制任务。

## shoot — 发射机构

操作、硬件默认值及堵转处理见 [shoot/README.md](shoot/README.md)。已实现摩擦轮、单发、连发及有限堵转反转，裁判热量管理尚未接入。

## 约定

- 所有执行器输出必须以 `safety::ShouldOutput()` 为总开关, 不得绕过;
- "必须与实车一致"的硬件事实 (CAN ID、转向、行程、PID) 一律集中在各 `*Config.hpp`, 禁止散落在逻辑代码中;
- 应用层每个模块一个 FreeRTOS 任务, 任务内禁止长时间阻塞, 共享数据的读写遵循各模块头注释声明的线程模型。
