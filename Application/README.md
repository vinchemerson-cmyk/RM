# Application 层文档

## 当前双板状态（2026-11）

- 角色装配由 `RobotConfig.hpp` 的 `BoardRole` 决定，`Robot.cpp` 使用 `if constexpr`。
- 云台板：RC、SAFETY、GIMBAL、SHOOT；底盘板：CHASSIS、IMU、UART7、USB；两板都启用 interboard。
- 云台板每 10ms 发布 `chassis/command`（10 字节）与 `chassis/safety`（4 字节）；底盘板订阅并检查 100ms 超时。
- 云台板 UART7 输出 aim pitch 轴八通道；yaw 的第二次 provider 调用不会创建任务；底盘板 UART7 输出四轮 12 通道。
- 本文以下历史章节中的 GM6020、六通道等旧描述已过时；当前架构以 [docs/dual-board-isolation.md](../docs/dual-board-isolation.md) 为准。

Application 层是"整车业务"层: 决定 **这台机器人要做什么**。它向下调用 `Modules/` 提供的驱动与算法, 通过 `BSP/` 访问硬件,
自身不直接操作寄存器。与 CubeMX 生成代码 (`Core/`) 的唯一边界是 `Robot.h` 暴露的 C 接口 `InfantryInit()`。

```
Core/Src/main.c        外设初始化 (CubeMX 生成)
  └─ freertos.c        MX_FREERTOS_Init(): 创建 defaultTask, 调用 InfantryInit()
       └─ Robot.cpp    InfantryInit(): 按 RobotConfig.hpp 开关装配各模块
            └─ osKernelStart() 后, 各模块任务并行运行
```

## 目录

| 文件/目录               | 职责                                        | 状态                               |
|-------------------------|---------------------------------------------|------------------------------------|
| `Robot.h` / `Robot.cpp` | 整车初始化入口, 装配各模块任务              | 已实现                             |
| `RobotConfig.hpp`       | 板卡角色、FDCAN 分配、模块启停开关 (编译期) | 已实现                             |
| `gimbal/`               | 云台控制及编码器快照                        | 两轴 LQI，当前手填 K，可选上电求解 |
| `chassis/`              | 四轮 M3508 控制与板间命令/安全状态消费      | 已实现，硬件参数待实测             |
| `shoot/`                | 摩擦轮 + 拨弹盘                             | 拨弹 0x201、右 0x202、左 0x203     |

## RobotConfig.hpp — 编译期配置

烧录前必须核对的三件事:

1. **板卡角色**: `INFANTRY_BOARD_GIMBAL` — 云台板为 `1`, 底盘板为 `0`, 两块板必须相反。
2. **安全来源**: `ENABLE_SAFETY` 只在云台板为 true；底盘板消费转发到 `chassis/safety` 的状态，不再运行本地安全任务。
3. **FDCAN 分配**:

| 板卡 | 控制器 | 用途 | 配置位置 |
|---|---|---|---|
| 云台板 | FDCAN1 | FD，板间链路 | `board_can_config.h` / `interboard.cpp` |
| 云台板 | FDCAN2 | Classic，双摩擦轮/拨弹盘 | `ShootConfig.hpp` |
| 云台板 | FDCAN3 | FD，yaw/aim pitch DM | `GimbalConfig.hpp` |
| 底盘板 | FDCAN1 | Classic，四轮 M3508 | `ChassisConfig.h` |
| 底盘板 | FDCAN2 | FD，板间链路 | `board_can_config.h` / `interboard.cpp` |
| 底盘板 | FDCAN3 | Classic，空闲 | — |

`ENABLE_INTERBOARD=true` 两板都启动板间任务。云台板注册 TX 主题 `chassis/command`（10B）与 `chassis/safety`（4B）；底盘板注册对应 RX 主题并在 100ms 超时后停机。
`ENABLE_ENCODER_UART7=true`：云台板输出 aim pitch 轴八通道（yaw 的第二次 provider 调用不生效），底盘板输出四轮十二通道。RC 掉线判定为 1000ms。

## gimbal — 云台控制 (已实现)

### 数据流

```
DT7 遥控器 ──2.4G──> DR16 ──DBUS──> UART5 中断 (rc 模块解析)
                                        │ rc::RightHorizontal/RightVertical
                                        ▼
   GimbalTask (1kHz, osPriorityHigh)
   ├─ dt 计算 (钳位到 (0, 0.05s], 调度抖动不进 PID)
   ├─ outputAllowed = !kEncoderCalibrationOnly && rc::IsOnline() && safety::ShouldOutput()
   ├─ UpdateAxis(yaw)   连续位置目标─────┤
   ├─ UpdateAxis(pitch) ──写 motor_settings──┤
   └─ LQI 电流计算 ──> DJI_Motor_Control(): 打包 0x1FE ──> FDCAN1 ──> 双轴GM6020
```

### 可重复 yaw 遥控指令测试（默认关闭）

`GimbalConfig.hpp` 的 `kYawRemoteCommandTestEnabled=true` 启用测试；
`kYawRemoteCommandTestConfig` 配置半杆幅值和各段毫秒时长。
`RemoteCommandTest.hpp` 的 `GenerateRemoteYawCommand(elapsed_ms, config)`
模拟 DBUS 解析之后的归一化摇杆值，不向串口注入帧、不伪造遥控在线状态。

默认序列共6秒：保持1秒、yaw正向0.5半杆0.5秒、保持2秒、反向半杆0.5秒、保持2秒。
按120°/s目标变化率，每段移动30°目标角度，反向段结束后目标回到本轮起始目标。
先将左拨杆置 MIDDLE，右拨杆离开 UP，再拨到 UP 并保持，触发一次。
首次使能/重连时右拨杆已在 UP 不会自动启动，必须释放后重新触发。
结束后保持位置，不自动循环；再次触发须先让右拨杆离开 UP。
右拨杆离开 UP 或左拨杆离开 MIDDLE 会取消后续序列，但保持取消时的角度目标；
遥控掉线、双 DOWN 急停或控制周期异常仍走原有禁用逻辑，并清除测试触发状态。
测试模式下 pitch 摇杆输入固定为零，原限位逻辑仍有效；摩擦轮仍受真实遥控器控制，
测试时保持左拨杆 MIDDLE，避免启轮。测试不得与固定电流模式同时开启。
串口八通道不变，目标速度列会显示模拟给定的 ±60°/s。
每轮尽量从同一实际角度、静止状态开始，并保持相同负载、电源和测试配置，
仅改变一个控制参数，再比较超调、稳定时间与峰值电流。该序列不是机构仿真。

### 控制模式

由`GimbalConfig.hpp`决定：两轴均使用 LQI 位置控制（`kYawPositionMode=true`），当前均为 Manual。`kYawLqiConfig` 和
`kPitchLqiConfig`
各自完整声明，不再复制配置；yaw位置增益0.28、速度反馈增益0.0035，pitch仍为0.1、0.01。目标变化率仍为120°/s，松杆保持角度；yaw连续旋转。BootSolve仍需分别辨识各轴模型。

- **标定模式（已关闭）**: `kEncoderCalibrationOnly=true`时两轴始终发送零电流，可手动读取编码器。位置模式请求和 LQI
  配置保留但不执行运动。
- **位置模式（pitch当前）**: 安全区内右摇杆上下按`kPitchTargetRateDps`推动有界角度目标，LQI 计算电流，松杆保持目标；进入端点内侧10°保护区后临时使用
  K_speed 速度反馈，只允许摇杆向水平零位退回，松杆或向外拨时Disable。退回安全区后重新同步当前位置并恢复位置保持。
- **备用速度测试（默认关闭）**: `kYawPositionMode=false` 时才使用速度目标及 `kYawSpeedTestMaxDps`。实际输出限幅以
  `kYawLqiConfig` 为准，切换测试前须单独检查参数。

### LQI 参数预留

当前 yaw 目标速度前馈：`kYawTargetSpeedFeedforwardAperDps=0.002` A/ (°/s)，本轮仅从0.0015提高此前馈。
仅正常位置模式使用。总电流为 `clip(位置项 + 速度反馈项 + 0.002*目标速度, -3, 3)`；
当前积分范围仍为零，未启用积分。固定测试 ±60°/s 增加 ±0.12A，打满 ±120°/s 增加 ±0.24A。
松杆前馈立即归零，失联/急停仍禁用输出；pitch、端点恢复、备用速度模式和固定电流测试不加此前馈。
前馈与反馈统一限幅，并共同参与条件积分抗饱和判断，非法前馈返回零输出。
UART八通道不变：位置项和速度项仍仅为各自反馈贡献，未饱和时总指令与两项之差
为前馈（当前无积分）；不要将该差值误判为计算错误。此配置尚未上板验证，可能增加速度峰值与松杆超调。

`GimbalConfig.hpp` 提供 `kPitchLqiConfig`、`kYawLqiConfig`。`gimbal::Init()` 在调度器启动前按每轴 `GainSource` 装载 K：
`Manual` 直接装载 `k_angle/k_speed/k_integral`，不执行求解、不使用 A/B 或 Q/R；`BootSolve` 调用一次 `LQI_Solve`
，成功后将结果装载到独立的 `LQIController`，覆盖手填 K。非法手填 K/限幅和求解失败均使对应轴保持禁用，不在两种模式之间自动回退。1kHz
任务不重新求解、不执行原云台 PID。摩擦轮控制不变。

配置包含三个反馈增益 K、Q/R、采样周期、电流输出范围及积分状态范围。当前周期 1ms，两轴电流范围均为 ±3A，积分范围仍为零。手填模式调
K，求解模式调模型与 Q/R，修改配置后编译烧录并重启。

当前 yaw 的`k_angle=0.28` A/度、`k_speed=0.0035` A/ (度/秒)；pitch仍为0.1、0.01。两轴`k_integral=0.01`，积分范围为零，输出限幅
±3A。本次仅拆分两轴配置，保留用户已设置的yaw位置增益0.28，不调整任何实际参数。K、Q/R、输出限幅和积分范围可分别填写；采样周期仍应与共同任务周期一致。位置项保留，不能等同于纯速度测试。当前参数尚未完成板上验证，不代表高频振荡不会复现。

单位约定：角度为度，速度为度/秒，积分为度*秒，输出为 A。定义 `e = 实际角度 - 目标角度`、`z = integral(e * dt)`，定点保持的控制律为
`I = -k_angle*e - k_speed*实际速度 - k_integral*z`。离线模型必须沿用这些单位与积分符号；若求解工具使用 `目标 - 实际`
的积分状态，导入前须转换积分增益符号。填入模型和 Q/R 后可调用 `LQI_Solve` 计算 K，仍需另行接入运行代码；GM6020 发送前使用
`kDjiGm6020CommandUnitsPerAmp` 将 A 换算为指令码。

实车模型入口为 `kPitchLqiModel` / `kYawLqiModel`，当前 A/B 全零，须先辨识并离散化后填写。模型为 `x[k+1]=A*x[k]+B*I[k]`，其中
`x=[角度,速度]`。求解器增加 `z[k+1]=z[k]+Ts*e[k]`，使用 `Q=diag(q_angle,q_speed,q_integral)`、`R=r_current`、`N=0` 解离散
Riccati 方程，不做连续模型离散化。

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

求解器返回迭代次数、相对 Riccati 残差与 P，并检查 float K 的闭环稳定性。失败时对应轴保持零电流，RTT
打印轴名、状态和迭代次数，不影响其他模块。状态编号：0成功、1非法输入、2数值异常、3未收敛、4非稳定解。配置的积分限幅目前为零，模型和权重填好后还须设置非零积分范围才能消除静差。

`LQIController` 使用更新前的 `z[k]` 计算电流，再更新 `z[k+1]`，与求解器离散时序一致；包含积分限幅和条件积分抗饱和。失联、急停、异常周期、反馈建立新基准时清空积分。pitch
保护区内只接受向内摇杆，临时使用已求出 K 的速度反馈项，清空积分和位置目标；松杆/向外时零输出，回到安全区后重新同步当前位置。UART7
六通道为 yaw 目标速度、实际连续角度、实际速度、指令电流、反馈年龄、反馈转矩电流；禁用时目标速度和指令电流显示零。

### 安全行为 (每个周期判定)

| 条件                                                  | 行为                                                                             |
|-------------------------------------------------------|----------------------------------------------------------------------------------|
| 遥控失联 (`rc::IsOnline()` == false)                  | 两轴 Disable, 发送零输出                                                         |
| 急停 (`safety::ShouldOutput()` == false, 双拨杆 DOWN) | 同上                                                                             |
| 编码器标定模式                                        | 两轴无条件Disable并发送零电流，UART7继续输出反馈                                 |
| 电机无反馈/反馈超时                                   | 两轴位置模式均需首帧定位，之后反馈超时不停机                                     |
| 位置模式首帧建立角度基准 (`angle_epoch` 变化)         | 同步当前角度，不自动寻找水平零位                                                 |
| pitch进入顶点内侧10°保护区或越界                      | 位置模式临时使用速度环，只接受摇杆朝零位退回；向外及零目标撤输出，不保证制动距离 |
| pitch回到安全区                                       | 同步当前位置，恢复位置保持，不追逐旧目标                                         |

重新使能的语义: 所有目标状态被清空, 位置目标从 **当前实测角度**重新开始, 不会追逐失联前的旧目标; PID 历史被
`DJI_Motor_Enable()` 清零。

### 位置标定状态与后续流程

pitch零位893、当前代码配置上顶点120、下顶点1490；位置目标限制在两端各内收10°的行程内。当前位置进入保护区后只允许手动向内退回，无法保证惯性制动距离。编码器计数增大对应向下。yaw零位3741，经用户确认无限位且线束允许连续旋转，已设continuous=true并启用。
`static_assert`检查非连续轴标定。机构变化时须重新核对：

1. 记录机械零位 (云台正前) 时两轴的 13 位编码器原始值, 填入 `zero_raw`, 置 `calibrated = true`;
2. 实测相对零位的最小/最大可动角度 (必须落在零点 ±180° 内), 填入 `min_deg` / `max_deg`;
3. yaw 若确认可以连续旋转 (滑环), 可改用 `continuous = true` 而不做行程限位; pitch 有机械限位, **禁止**使用 continuous;
4. 方向不对时核对 `kYawMotorReverse` / `kPitchMotorReverse`，模型 A/B 与反馈、电流须使用方向处理后的同一逻辑坐标。

### 实车整定参数 (均在 GimbalConfig.hpp)

| 参数           | yaw                 | pitch                 | 说明                                      |
|----------------|---------------------|-----------------------|-------------------------------------------|
| 增益来源       | `kYawLqiGainSource` | `kPitchLqiGainSource` | 当前 Manual；可切到 BootSolve             |
| 模型 A/B       | `kYawLqiModel`      | `kPitchLqiModel`      | 当前零占位，仅 BootSolve 模式需要         |
| Q/R 与积分限幅 | `kYawLqiConfig`     | `kPitchLqiConfig`     | Q/R 为占位权重，积分范围目前为零          |
| K              | 当前直接使用手填值  | 当前直接使用手填值    | BootSolve 则上电求解；原 PID 增益不再使用 |
| 电流输出范围   | ±3A                 | ±3A                   | GM6020 内部电流环必须开启                 |
| 摇杆目标       | 位置变化率120°/s    | 位置变化率120°/s      | 两轴松杆保持角度目标                      |

启动求解在调度器启动前使用 MSP；Debug 栈用量报告显示求解调用链超过原 1KB 预留，因此链接脚本与 CubeMX 的主栈配置同步增加至
4KB。GimbalTask 的 2KB 任务栈不承担 Riccati 求解。

## chassis — 底盘 (已实现)

见 [chassis/README.md](chassis/README.md)。底盘板使用 FDCAN1 控制四轮 M3508；通过 `chassis/command` 接收速度/模式，通过 `chassis/safety` 接收云台板安全状态。命令或安全状态超过 100ms 未刷新即停机；`FollowGimbal` 尚未实现。硬件方向、轮速与 PI 仍需实车核对。

## shoot — 发射机构

操作、硬件默认值及堵转处理见 [shoot/README.md](shoot/README.md)。已实现摩擦轮、单发、连发及有限堵转反转，裁判热量管理尚未接入。

注意：拨弹盘 `0x201`、右摩擦轮 `0x202`、左摩擦轮 `0x203`，ID 已不重复，`kEnableFeeder=true` 可以注册。首次上板仍需核对拨弹方向、槽数和堵转保护。

## 约定

- 云台板本地执行器仍以 `safety::ShouldOutput()` 为总开关；底盘板以转发的 `chassis::SafetyState` + 100ms 超时为总闸门，不得绕过;
- "必须与实车一致"的硬件事实 (CAN ID、转向、行程、PID) 一律集中在各 `*Config.hpp`, 禁止散落在逻辑代码中;
- 应用层每个模块一个 FreeRTOS 任务, 任务内禁止长时间阻塞, 共享数据的读写遵循各模块头注释声明的线程模型。
