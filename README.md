# infantry_H723 — RoboMaster 步兵主控固件

基于 **STM32H723** 的 RoboMaster 步兵机器人固件, 采用 **云台板 + 底盘板双板架构**: 两板共用本工程, 由编译期开关区分角色,
经一路专用 FDCAN 互连。

- 芯片: STM32H723VGT6（以 `infantry_H723.ioc` 为准，Cortex-M7，硬件浮点）
- 框架: STM32CubeMX 生成底层 + CMake/Ninja 构建 + FreeRTOS (CMSIS-OS2)
- 语言: C/C++ 混合, 全中文 Doxygen 注释
- 生成工程: `infantry_H723.ioc` (修改外设配置后用 CubeMX 重新生成 `Core/`, 不要手改生成文件, 用户代码一律写在
  `USER CODE` 区或 Application/Modules/BSP 三层)

## 分层架构

```
┌─────────────────────────────────────────────────────┐
│ Application/  整车业务: Robot.cpp 装配 + 云台/底盘/发射 │  ← 本层决定"做什么"
├─────────────────────────────────────────────────────┤
│ Modules/      能力层: 电机驱动、PID、遥控、急停、双板通信 │  ← 本层提供"怎么做"
│               IMU、消息中心、蜂鸣器/LED 任务            │
├─────────────────────────────────────────────────────┤
│ BSP/          板级支持: FDCAN/UART/PWM/RTT 封装        │  ← 本层封装"用什么"
├─────────────────────────────────────────────────────┤
│ Core/ Drivers/ Middlewares/  CubeMX 生成 + HAL + RTOS │  ← 生成代码, 不手改
└─────────────────────────────────────────────────────┘
```

分层文档: [Application/README.md](Application/README.md) · [Modules/README.md](Modules/README.md) · [BSP/README.md](BSP/README.md)

## 当前状态 (2026-09)

| 子系统                                                    | 状态            | 说明                                                                                                |
|-----------------------------------------------------------|-----------------|-----------------------------------------------------------------------------------------------------|
| 云台 (pitch/yaw GM6020, FDCAN1)                           | ✅ 两轴位置模式 | pitch位置目标受软件行程限制，yaw连续位置控制；参数以`GimbalConfig.hpp`为准，未完成实车整定          |
| UART7 pitch 调试输出                                      | ✅ 已启用       | PE8 TX，921600/8N1，每50ms输出FireWater三通道：目标位置、实际位置（相对水平零位，°）、反馈电流（A） |
| 遥控 (DT7/DR16, UART5 DBUS)                               | ✅ 已启用       | 18 字节 DBUS、跨回调残帧重组、100ms 掉线判定                                                        |
| 急停 (双拨杆 DOWN + 失联保护)                             | ✅ 已启用       | 上电/失联后须摇杆居中、双 DOWN 解锁；所有输出经过门控                                               |
| 双板 CAN 链路 (0x301/0x302)                               | 测试关闭        | FDCAN2 当前接双摩擦轮，禁止链路同时占用                                                             |
| DJI 电机驱动 (GM6020/M3508/M2006)                         | ✅ 已实现       | 按物理总线/指令帧打包；测试阶段电机反馈超时不停机                                                   |
| PID / 角度目标 (可宿主机单测)                             | ✅ 已实现       | 测量微分 + 条件积分抗饱和                                                                           |
| BMI088 六轴姿态 CDC 输出、UART 回环自测、RTT 日志、蜂鸣器 | ✅ 已实现       | 源码与编译验证，姿态轴向待实测                                                                      |
| IMU 加热控制                                              | 🔶 部分         | 温控代码已有，尚未接入整车初始化                                                                    |
| 底盘 (chassis)                                            | ⬜ 占位         | 模块边界已约定, 见 Application/README.md                                                            |
| 摩擦轮 (M3508 ×2, FDCAN2)                                 | ✅ 测试启用     | 双轮共同启停/爬升到 1000RPM；拨弹不注册，实车 ID、方向、PID 待核对                                  |

## 构建与烧录

### 依赖

- `arm-none-eabi-gcc` (工具链前缀, 需在 PATH 中; 工具链文件 `cmake/gcc-arm-none-eabi.cmake`)
- CMake ≥ 3.22, Ninja
- (烧录调试) OpenOCD + CMSIS-DAP 调试器, 配置见 `stm32h7_daplink.cfg` (SWD, 1MHz 时钟)

### 命令

```bash
# 配置 + 构建 (CMakePresets.json 提供云台板/底盘板四个角色预设)
cmake --preset Debug-Gimbal
cmake --build build/Debug-Gimbal

# 底盘板
cmake --preset Debug-Chassis
cmake --build build/Debug-Chassis

# 产物: build/Debug-Gimbal/infantry_H723.elf
#       build/Debug-Chassis/infantry_H723.elf
```

### 烧录与调试

```bash
# OpenOCD 烧录 (DAPLink/CMSIS-DAP)
# 云台板
openocd -f stm32h7_daplink.cfg -c "program build/Debug-Gimbal/infantry_H723.elf verify reset exit"

# 底盘板：把路径替换为 build/Debug-Chassis/infantry_H723.elf
```

- 调试日志走 SEGGER RTT (`BSP/log/`), 在 J-Link RTT Viewer 或支持 RTT 的上位机中读取, 不占用串口。
- 首次上电前核对 `Application/RobotConfig.hpp`: 板卡角色 (`INFANTRY_BOARD_GIMBAL`) 与 FDCAN 分配; 云台参数核对清单见
  `Application/gimbal/GimbalConfig.hpp` 的注释。
-
当前四电机测试：FDCAN1的pitch/yaw均为位置模式，pitch使用配置的软件行程，yaw连续旋转；FDCAN2双摩擦轮不变。云台板UART7输出pitch目标/实际位置及反馈电流，保留遥控失联与双DOWN急停；底盘板UART7输出四轮十二通道，底盘未初始化时为0。操作见 [测试说明](Application/shoot/README.md)。

### 宿主机单元测试

纯逻辑和实际 DJI/CAN 驱动（用 HAL 替身）可在宿主机测试，必须使用主机 g++：

```bash
cmake -S tests -B build/host-tests -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host-tests --parallel
ctest --test-dir build/host-tests --output-on-failure
```

## 关键机制速查

| 机制            | 位置                                   | 一句话                                                                   |
|-----------------|----------------------------------------|--------------------------------------------------------------------------|
| 输出总开关      | `Modules/safety/`                      | 上电需解锁，双 DOWN 急停，遥控失联撤输出且撤销解锁状态                   |
| 电机连续角度    | `Modules/motor/dji/dji.cpp`            | 仅首帧建立角度基准，测试阶段移除超时重锚，反馈中断后的累计角度不保证准确 |
| 位置目标限幅    | `Modules/motor/angle_target.h`         | 未实测标定的行程无法启用 (`static_assert` 兜底)                          |
| 双板话题        | `Modules/communication/interboard.cpp` | 话题 ID + 序号/分片, 最新值邮箱语义, 无重发                              |
| DMA 内存约束    | `BSP/README.md` 内存布局节             | DMA 缓冲必须放 `.axi_bss` (DTCM 对 DMA 不可见)                           |
| UART 不定长接收 | `BSP/usart/usart.hpp`                  | DMA+Idle, 半传输中断已关 (HAL 设计缺陷规避)                              |

## TODO / 已知问题

- [ ] 云台新PID与位置模式实车验证；pitch标定已写入、yaw连续旋转已获用户确认 (流程见 Application/README.md)
- [ ] 底盘模块落地 (M3508 ×4 + interboard 指令 + 掉线保护)
- [x] 遥控摩擦轮 + 拨弹盘（单发/连发/有限堵转反转）软件实现与回归测试
- [ ] 发射机构实车方向、PID、拨盘槽数和射速标定；裁判热量门控
- [ ] IMU 加热闭环接入整车初始化
- [ ] LED 状态指示任务; 蜂鸣器接入解锁/报警逻辑
- [ ] 裁判系统接入 (UART), 支撑功率限制与热量管理
