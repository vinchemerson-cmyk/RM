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

## 当前状态 (2026-11)

| 子系统 | 状态 | 说明 |
|---|---|---|
| 云台 (DM4310, FDCAN3) | 🔶 配置完成 | aim pitch/yaw LQI 位置控制；folding pitch 已启用（0x207），限位待实测 |
| 底盘 (M3508×4, FDCAN1) | 🔶 已实现 | 板间命令驱动；电机 ID/减速比/限幅已填，方向与 PI 待上板核对 |
| 板间链路 | ✅ 已启用 | 云台板 CAN1 / 底盘板 CAN2；命令 topic 1（10B）与安全 topic 2（4B），100ms 超时 |
| UART7 | ✅ 已启用 | 云台板 aim pitch 轴八通道；底盘板四轮 12 通道；10ms 周期 |
| 遥控 | ✅ 已启用 | DBUS；掉线判定 1000ms |
| 安全 | 🔶 跨板 | 云台板产生 SafetyState；底盘板不运行本地安全任务，100ms 超时停机 |
| 发射 | 🔶 摩擦轮 | 拨弹 0x201、右轮 0x202、左轮 0x203；ID 已不重复，方向/PID 待实测 |
| DJI 驱动 | ✅ 已实现 | M3508/M2006；GM6020 已移除 |
| PID / LQI | ✅ 已实现 | 条件积分抗饱和；云台 LQI 手填增益 |
| IMU / USB / RTT | ✅ 已实现 | BMI088、USB CDC、RTT；IMU 加热尚未接入整车 |
| CMSIS-DSP / Kalman | ✅ 编译接入 | C++20；Kalman 头文件已加入，尚未用于控制 |

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
当前双板状态：云台板 FDCAN3 控制 DM yaw/aim pitch/folding pitch，FDCAN1 为板间链路，FDCAN2 接双摩擦轮；云台板每 10ms 发布 `chassis/command` 与 `chassis/safety`。底盘板 FDCAN1 控制四轮 M3508，FDCAN2 接收板间命令/安全状态，100ms 超时即停机。云台板 UART7 当前输出 aim pitch 单轴八通道；底盘板 UART7 输出四轮十二通道。

已知风险：折叠 pitch 已启用（反馈 `0x207`），但零点、方向与 0..29° 限位尚未实车核对；UART7 的第二个 yaw provider 调用不会创建任务。上板前先确认折叠限位与 UART7 需求。操作见 [测试说明](Application/shoot/README.md)。

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
