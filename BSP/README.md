# BSP 层文档

BSP (Board Support Package) 层把 STM32H723 的外设封装成与业务无关的 C++ 接口: 注册、收发、回调分发。向上只依赖 `Core/` 里
CubeMX 生成的 HAL 句柄 (`hfdcanX`/`huartX`/`htimX`/`hspi2`), 向下被 `Modules/` 与 `Application/` 使用。 **BSP
不含任何机器人逻辑。**

## 组件一览

| 目录      | 组件                                    | 硬件             | 说明                                         |
|-----------|-----------------------------------------|------------------|----------------------------------------------|
| `can/`    | `CAN_<classicCan>` / `CAN_<Fdcan>` 模板 | FDCAN1/2/3       | 电机与常规 CAN 设备收发                      |
| `usart/`  | `usart` 类                              | UART1/5/7/8/9/10 | DMA+Idle 不定长接收, 三种发送模式            |
| `usb/`    | 规划说明                                | USB HS (CDC)     | 实际设备栈在 CubeMX 生成的 `USB_DEVICE/`     |
| `pwm/`    | `pwm` 类                                | TIM 通道         | PWM 注册与占空比控制                         |
| `BUZZER/` | Buzzer 接口                             | TIM12 CH2        | 蜂鸣器 (基于 pwm 思路的独立实现)             |
| `log/`    | `BSPLogInit/LOGI/...`                   | SEGGER RTT       | 调试日志 (详见 [bsp_log.md](log/bsp_log.md)) |

## can/ — FDCAN 封装

### 两种接收路径

1. **常规注册** (`CAN_<T>::Register()`): 每个实例按 `rx_id` 占用一个标准帧硬件过滤器, `FilterIndex` 与静态注册表下标
   **一一对应**, 中断据此直接把帧投递到实例并回调上层 — 无遍历、无映射表。
2. **独占控制器** (`CAN_SetDedicatedRxHandler()`): 双板 interboard 这类独占整路 FDCAN 的模块, 直接接管 FIFO 接收事件,
   自行取空硬件 FIFO, 不占用过滤器表。HAL 回调 (can.cpp) 优先派发给独占处理器。

### 关键设计

- **过滤器上限**: 以 CubeMX 配置的 `StdFiltersNbr` (而非模板常量) 为准, 防止 Release 构建下 HAL 只做 assert 导致过滤器写到
  Message RAM 之外。
- **启动管理**: 首个实例注册时 `HAL_FDCAN_Start()`, 后续实例只追加过滤器 (对已 BUSY 的控制器重复 Start 会返回错误)。
- **未命中即丢弃**: `FDCAN_REJECT` 全局过滤, 总线上无关帧不进 FIFO、不产生中断。
- **DLC 处理**: 经典 CAN 0-8 字节直通; FDCAN 的 9-64 字节按 HAL 非线性编码转换, 收发两侧都有越界防护。
- **发送**: 每实例持有独立 `tx_header`, 同控制器多实例可并行填帧; `attribute.tx_id` 在发送前可被上层改写 (dji
  驱动按指令帧覆盖)。发送 FIFO 满返回 `HAL_ERROR` 不阻塞。
- **traits**: `classicCan` (8 字节) / `Fdcan` (64 字节) 决定缓冲区大小与帧格式, 显式实例化在 can.cpp。
- 中断回调 (`HAL_FDCAN_RxFifo0/1Callback`) 按 `FrameFormat` 自动分流 classic/FD; 回调顺序约定为 **先更新 Output,
  再执行上层回调**。

## usart/ — UART DMA 封装

- **接收模型**: `HAL_UARTEx_ReceiveToIdle_DMA` + 关闭 DMA 半传输中断 (HAL 的设计缺陷会导致半传输/全传输/IDLE
  三种事件都触发同一回调, 这里只要后两种)。回调链: HAL → `usart::isrHandler` → 按句柄查注册表 → 更新 `rx_size` →
  上层回调 → 自动重启接收。
- **发送模式** (`TX_MODE`): `NO_TRANSFER` (只收, 遥控器用)、`TRANSFER_BLOCKING` (阻塞)、`TRANSFER_IT`、`TRANSFER_DMA` (异步,
  数据复制到实例长期缓冲)。
- **恢复路径**: `restartRx()` 供上层在协议失步时主动重启接收; `errorHandler()`
  在校验错/溢出/帧错后自动清标志并重启接收。三处都遵循同一套"清标志 → 清 RDR → 重启 DMA → 关 HT 中断"流程。
- **实例约束**: 同一上层模块指针只能注册一个实例; 缓冲区 256 字节; 注册表位于 `.axi_bss` (见内存布局)。

## USB CDC 设备

`Core/Src/freertos.c` 中的 defaultTask 调用 CubeMX 的 `MX_USB_DEVICE_Init()`。CDC 发送任务使用 `CDC_Transmit_HS()`
，并等待上一包发送完成；接收回调位于 `USB_DEVICE/App/usbd_cdc_if.c`。原 `BSP/usb` 设备注册封装没有进入构建，也没有调用方，已移除，目录中只保留说明。

## pwm/ 与 BUZZER/ — PWM 输出

- `pwm` 类: `Register` (校验通道/去重, 比较值清零保安全电平) → `Start/Stop` → `setCompare` (0..ARR+1) / `setDutyCycle`
  (0.0..1.0, 四舍五入)。重复 Start/Stop 幂等。
- BUZZER 是一个更简单的特化: TIM12 CH2, 以"占空比百分数"为接口 (`BuzzerOn/Off/SetDuty/IsOn`), 初始化即启动 PWM、比较值 0
  表示静音。

## 时间基准

当前任务周期、掉线判断和控制周期统一使用 `osKernelGetTickCount()`，单位由 `osKernelGetTickFreq()` 换算。未使用的 DWT 和
TIM6 计时封装已移除；需要微秒计时时，再按具体调用场景增加一个统一接口。

## log/ — SEGGER RTT 日志 (详见 [bsp_log.md](log/bsp_log.md))

`BSPLogInit()` 初始化; `LOGINFO/LOGWARNING/LOGERROR` 带颜色分级别; 通过调试器在上位机 (J-Link RTT Viewer 或支持 RTT 的
DAPLink 工具) 读取, 不占用 UART。 **不支持浮点格式化**, 需先用 `Float2Str()` 转字符串。Release 可用 `DISABLE_LOG_SYSTEM`
宏整体关闭。

## 内存布局约束 (重要)

STM32H7 的 DTCM (默认栈/堆所在) **对 DMA1/DMA2 不可见**。所有被 DMA 或外设直接访问的缓冲区必须放入 AXI SRAM,
本工程的约定:

- 走 `__attribute__((section(".axi_bss")))` 显式放置: UART 实例表 (`usart.cpp`)、Bmi088 SPI DMA 缓冲 (`Bmi088.cpp`);
- DMA 描述符类对象 (如 `SpiDmaBuffers`) 需 32 字节对齐以配合 cache line;
- 链接脚本为 `STM32H723xG_flash.ld` (CubeMX 生成), 新增 DMA 缓冲区时检查它是否落在 AXI SRAM 区段。

## 通用约定

1. 注册型接口 (`CAN_/usart/pwm::Register`) 只允许在启动阶段调用; 注册表为静态存储, 中断随时可能按已注册内容分发。
2. BSP 内的 `Error_Handler()` 表示 **程序性错误** (重复注册、参数非法、表满), 属于开发期错误, 直接停机暴露问题; 运行期可恢复的异常
   (总线超时、掉线) 由上层处理, 不停机。
3. 回调内 (BSP 调进上层的函数) 禁止阻塞; 需要异步处理时入队转任务 (见各通信模块)。
