# 双板通信

`Application/RobotConfig.hpp` 中的 `INFANTRY_BOARD_GIMBAL` 是烧录角色：云台板为 `1`，底盘板为 `0`。`INFANTRY_INTERBOARD_FDCAN` 选择专用于板间通信的控制器，当前默认为 FDCAN2（PB5/PB6）。两板须使用同一控制器对应的外部 CAN 收发器、相同的波特率和总线接线；具体板上收发器接线仍须按实物核对。此控制器由双板模块独占，不能同时分配给电机或裁判系统。

板间使用经典 CAN 标准帧，云台到底盘 ID 为 `0x301`，底盘到云台 ID 为 `0x302`，周期 100 ms 心跳。收到对方帧后，本板把 `interboard/link` 话题的 `LinkStatus` 更新为在线；300 ms 没收到对方帧则更新为离线。应用读取这个状态后，应自行处理电机等执行器的失联保护。

数据帧第 0 字节为话题 ID（`1..255`），第 1 字节高四位为消息序号、低四位为分片编号，后六字节为数据。一个话题固定长度最多 48 字节；接收端仅发布完整、顺序正确、在 50 ms 内收齐的消息。CAN 的 CRC 保护单帧；当前协议不提供应用层确认或重发。消息中心仍使用最新值邮箱，生产速度超过链路发送速度时会覆盖旧值。

业务模块在 `InfantryInit()` 调用 `interboard::Start()` 之后、调度器启动之前注册话题。两板必须为同一 ID 注册相同长度，一个为 Tx，另一个为 Rx。示例：

```cpp
// 云台板，发送一个 4 字节底盘命令
static char commandTopic[] = "chassis/command";
interboard::RegisterTxTopic(1, commandTopic, 4);

// 底盘板，接收相同 ID 和长度
static char commandTopic[] = "chassis/command";
interboard::RegisterRxTopic(1, commandTopic, 4);
```

发送端使用 `PubRegister(commandTopic, 4)` 和 `PubPushMessage()` 发布；接收端使用 `SubRegister(commandTopic, 4)` 和 `SubGetMessage()` 读取。字段字节序、结构体布局和业务超时须由两板共用的消息定义明确规定，不能直接依赖未约定的 `struct` 填充字节。
