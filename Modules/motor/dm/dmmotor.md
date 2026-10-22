# 达妙MIT接口

当前实现只保留MIT。统一说明见 [电机模块说明](../README.md)，接口见 dmmotor.h。

## 初始化示意（非实车参数）

```cpp
MIT_Init_Config_s config{};
config.can_init_config.can_handle = /* 实际经典模式FDCAN句柄 */;
config.can_init_config.master_id = /* 电机MIT命令ID，1..15 */;
config.can_init_config.can_id = /* 电机反馈CAN ID */;
config.controller_param_init_config.Kp = /* 已整定MIT刚度 */;
config.controller_param_init_config.Kd = /* 已整定MIT阻尼 */;
// protocol_limits必须与该电机调试助手配置一致。
auto* motor = MITMotorInit(&config); // 默认停止，不发送使能。
```

设置参考：DMMotorSetMITAngle(rad)、DMMotorSetMITSpeed(rad/s)、DMMotorSetMITTorque(N·m)。
Enable后等待新的使能反馈，DMMotorTask由上层固定1ms调用，内部2次分频。
Stop是零Kp/Kd/转矩，Disable才是硬件0xFD；不会自动清错或恢复。
初始化、命令设置及周期控制由一个任务串行访问，反馈数据由CAN回调更新。

旧普通模式PID、PIDCalculate/PIDClear、MCU前馈指针、重复DM_Motor_s接口均不再支持。
MIT报文的Kp/Kd必须保留。具体行为、协议量程/单位边界和验证限制参见上层README。



