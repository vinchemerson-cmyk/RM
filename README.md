# RM

RoboMaster 电控工程集合。仓库中的每个一级文件夹代表一个可独立构建、
调试和烧录的工程；功能开发使用分支，工程之间不再通过分支区分。

## 工程列表

| 工程目录 | 说明 | 当前状态 |
| --- | --- | --- |
| [`f103-c610-m2006-can-basic`](f103-c610-m2006-can-basic/) | STM32F103C8T6 + C610/M2006 | 基础 CAN 收发 |
| [`f103-c610-m2006-pid-sine-position`](f103-c610-m2006-pid-sine-position/) | STM32F103C8T6 + C610/M2006 | 串级 PID 与正弦位置控制 |
| [`f407-gm6020-gimbal`](f407-gm6020-gimbal/) | STM32F407 + GM6020 云台控制 | Yaw 单轴 DBUS 遥控测试 |
| [`Mystorege`](f407-gm6020-gimbal/) | STM32F407 + GM6020 双轴云台控制 | Yaw 单轴 DBUS 遥控测试 |

每个工程的硬件配置、通信协议、构建方法和使用注意事项，见对应目录中的
`README.md`。
