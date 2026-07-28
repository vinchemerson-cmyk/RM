# STM32F103 C610/M2006 PID 正弦位置工程

基于 STM32F103C8T6 与 HAL 库的 C610 电调、M2006 电机控制工程，在
CAN 收发基础上加入位置环、速度环串级 PID 和正弦位置目标。

该工程从旧 `Mystorege` 仓库的
`f103c8t6,c610,2006PIDSIN_position` 分支中独立出来。

## 构建

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

PID 参数、电机 ID、目标幅值与机械范围均属于旧工程配置，上机前需要
重新核对。
