# STM32F103 C610/M2006 基础 CAN 工程

基于 STM32F103C8T6 与 HAL 库的 C610 电调、M2006 电机基础控制工程，
包含 CAN 初始化、反馈接收和电流命令发送代码。

该工程从旧 `Mystorege` 仓库的 `main` 分支中独立出来，原目录为
`2006/`。

## 构建

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

硬件接线、CAN ID 和电流范围应在上机前重新核对。
