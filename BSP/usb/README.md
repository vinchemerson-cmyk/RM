# USB CDC

当前固件使用 CubeMX 生成的 `USB_DEVICE/` 设备栈：`Core/Src/freertos.c` 中的 defaultTask 调用 `MX_USB_DEVICE_Init()`，`Modules/communication/usb_serial_task.cpp` 使用 `CDC_Transmit_HS()` 发送姿态数据。

本目录仅保留接口规划位置；不再维护另一套 USB Device 初始化封装。
