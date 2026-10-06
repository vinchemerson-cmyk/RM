# 云台双轴与双摩擦轮遥控测试

当前目标为STM32H723VGT6。pitch/yaw均使用 LQI，当前 Manual 模式直接装载手填 K，不需要 A/B；可选 BootSolve 模式上电求解一次，届时须先填写模型。yaw连续旋转，pitch受端点方向保护。UART7 FireWater三通道和CAN2双摩擦轮逻辑不变；拨弹与双板通信仍关闭。

## UART7 pitch FireWater 遥测

- VOFA+ 串口设为 **921600、8N1**、无流控，数据引擎选 **FireWater**。固件UART7 TX为PE8、RX为PE7；输出只需UART7 TX接USB-TTL RX，并共地，请按实际板卡接口核对接线。
- `ENABLE_ENCODER_UART7=true`，旧回环任务`ENABLE_UART_SERIAL=false`。每50ms输出例如：

```text
12.000,10.000,0.230
```

- 三个固定通道依次为CH1=pitch位置目标（°）、CH2=pitch反馈实际位置（°）、CH3=pitch反馈实际转矩电流（A），均保留三位小数、逗号分隔、换行结束，不输出字段名。前两项都以水平编码器零位893为`0°`；CH3按电流模式`原始值 × 3/16384`换算，不是CAN下发电流指令。未收到反馈或最近反馈超过100ms时，CH2/CH3输出`0.000`占位；这只影响显示，不改变电机使能。每50ms采样，不适合观察高频抖动。
- `kEncoderCalibrationOnly=true`注册两轴反馈并无条件发送零电流，保留用户的位置模式/PID配置，不需要编造限位即可编译采集。先支撑云台，双拨杆保持DOWN，手动移动记录中间零位、下限、上限的计数，避免重力下坠或碰撞。
- 当前`kEncoderCalibrationOnly=false`、`kPitchPositionMode=true`：pitch水平零位893、代码配置的上顶点120、下顶点1490。位置目标限制在两顶点各内收10°的范围；进入保护区后只允许手动向水平零位退回，松杆或向外拨时Disable；回到安全区才恢复位置保持，不会自动回正。yaw正前方3741，保留连续多圈位置控制，不上电自动回正。
- 开始测试时先支撑云台并放在水平893附近，短时小幅试拨。pitch松杆保持位置目标，重力下仍须注意机构和电流；重新标定可将`kEncoderCalibrationOnly`改回true，先停机再手动移动。

## 接线与用户确认的 ID

| 部件 | 总线 | 电调 ID | 反馈 ID | 指令帧槽位 |
|---|---|---|---|---|
| pitch GM6020 | FDCAN1，PD0 RX / PD1 TX | ID1 | 0x205 | 0x1FE 电流帧 DATA[0:1] |
| yaw GM6020 | FDCAN1，PD0 RX / PD1 TX | ID2 | 0x206 | 0x1FE 电流帧 DATA[2:3] |
| 右摩擦轮 M3508/C620 | FDCAN2，PB5 RX / PB6 TX | ID2 | 0x202 | 0x200 DATA[2:3] |
| 左摩擦轮 M3508/C620 | FDCAN2，PB5 RX / PB6 TX | ID1 | 0x201 | 0x200 DATA[0:1] |

两路均为 Classic CAN、1Mbps。以上引脚是当前固件复用配置，板上 CAN 插座对应关系须按实际板卡确认。CAN2的两轮合并一个0x200帧，DATA[4:7]保持零。标定模式下CAN1两个槽位均为零电流。

GM6020仍用电流控制，不发送0x1FF/0x2FF电压帧。两台电机在RoboMaster Assistant中应开启“电流环开关”（官方说明固件>=1.0.11.2，Assistant>=2.7）。pitch/yaw使用 LQI 直接计算电流，MCU 不再执行两轴 PID，内部电流环由驱动器完成；模型、Q/R 和积分限幅见 `../gimbal/GimbalConfig.hpp`。

协议依据：[GM6020 官方 v1.4 使用说明](https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E20231013.pdf)，第5至7页。

ID 不同应改 `../gimbal/GimbalConfig.hpp` 的 `kPitchMotorFeedbackId` 和 `ShootConfig.hpp` 的左右轮反馈 ID。两轮 ID 必须唯一。方向默认 pitch/右轮正向、左轮反向，待上板核对；输出与反馈的方向同时变换。

## 操作

1. 移除弹丸、确认机构可安全运动，摇杆居中，双拨杆 DOWN，保持至少三个安全任务周期（任务每 1ms 更新）。
2. 填写两轴模型并确认 RTT 启动日志显示 LQI ready 后，左拨杆保持DOWN，右拨杆改UP解除急停。右摇杆上下推动pitch位置目标、左右推动yaw位置目标；两轴松杆保持目标。方向异常立即双DOWN。
3. 左拨杆UP，两轮共同爬升至转子1000RPM（用户当前配置1000RPM/s）；左拨杆MIDDLE或DOWN两轮同时关闭。
4. 右拨杆保持 UP 即可测试；MIDDLE/DOWN 不会驱动拨弹，因为拨弹电机没有注册。
5. 双拨杆 DOWN 急停全部输出。遥控超过 100ms 无合法帧也立即禁止输出，恢复后重新执行解锁。

## 测试模式边界

- 已移除电机反馈超时停机。pitch/yaw位置模式均需首帧定位；之后反馈中断仍可能持续发送电流，位置保护依赖最近反馈，不能代替机械安全措施。
- 双轮保留共同启停、相同目标和爬升，以及“两轮转速都达标才就绪”的联动；未新增转速差交叉补偿。
- 遥控失联、解锁、双DOWN急停、限幅、非法数值/异常周期检查仍保留。pitch位置模式进入端点保护区后临时用速度环单向向内退回；yaw连续位置模式不变。
- 移除反馈超时重锚后，反馈中断期间转过半圈以上可能使累计角度失真，不适合直接验证单发位置精度。
- 摩擦轮关闭后发送零电流，不执行受控减速。两轴当前用手填 K；切到 BootSolve 后求解失败不会自动回退到手填 K 或 PID。尚未进行板上整定。
- yaw已启用；恢复拨弹需修改`kEnableFeeder`，旧拨弹ID1已被左轮占用，必须先选择其他拨弹ID，否则编译期校验会拒绝启用。还需重新确认CAN资源、拨盘参数与反馈保护。

## 验证

```powershell
cmake --build --preset Debug --parallel
cmake --build --preset Release --parallel
cmake --build build/host-tests --parallel
ctest --test-dir build/host-tests --output-on-failure
```

宿主机测试覆盖实际DJI/CAN驱动GM6020 0x1FE/0x2FE电流帧、协议和应用限幅、双轴槽位输出与禁用清零、RPM单格跳变电流、CAN1/CAN2分组、无反馈和超时仍输出、yaw连续多圈目标、pitch用户配置限位、双轮联动及关闭拨弹；编译与测试不替代板上验证。
