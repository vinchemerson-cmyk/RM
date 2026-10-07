# 云台双轴与双摩擦轮遥控测试

当前目标为STM32H723VGT6。pitch 使用 LQI 位置保持，yaw 当前为独立速度反馈测试（Manual），无位置保持和积分。pitch 可选 BootSolve，届时须先填写模型。UART7 输出 yaw 六通道诊断，CAN2双摩擦轮逻辑不变；拨弹与双板通信仍关闭。

## UART7 yaw FireWater 诊断

- VOFA+ 串口设为 **921600、8N1**、无流控，数据引擎选 **FireWater**。固件UART7 TX为PE8、RX为PE7；输出只需UART7 TX接USB-TTL RX，并共地，请按实际板卡接口核对接线。
- `ENABLE_ENCODER_UART7=true`，旧回环任务`ENABLE_UART_SERIAL=false`。每10ms输出例如：

```text
30.000,10.000,6.000,0.480,1,0.450
```

- 六个固定通道依次为CH1=yaw目标速度（°/s）、CH2=yaw反馈连续角度（°）、CH3=yaw实际速度（°/s）、CH4=yaw指令电流（A）、CH5=反馈年龄（ms）、CH6=yaw反馈转矩电流（A）。电流、角度和速度保留三位小数，年龄为整数。角度不是以正前方为零的单圈角度；电流是逻辑指令，不是反馈实际电流或发送成功证明。未收到反馈时年龄为-1，过期时保留最近反馈。每10ms采样，不能解析高频振荡。yaw 当前仅速度反馈测试：打满 ±60°/s、增益0.03 A/(°/s)、限流 ±1.8A，静止打满输出 ±1.8A，无位置保持和积分；松杆目标速度为零，静止后零电流。pitch、摩擦轮和遥控保护不变。
- `kEncoderCalibrationOnly=true`注册两轴反馈并无条件发送零电流，保留用户的位置模式/PID配置，不需要编造限位即可编译采集。先支撑云台，双拨杆保持DOWN，手动移动记录中间零位、下限、上限的计数，避免重力下坠或碰撞。
- 当前`kEncoderCalibrationOnly=false`、`kPitchPositionMode=true`：pitch水平零位893、代码配置的上顶点120、下顶点1490。位置目标限制在两顶点各内收10°的范围；进入保护区后只允许手动向水平零位退回，松杆或向外拨时Disable；回到安全区才恢复位置保持，不会自动回正。yaw正前方3741，当前速度测试不保持位置、不自动回正。
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
2. 确认 RTT 启动日志显示 LQI ready 后（当前 Manual 不需要模型），左拨杆保持DOWN，右拨杆改UP解除急停。右摇杆上下推动pitch位置目标、左右控制yaw速度；pitch松杆保持位置，yaw松杆目标速度为零。方向异常立即双DOWN。
3. 左拨杆UP，两轮共同爬升至转子1000RPM（用户当前配置1000RPM/s）；左拨杆MIDDLE或DOWN两轮同时关闭。
4. 右拨杆保持 UP 即可测试；MIDDLE/DOWN 不会驱动拨弹，因为拨弹电机没有注册。
5. 双拨杆 DOWN 急停全部输出。遥控超过 100ms 无合法帧也立即禁止输出，恢复后重新执行解锁。

## 测试模式边界

- 已移除电机反馈超时停机。pitch/yaw位置模式均需首帧定位；之后反馈中断仍可能持续发送电流，位置保护依赖最近反馈，不能代替机械安全措施。
- 双轮保留共同启停、相同目标和爬升，以及“两轮转速都达标才就绪”的联动；未新增转速差交叉补偿。
- 遥控失联、解锁、双DOWN急停、限幅、非法数值/异常周期检查仍保留。pitch位置模式进入端点保护区后临时用速度环单向向内退回；yaw当前为独立速度反馈测试。
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
