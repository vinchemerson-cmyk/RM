# 电机模块入口

本目录不创建任务，不决定遥控映射或比赛模式，也不接入底盘草稿。

| 文件 | 职责 |
|---|---|
| motor_common.h | 方向、反馈来源、闭环标志及基础反馈结构，不放品牌控制器 |
| angle_target.h | 现有角度目标及软限位，仍含DJI编码器工具，尚未通用化 |
| dji/dji.h、dji.cpp | DJI初始化、反馈解析、闭环计算、同帧电流合并发送 |
| dji/dji_controller.h、cpp | DJI独有的运行PID、配置、可选PID初始化 |
| dm/dmmotor.h、cpp | 达妙MIT唯一实例、注册、收发、启停 |
| dm/mit_protocol.h | 不依赖HAL的MIT量程映射和8字节编解码 |
| dm/dm.h | 旧包含路径兼容入口，只转发dmmotor.h，不再声明第二套未实现驱动 |

## 使用顺序

DJI保留现有接口：配置 → DJI_Motor_Init → 写目标 → Enable → 周期Control。
云台应用层LQI直接给电流码；摩擦轮/拨弹PID仍保留。未改变现有PID数值、CAN ID和方向约定。
同一总线、同一电流帧的全部电机必须一次提交，不能多个任务分别覆盖槽位。
current_PID字段保留接口兼容，但当前软件计算不执行电流PID，电流内环由电调完成。

达妙只用MIT：MIT_Init_Config_s → MITMotorInit → 设置MIT目标 → 显式DMMotorEnable →
外部1ms周期调用DMMotorTask。注册默认停止，不自动创建任务、清错或重启电机。
普通模式DMMotorInit/SetRef/OuterLoop/ChangeFeed和NORMAL_MODE分支已经移除。
原DM_Motor_s/DM_Motor_Init等未实现的第二套接口也移除，调用者应使用DMMotorInstance。

## 单位边界

应用层应以机构输出轴deg、deg/s工作；但本轮不同时迁移控制器的数值坐标：
- DJI反馈仍为转子deg/deg/s，直接current为协议指令码，GM6020按现有A换算。
- DM MIT目标/反馈为电机协议rad、rad/s、N·m。协议可能已经包含电机减速箱，
  外部机构额外传动比需按实际硬件确认，禁止默认再次除减速比。
输出轴统一适配、PID/LQI模型换算是后续独立工作，本轮没有伪称完成。

## 安全与验证范围

MIT初始化默认停机是行为变化；显式Enable只表示0xFC已入队，必须等到新鲜的
使能反馈才能输出目标。等待超时、反馈过期、驱动故障、非有限指令或发送失败锁存停止。
恢复需上层明确重新使能，不会自动清除故障；重设零位只允许软件停止，入队不等于完成。
软件Stop将目标清零并发送零增益/转矩；硬件Disable另发0xFD。量化后零转矩存在半LSB误差，
软件停止不是安全认证的硬件失能，也不是物理瞬间停止。支撑机构失去转矩可能下坠。

CAN模板改用classicCan，因为MIT是8字节经典帧，BSP按外设FrameFormat分发接收；
原Fdcan模板与经典外设混用会注册到另一张表。硬件仍是STM32 FDCAN。
当前接口沿用master_id为发送ID、can_id为接收ID，不再做另一次全工程重命名。
驱动限定MIT命令ID1..15且反馈低4位ID与其一致；不支持一个共享反馈CAN ID下复用多电机。
注册前必须确保CAN资源无重复，协议量程与实际电机配置一致。
`dmProtocol::Limits` 的默认值取自达妙说明书与官方 dm4310 例程的出厂预设
（Pos ±12.5 / Vel ±30 / Torque ±10，KP 0~500，KD 0~5），4310 与 4340 相同；
但量程是"驱动器内可改的设定值"，若在达妙调试助手里调过范围，必须用 `LimitsFor()` 覆盖，
否则下发命令会被等比例缩放且不报错。

所有目标写入、使能、停止及DMMotorTask应由同一控制任务串行调用，反馈由ISR更新。
不要在ISR调用注册、启停或任务函数；任务读取反馈应采用临界区快照。
两次有效采样跨越超过半个位置范围时，多圈解算无法恢复真实圈数。

本轮不修改Application/Robot.cpp，不接入底盘，不改HAL生成代码。
完整工程仍需单独核对底盘中的未完成类型和其它脏改动；编译或协议自检不能证明上板安全。
