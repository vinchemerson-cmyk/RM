# 底盘草稿：未接入实机

参考 `RM/05-兵种开源/basic_framework-master/application/chassis/chassis.c`（NeoZeng）
的流程组织：获取命令 → 模式处理 → 云台/底盘坐标变换 → 四全向轮解算 → 限幅 → 状态反馈。
这里重写为独立C++纯逻辑，不复制参考中的PID参数、硬件ID和功率控制接口。
参考目录的mecanum.h/steering.h目前没有可复用的算法实现。根据用户确认，本稿仅支持
四全向轮，不使用参考的麦克纳姆公式，也不实现舵轮。假定轮心位于矩形四角，
每轮主动滚动方向由drive_direction_deg配置，实车方向与轮距待确认。

## 文件和边界

- ChassisDraft.hpp：配置、命令、反馈、模式和四轮目标/速度估计。
- ChassisDraftBus.hpp：调用当前SubRegister/PubRegister等接口的板内适配草稿。
- ChassisDraftCheck.cpp：独立电脑端数学自检，不是固件源码，不加入正式CMake。

保留现有chassis.cpp/chassis_control.h及其未提交修改；不修改Robot.cpp或根CMake，
不创建RTOS任务、不注册CAN、电机，不调用DJI_Motor_Control。
默认Config参数为占位，commissioned=false；只有明确配置并确认后才产生运动请求。

## 坐标与单位

右手系x前、y左、z上，逆时针yaw为正，和参考文件y右的描述不同，不能混用正负号。
平移m/s，底盘yaw角速度deg/s，轮速为实际车轮轴deg/s；轮序FL/FR/RL/RR。
每个轮的正轮速定义为沿drive_direction_deg方向主动滚动，电机安装反向由适配层统一处理。
对每轮，轮心(x,y)处的速度为(vx-y*wz, vy+x*wz)，投影到驱动方向(cosθ,sinθ)，
再除轮半径并换成deg/s。注意角度是滚动方向，不是轮轴方向；不能照抄样例角度。
因此四轮速度一般不会在纯前进时同号，实际符号取决于各轮主动方向定义。
转动中心暂固定为底盘几何中心，不支持参考中的云台偏置旋转中心补偿。
Gimbal坐标速度通过云台相对底盘yaw转换；Follow模式使用有界比例跟随，
而非未经单位核对照抄参考的平方跟随增益与自旋常量4000。

轮速限幅按四轮同比例缩放，不是功率控制。速度估计只使用轮速，不检测打滑、
不融合IMU，也不代表可靠里程计。没有反馈不阻止纯逻辑生成请求；
实际执行器适配层必须再决定反馈失效处理，不能直接视作实车安全许可。

## 接入草图（尚未执行）

1. 初始化前填写Config：实测轮径、轴距、轮距、输出轴速度限制，以及四个驱动方向角。
   不可控的退化布局会被拒绝；非矩形四角安装需要进一步扩展逐轮位置配置。
2. 启动前创建BusEndpoint并Init，同时命令端注册chassis_draft_cmd，状态端订阅
   chassis_draft_state。每个话题必须使用相同结构和长度；目前深度1只保存最新值。
3. 周期任务调用Tick(safety_allowed, feedback, now_ms, dt_s)。
   无新消息可在有效期内使用旧命令，超时/禁用/安全撤销返回零目标且output_requested=false。
   source_tick_ms必须来自原始有效指令；当前板内使用同一tick时基。
   安全状态撤销后丢弃旧指令，仍须外部安全模块重新解锁，不可只依据新命令恢复。
4. 执行器适配层将返回目标写入四轮，禁用时也须持续落实零输出。
   当前DJI驱动若仍使用转子速度反馈，则使用总传动比将输出轴目标转回转子单位，
   控制器内部反馈和增益保持同一坐标；不要只改目标或反馈的一边。
5. 四轮同总线同指令帧须一次提交，不能与摩擦轮等任务在同帧各发一半覆盖槽位。
   CAN总线、ID、方向、传动比未确认，因此本稿刻意不提供可直接上电的电机注册配置。
6. 裁判功率限制、超级电容、真实速度估计融合、轮速变化率限制、双板序列化均未实现。
   量产接入前必须补充这些策略及驱动反馈有效性检查，先架空低速验证。

消息只含值，不包含指针、HAL句柄或PID对象。板内可memcpy，跨板不得直接假定
结构体字节布局、时钟或浮点编码一致；当前Result也未承诺符合interboard的48字节上限。

## 独立验证

在仓库根目录执行（输出目录需已存在）：

```powershell
g++ -std=c++20 -Wall -Wextra -Werror -I Modules Application/chassis/draft/ChassisDraftCheck.cpp -o build/chassis-draft-check.exe
./build/chassis-draft-check.exe
```

只验证数学、模式和停止条件；不证明电机方向、CAN发送、安全动作或功率限制正确。
