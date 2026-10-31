/** @file chassis.cpp @brief 四全向轮控制与快照接口；当前被注释的初始化校验见Init说明。 */
#include "chassis.h"
#include "bsp_log.h"
#include "ChassisConfig.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "safety/safety.h"

namespace
{
	// 匿名命名空间内的状态仅供本文件使用，对外通过Init和快照接口访问。
	// 四轮各自保存电机实例和快照；PI历史位于motor->controller.speed_PID，不在这里重复保存。
	struct ChassisSingleMotor
    {
        DJI_Motor_s* motor{}; // DJI_Motor_Init返回的实例；nullptr表示尚未注册成功。
        chassis::WheelControlSnapshot snapshot{}; // 最后一个控制周期的诊断数据，供其他任务读取。
    };
    std::array<ChassisSingleMotor, chassis::kWheelCount> wheels{}; // 轮序始终为FL/FR/BL/BR。
    chassis::WheelVelocityPlanner planner{}; // 车体斜坡、全向轮逆解及统一轮速限幅。
    osThreadId_t taskHandle{}; // RTOS任务句柄，不是电机指针；非空表示创建任务成功。
    bool registrationAttempted{}; // 记录是否开始占用驱动静态池；部分失败后禁止自动重复注册。
    uint32_t periodTicks{}; // 绝对调度周期，单位tick；原设计由ValidateConfiguration赋值。
    constexpr osThreadAttr_t taskAttrs{
        .name = "ChassisTask", .stack_size = 2048U, .priority = osPriorityHigh, // 栈大小单位为字节。
    };

    /** @brief 对归一化摇杆量去死区，并把剩余行程连续映射到[-1,1]，保留方向。 */
    float Deadband(float stick)
    {
        // 例如死区0.03：[-0.03,+0.03]输出0，避免摇杆中位噪声导致车体缓慢漂移。
        if (std::fabs(stick) <= kChassisStickDeadband) return 0.0F;
        // 减去死区后再除以剩余行程，使刚离开死区时从0平滑起步，打满时仍能到1。
        // copysign把原来的正负方向附加到计算后的幅值；调用方先用Normalize限制输入。
        return std::copysign((std::fabs(stick) - kChassisStickDeadband) /
                            (1.0F - kChassisStickDeadband), stick);
    }

    /** @brief 用RTOS tick判断一份数据是否仍在毫秒超时范围内；恰好到达超时点即失效。 */
    bool Fresh(uint32_t now, uint32_t sample, uint32_t timeoutMs)
    {
        // now-sample先按uint32_t求差，允许正常计数回绕；调用方仍须区分从未收到数据。
        // 比较“tick差*1000”和“毫秒上限*tick频率”，避免默认tick必然等于1ms。
        // 乘法前提升为uint64_t，减少长时间差乘1000时的整数溢出风险。
        return static_cast<uint64_t>(now - sample) * 1000U <
               static_cast<uint64_t>(timeoutMs) * osKernelGetTickFreq();
    }

    /** @brief 为一轮生成速度PI配置；仅填写参数，不注册、不使能、不发CAN。 */
    DJI::Motor_Init_Config_s MakeChassisMotorConfig(const ChassisWheelHardware& hardware)
    {
        DJI::Motor_Init_Config_s config{}; // 值初始化：未使用的角度/电流PID及外部反馈指针保持零配置。
        config.motor_type = hardware.type;
        config.controller_param_init_config.speed_PID = kChassisWheelSpeedPi;
        auto& setting = config.controller_setting_init_config;
        setting.close_loop_type = SPEED_AND_CURRENT_LOOP; // MCU速度PI算电流指令，电调内部闭合电流环。
        setting.outer_loop_type = SPEED_LOOP; // 记录外层控制目标为速度，不要求车轮保持累计转角。
        // 驱动统一处理反馈和输出方向，应用层始终使用几何配置所定义的逻辑正轮速。
        setting.motor_reverse_flag = hardware.reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
        setting.angle_feedback_source = MOTOR_FEED;
        setting.speed_feedback_source = MOTOR_FEED;
        setting.feedforward_flag = FEEDFORWARD_NONE; // 当前只用速度误差PI，不叠加外部速度/电流前馈。
        config.can_init_config.can_handle = kChassisCanHandle;
        config.can_init_config.can_id = hardware.feedback_id; // 接收电机反馈ID；发送分组及槽位由DJI驱动解析。
        return config;
    }

    /** @brief 清四轮速度/PI状态及车体斜坡；本函数本身不向CAN发送停止帧。 */
    void StopWheels()
    {
        planner.Reset(); // 避免恢复输出时继续追逐停机前的车体目标。
        for (auto& wheel : wheels) DJI_Motor_Disable(wheel.motor); // 驱动允许空指针。
    }

    /**
     * @brief 在控制计算后保存单轮诊断快照，不是串口发送或消息中心发布函数。
     * @param index 已确认有效的FL/FR/BL/BR索引，调用方保证小于kWheelCount。
     * @param tick 本周期RTOS tick；不是毫秒时间戳。
     * @param dt 本周期实际秒数，用于观察调度抖动，不在这里替换异常值。
     */
    void PublishWheelControlSnapshot(unsigned index, uint32_t tick, float dt)
    {
        auto& wheel = wheels[index];
        if (wheel.motor == nullptr) return;
        chassis::WheelControlSnapshot snapshot{};
        const auto& motor = *wheel.motor;
        // 反馈由CAN中断更新，短临界区中复制，避免角速度/时间戳来自不同帧。
        // 电机控制状态由本底盘任务持有，下面的单位换算不需要长时间关闭中断。
        taskENTER_CRITICAL();
        const auto feedback = motor.feedback;
        snapshot.control_sequence = wheel.snapshot.control_sequence + 1U; // 每次快照递增，允许uint32_t循环回绕。
        taskEXIT_CRITICAL();
        snapshot.control_tick = tick;
        snapshot.control_dt_s = dt;
        // 驱动内部为转子°/s；除以每轮实际减速比，向应用和遥测提供输出轴°/s。
        // gear_ratio必须在运行前确认非零，不能用未知减速比进行换算。
        snapshot.target_speed_dps = motor.enabled
            ? motor.motor_settings.speed / kChassisWheels[index].gear_ratio : 0.0F;
        snapshot.actual_speed_dps = feedback.speed / kChassisWheels[index].gear_ratio;
        // 这是PI逻辑坐标的电流指令码，不是安培，也不是方向翻转/量化后的最终CAN数据。
        snapshot.command_current_raw = motor.enabled ? motor.controller.speed_PID.Output() : 0.0F;
        snapshot.feedback_current_a = feedback.current; // 已由驱动解码并统一反馈方向，单位A。
        snapshot.output_enabled = motor.enabled;
        snapshot.feedback_seen = feedback.angle_initialized;
        snapshot.feedback_tick = feedback.last_update_tick;
        snapshot.tx_attempts = motor.tx_enqueue_attempts;
        snapshot.tx_failures = motor.tx_enqueue_failures;
        // 一次替换整个快照；读取接口使用同样的临界区保护，避免其他任务读到半更新状态。
        taskENTER_CRITICAL();
        wheel.snapshot = snapshot;
        taskEXIT_CRITICAL();
    }

    /** @brief 四轮全部收到过反馈、转速有限且反馈未超时，才允许整组闭环输出。 */
    bool FeedbackReady(uint32_t tick)
    {
        bool ready = true;
        taskENTER_CRITICAL();
        for (const auto& wheel : wheels)
        {
            if (wheel.motor == nullptr) { ready = false; break; } // 少一轮也不按剩余三轮继续驾驶。
            const auto& feedback = wheel.motor->feedback;
            // angle_initialized在这里用作“收到首帧”标志，不表示底盘需要角度环。
            if (!feedback.angle_initialized || !std::isfinite(feedback.speed) ||
                !Fresh(tick, feedback.last_update_tick, kChassisFeedbackTimeoutMs))
            { ready = false; break; }
        }
        taskEXIT_CRITICAL();
        return ready;
    }

    /** @brief 周期执行：等截止时间 -> 取指令/安全条件 -> 四轮目标 -> PI/CAN -> 快照。 */
    void Task(void*)
    {
        uint32_t lastTick = osKernelGetTickCount(); // 用实际两次采样时刻计算dt，不把标称周期当成实测值。
        uint32_t wakeTick = lastTick; // 绝对调度时间，与lastTick用途不同。
        bool txFault = false; // CAN入队失败锁存；不是电调反馈的故障状态。
        // Task只会在四轮注册完成后创建，故这里取得的是有效指针，不是启动时复制的四个nullptr。
        DJI_Motor_s* motors[chassis::kWheelCount]{};
        for (unsigned i = 0; i < chassis::kWheelCount; ++i) motors[i] = wheels[i].motor;

        for (;;)
		{
			// 绝对调度；若工作已跨过截止时间，重新对齐，避免连续补跑。
			wakeTick                  += periodTicks;
			// 无符号差转有符号用于判断截止时间是否已到，前提是周期/时间差在半个计数范围内。
			const uint32_t beforeWait = osKernelGetTickCount();
			if (static_cast<int32_t>(wakeTick - beforeWait) <= 0) wakeTick = beforeWait + periodTicks;
			const bool     waited = osDelayUntil(wakeTick) == osOK;
			const uint32_t now    = osKernelGetTickCount();
			// tick差先用无符号整数求出，再转float秒；1kHz下1 tick为0.001秒。
			// 不能用整数秒，否则毫秒周期会被截断成0，PI积分和加速度计算失效。
			const float    dt     =
				static_cast<float>(now - lastTick) /
				static_cast<float>(osKernelGetTickFreq());
            lastTick = now;
            // 等待失败、同tick重复运行或周期过长均禁止运动，不用裁剪dt掩盖异常。
            const bool timingValid = waited && std::isfinite(dt) && dt > 0.0F && dt <= kChassisMaxDtS;
            const auto remote = rc::Snapshot();
            // 使用同一份遥控快照的received和原始更新时间，避免读取多份不同帧的状态。
            const bool remoteValid = remote.received && Fresh(now, remote.last_update_tick, rc::OFFLINE_TIMEOUT_MS);
            const bool runRequested = remote.switch_left == kChassisRunSwitch; // 当前按左拨杆决定是否请求底盘运行。
            // 发送故障须先离开运行拨杆位置再进入，不在故障后自动续转。
            if (remoteValid && !runRequested)
            	txFault = false;
            // 任一条件不满足都停整组；安全许可不等于已有可靠轮速反馈，二者分别检查。
            bool allowed = timingValid && remoteValid && runRequested && !txFault &&
                           safety::ShouldOutput() && FeedbackReady(now);
            std::array<float, chassis::kWheelCount> target{};
            if (allowed)
            {
                // 输入处理顺序：原始DBUS通道 -> [-1,1] -> 连续死区 -> 方向映射 -> 物理速度。
                // Sign只用于修正摇杆方向，不改变右手坐标系，也不修正单个电机安装反向。
                const float vx = Deadband(rc::Normalize(remote.channels[kChassisForwardChannel])) *
                                 kChassisForwardSign * kChassisMotionLimits.max_translation_mps;
                const float vy = Deadband(rc::Normalize(remote.channels[kChassisLeftChannel])) *
                                 kChassisLeftSign * kChassisMotionLimits.max_translation_mps;
                const float yaw = Deadband(rc::Normalize(remote.channels[kChassisYawChannel])) *
                                  kChassisYawSign * kChassisMotionLimits.max_yaw_dps;
                allowed = planner.Update(vx, vy, yaw, dt, target); // 返回输出轴°/s；计算失败同样进入停机分支。
            }
            if (!allowed) StopWheels();
            else
            {
                for (unsigned i = 0; i < chassis::kWheelCount; ++i)
                {
                    auto& motor = *motors[i];
                    if (!motor.enabled) DJI_Motor_Enable(&motor); // 每周期都Enable会反复清积分，所以只在状态切换时调用。
                    motor.motor_settings.speed = target[i] * kChassisWheels[i].gear_ratio; // 输出轴°/s转为转子°/s。
                    motor.motor_settings.current = 0.0F; // 不额外叠加电流偏置，最终指令由速度PI产生。
                }
            }
            // 发送前记录累计失败计数，发送后比较是否变化，用于判断“本次”是否发生入队失败。
            uint32_t failures[chassis::kWheelCount]{};
            for (unsigned i = 0; i < chassis::kWheelCount; ++i) failures[i] = motors[i]->tx_enqueue_failures;
        	//发送控制帧
            DJI_Motor_Control(motors, chassis::kWheelCount, timingValid ? dt : kChassisControlPeriodS);
            for (unsigned i = 0; i < chassis::kWheelCount; ++i)
                if (motors[i]->tx_enqueue_failures != failures[i]) txFault = true;
            if (txFault)
            {
                StopWheels();//电机电流归零
                // 最多补发一次零指令，不无限重试；不能撤回已入队的旧帧，也不能保证总线故障时成功送达。
                DJI_Motor_Control(motors, chassis::kWheelCount, kChassisControlPeriodS);
            }
            // 放在PI与发送之后，快照才能包含本周期输出和入队计数；四轮共享本周期时间标记。
            for (unsigned i = 0; i < chassis::kWheelCount; ++i) PublishWheelControlSnapshot(i, now, dt);
            if (!waited) osDelay(periodTicks); // 正周期时用于让出CPU；periodTicks仍为0时不能保证有效等待。
        }
    }

    // 下列完整配置校验目前被手动注释，本次仅补说明，不恢复执行。
    // 原职责：确认硬件开关/CAN模式、计算periodTicks、检查摇杆映射和PI、核对四轮ID/减速比，
    // 最后调用planner.Configure缓存几何。校验并非只检查参数，还承担这些必要的初始化副作用。
    // 注意：当前没有执行此函数，periodTicks仍为0，planner也未配置；不能据此状态直接上板运行。
    bool ValidateConfiguration()
    {
        if (!kChassisHardwareConfirmed || kChassisCanHandle == nullptr ||
            kChassisCanHandle->Init.FrameFormat != FDCAN_FRAME_CLASSIC ||
            !chassis::ControlPeriodTicks(kChassisControlPeriodS, osKernelGetTickFreq(), periodTicks) ||
            !std::isfinite(kChassisMaxDtS) || kChassisMaxDtS < kChassisControlPeriodS ||
            kChassisFeedbackTimeoutMs == 0U || kChassisFeedbackTimeoutMs >= 0x80000000U ||
            kChassisForwardChannel >= rc::CHANNEL_COUNT || kChassisLeftChannel >= rc::CHANNEL_COUNT ||
            kChassisYawChannel >= rc::CHANNEL_COUNT ||
            !std::isfinite(kChassisStickDeadband) || kChassisStickDeadband < 0.0F || kChassisStickDeadband >= 1.0F)
            return false;
        for (float sign : {kChassisForwardSign, kChassisLeftSign, kChassisYawSign})
            if (sign != 1.0F && sign != -1.0F) return false;
        PIDInstance pi;
        if (!pi.Configure(kChassisWheelSpeedPi) || kChassisWheelSpeedPi.kd != 0.0F ||
            kChassisWheelSpeedPi.kp <= 0.0F || kChassisWheelSpeedPi.ki < 0.0F) return false;
        for (unsigned i = 0; i < chassis::kWheelCount; ++i)
        {
            const auto& hardware = kChassisWheels[i];
            if ((hardware.type != DJI::M3508 && hardware.type != DJI::M2006) ||
                hardware.feedback_id < 0x201U || hardware.feedback_id > 0x208U ||
                !std::isfinite(hardware.gear_ratio) || hardware.gear_ratio <= 0.0F) return false;
            const float peakRotorSpeed = hardware.gear_ratio * kChassisGeometry[i].max_output_speed_dps;
            if (!std::isfinite(peakRotorSpeed) || peakRotorSpeed > std::numeric_limits<int16_t>::max() * 6.0F)
                return false;
            const float commandLimit = hardware.type == DJI::M3508 ? 16384.0F : 10000.0F;
            if (kChassisWheelSpeedPi.output_max > commandLimit || kChassisWheelSpeedPi.output_min < -commandLimit)
                return false;
            for (unsigned j = 0; j < i; ++j)
                if (hardware.feedback_id == kChassisWheels[j].feedback_id) return false;
        }
        return planner.Configure(kChassisGeometry, kChassisMotionLimits);
    }
}

bool chassis::Init()
{
    if (taskHandle != nullptr)
    	return true; // 幂等：任务已创建时直接成功，不重复注册或创建任务。
    // 必须在RTOS初始化完成、调度器启动前调用；注册失败后也不自动重试，避免重复占用静态池。
    if (registrationAttempted || osKernelGetState() != osKernelReady)
        return false;
    // 注意：配置校验入口当前被注释，kChassisHardwareConfirmed=false不会在这里阻止注册。
    // 因而Init不再具备“硬件未确认就零注册”的原保证；本次注释工作不改变你的该处代码状态。
    if (!ValidateConfiguration())
    {
        LOGERROR("[chassis] hardware/configuration incomplete; no motors registered");
        return false;
    }
    registrationAttempted = true; // 在首次注册前置位，后续任何部分失败均不允许重复尝试。
    for (unsigned i = 0; i < kWheelCount; ++i)
    {
        // 先为每轮生成独立配置，再交给驱动注册；成功返回的电机实例初始保持禁用。
        wheels[i].motor = DJI_Motor_Init(MakeChassisMotorConfig(kChassisWheels[i]));
        if (wheels[i].motor == nullptr)
        {
            StopWheels();
            LOGERROR("[chassis] wheel %u registration failed; task not created", i);
            return false; // 驱动池无注销接口，不重试以免重复占用。
        }
    }
    taskHandle = osThreadNew(Task, nullptr, &taskAttrs); // 四轮均注册成功后，才创建拥有它们的控制任务。
    if (taskHandle == nullptr)
    {
        StopWheels();
        LOGERROR("[chassis] task creation failed; motors remain disabled");
        return false;
    }
    return true;
}

chassis::WheelControlSnapshot chassis::GetWheelControlSnapshot(Wheel wheel)
{
    const auto index = static_cast<unsigned>(wheel);
    if (index >= kWheelCount)
    	return {}; // 非法枚举值返回全零结构，不访问越界数组。
    // 返回值拷贝，不暴露内部快照地址；读取与写入使用相同临界区，保证快照整体一致。
    taskENTER_CRITICAL();
    const auto snapshot = wheels[index].snapshot;
    taskEXIT_CRITICAL();
    return snapshot;
}

serialTasks::ChassisSnapshot chassis::GetTelemetrySnapshot()
{
    serialTasks::ChassisSnapshot snapshot{};
    constexpr Wheel kTelemetryOrder[kWheelCount]{
        Wheel::FrontLeft, Wheel::FrontRight, Wheel::BackLeft, Wheel::BackRight,
    };
    for (unsigned i = 0; i < kWheelCount; ++i)
    {
        const WheelControlSnapshot wheel = GetWheelControlSnapshot(kTelemetryOrder[i]);
        snapshot.wheels[i].target_speed_dps   = wheel.target_speed_dps;
        snapshot.wheels[i].actual_speed_dps   = wheel.actual_speed_dps;
        snapshot.wheels[i].feedback_current_a = wheel.feedback_current_a;
    }
    return snapshot;
}
