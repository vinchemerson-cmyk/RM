//
// Created by 19923 on 2026/10/12.
//

#include "chassis.h"

#include "ChassisConfig.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "communication/encoder_telemetry.h"
#include "motor/speed_target.h"
#include "motor/dji/dji.h"
#include "rc/rc.h"
#include "safety/safety.h"


struct DJI_Motor_s;

namespace
{
	osThreadId_t taskHandle = nullptr;

	struct WheelControlSnapshot
	{
		uint32_t control_tick{};
		uint32_t control_sequence{};
		float control_dt_s{};

		float target_speed_dps{};
		float actual_speed_dps{};
		float command_current_raw{}; // 电调指令码，不是 A
		float feedback_current_a{};

		bool output_enabled{};
		bool feedback_seen{};
		uint32_t feedback_tick{};

		uint32_t tx_attempts{};
		uint32_t tx_failures{};
	};

	struct chassissinglemotor
	{
		DJI_Motor_s*                 motor{};
		MotorSpeedTarget             speedTarget;
		// 速度 PI 由 motor->controller.speed_PID 持有，每个电机独立维护积分。
		Motor_Feedback_s             motorFeedback;
		uint32_t                     ControTick{};
		WheelControlSnapshot controlSnapshot{};
	};

	chassissinglemotor fl, fr, bl, br;
	// TODO[初始化]：补充底盘 Init 接口，按实物型号、CAN ID、安装方向分别注册四轮，
	// 检查所有注册结果非空后再创建任务；注册失败时保持禁止输出。
	// TODO[配置]：在 ChassisConfig.h 中集中配置轮径、轮位/滚动方向、减速比及速度/加速度上限；
	// CAN 总线和电调 ID 须核对与同总线其他模块无冲突，不直接套用云台配置。

	constexpr osThreadAttr_t taskAttrs = {
		.name       = "ChassisTask",
		.stack_size = 2048U,
		.priority   = osPriorityHigh,
	};


	// 注册完成后，在控制周期内由 fl/fr/bl/br.motor 组装局部指针数组再统一发送。
	/**
	 * @brief 根据传入的电机类型、CAN 反馈 ID 和方向，生成一个完整且安全的 DJI 电机初始化配置结构体
	 * @param motor_type 电机型号
	 * @param feedback_id 该电机的 CAN 反馈报文 ID
	 * @param reverse 是否反向安装
	 * @return
	 */
	DJI::Motor_Init_Config_s MakeChassisMotorConfig(const DJI::Motor_Type_e motor_type, const uint32_t feedback_id,
											 const bool              reverse)
	{
		DJI::Motor_Init_Config_s config{};
		config.motor_type = motor_type;

		// 使用电机自身反馈，不使用外部反馈与前馈通道。
		config.controller_param_init_config.other_angle_feedback_ptr = nullptr; //其他角度反馈
		config.controller_param_init_config.other_speed_feedback_ptr = nullptr; //其他速度反馈
		config.controller_param_init_config.speed_feedforward_ptr    = nullptr; //速度前馈
		config.controller_param_init_config.current_feedforward_ptr  = nullptr; //电流前馈

		config.controller_param_init_config.speed_PID = kChassisWheelSpeedPi;
		// MCU 计算速度 PI；电流环由电调闭合，不启用 MCU 角度环或电流 PID。
		config.controller_setting_init_config.close_loop_type    = SPEED_AND_CURRENT_LOOP;
		config.controller_setting_init_config.outer_loop_type    = SPEED_LOOP;
		config.controller_setting_init_config.motor_reverse_flag =
			reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
		config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
		config.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
		config.controller_setting_init_config.feedforward_flag      = FEEDFORWARD_NONE;

		config.can_init_config.can_handle = kChassisCanHandle;
		config.can_init_config.can_id     = feedback_id;
		return config;
	}

	Motor_Feedback_s FeedbackSnapshot(const DJI_Motor_s& motor)
	{
		taskENTER_CRITICAL();
		const Motor_Feedback_s feedback = motor.feedback;
		taskEXIT_CRITICAL();
		return feedback;
	}

	void StopWheel(DJI_Motor_s& motor, MotorSpeedTarget& target)
	{
		// Disable 已清除驱动内部 PI 历史；另清速度指令和斜坡状态。
		DJI_Motor_Disable(&motor);
		target.Reset();
	}


	void PublishWheelControlSnapshot(chassissinglemotor& wheel, float dt_s)
	{
		if (wheel.motor == nullptr)
			return;

		WheelControlSnapshot snapshot{};

		// 一次性读取电机反馈和控制状态，避免拷贝过程中被中断更新。
		taskENTER_CRITICAL();

		const auto& motor = *wheel.motor;

		snapshot.control_tick = wheel.ControTick;
		snapshot.control_sequence =
			wheel.controlSnapshot.control_sequence + 1U;
		snapshot.control_dt_s = dt_s;

		// 当前驱动这里使用的是转子 °/s。
		snapshot.target_speed_dps =
			motor.enabled ? motor.motor_settings.speed : 0.0F;
		snapshot.actual_speed_dps = motor.feedback.speed;

		// 在 DJI_Motor_Control() 之后调用，读取本周期 PI 输出。
		// 当前底盘未启用前馈；此值尚不是最终方向处理、量化后的 CAN 码。
		snapshot.command_current_raw =
			motor.enabled ? motor.controller.speed_PID.Output() : 0.0F;

		snapshot.feedback_current_a = motor.feedback.current;
		snapshot.output_enabled = motor.enabled;
		snapshot.feedback_seen = motor.feedback.angle_initialized;
		snapshot.feedback_tick = motor.feedback.last_update_tick;

		snapshot.tx_attempts = motor.tx_enqueue_attempts;
		snapshot.tx_failures = motor.tx_enqueue_failures;

		taskEXIT_CRITICAL();

		taskENTER_CRITICAL();
		wheel.controlSnapshot = snapshot;
		taskEXIT_CRITICAL();
	}

	// 先更新四轮目标速度，再统一计算 PI 并发送。
	// DJI_Motor_Control(motors, 4, dt_s);
	//
	// PublishWheelControlSnapshot(fl, dt_s);
	// PublishWheelControlSnapshot(fr, dt_s);
	// PublishWheelControlSnapshot(bl, dt_s);
	// PublishWheelControlSnapshot(br, dt_s);

	void Task(void*)
	{
		// TODO[周期调度]：另维护绝对唤醒 tick；将控制周期秒数转换为非零整数 tick。
		// 使用 osKernelGetTickFreq() 换算，并检查周期能否由 RTOS tick 分辨率表达。
		uint32_t last_tick = osKernelGetTickCount();

		for ( ; ; )
		{
			const uint32_t now = osKernelGetTickCount();
			// FIXME[单位]：当前差值是 tick 数，不是秒；应除以 osKernelGetTickFreq()。
			// 1 kHz 下相差 1 tick 实际为 0.001 s，否则下方 timingvalid 会错误拒绝输出。
			auto dt_s = static_cast<float>(now - last_tick);
			last_tick = now;

			// TODO[异常周期]：区分首次运行、同 tick 重入和超时；不要只用标称周期掩盖异常。
			// 超时应禁止输出并复位目标/PI；限制用于计算的 dt 不代表本周期恢复有效。
			if (dt_s <= 0.0F)
				dt_s = kChassisControlPeriodS;

			const bool timingvalid = dt_s < kChassisMaxDtS;
			if (!timingvalid)
				dt_s = kChassisMaxDtS;

			const auto remote = rc::Snapshot();
			// TODO[输出条件]：加入命令有效性/控制模式；正式运行前增加四轮首帧和反馈超时检查。
			// 遥控直连与板间命令接入时分别检查实际命令来源的新鲜度，不能只检查遥控在线。
			const bool outputAllowed = timingvalid &&
				rc::IsOnline()&&safety::ShouldOutput();

			// TODO[整车指令]：正常驾驶应将通道映射为 vx、vy、wz，并处理死区与模式开关。
			// 下列四通道各控一轮仅适合明确启用的单轮测试模式，不应直接用作全向底盘驾驶。
			float flStick = rc::Normalize(remote.channels[0]);
			float frStick = rc::Normalize(remote.channels[1]);
			float blStick = rc::Normalize(remote.channels[2]);
			float brStick = rc::Normalize(remote.channels[3]);

			// TODO[目标生成]：对 vx/vy/wz 做加减速处理，按实际全向轮几何进行逆运动学解算。
			// 若四轮任一超速，对整组目标等比例缩放，避免分别截断改变底盘运动方向。
			// 避免车体斜坡与逐轮 speedTarget 重复滤波造成额外延迟或比例失真。

			// TODO[单位边界]：应用层统一输出轴 °/s，写 motor_settings.speed 前乘实际减速比，
			// 与驱动转子 °/s 一致；从驱动反馈到应用层时反向换算。统一轮序及正方向。

			// TODO[执行分支]：outputAllowed 为真时更新各轮目标，并仅在需要时 Enable；
			// 禁止输出时对非空电机调用 StopWheel，清除速度目标和 PI 历史，重新使能从零起步。
			// 不要每周期重新 Enable：驱动 Enable 会重置 PI 状态，使积分无法正常累积。

			// TODO[统一发送]：从 fl/fr/bl/br.motor 组装局部数组，调用 DJI_Motor_Control(motors, 4, dt_s)。
			// 停止分支也须发送零输出；同总线同指令帧的电机应由同一调用统一提交，避免槽位互相覆盖。

			// TODO[快照]：统一发送后更新四轮 ControTick，再调用 PublishWheelControlSnapshot。
			// 当前快照速度仍是转子 °/s；串口/消息中心消费者需另行提供接口，快照赋值不等于已发送。

			// TODO[等待]：循环末尾使用 osDelayUntil 等待下一个绝对截止 tick，禁止无等待空转。
			// 截止时间已过时重新对齐调度，避免连续补跑；PI 的 dt 仍使用实际测量值。
		}
	}
	// TODO[任务装配]：注册成功后通过 osThreadNew(Task, nullptr, &taskAttrs) 创建任务并检查返回值。
	// 对外声明放 chassis.h；是否接入 Robot.cpp 由底盘板配置决定，完成独立验证前不自动接入整车。




}
