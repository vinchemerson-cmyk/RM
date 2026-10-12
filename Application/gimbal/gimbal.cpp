/**
 * @file gimbal.cpp
 * @brief 用右摇杆控制云台 yaw/pitch 两轴 GM6020 的速度或位置目标。
 *
 * 控制管线 (GimbalTask, 周期约 1ms):
 *   1. 计算 dt: 调度抖动被钳位到 (0, kGimbalMaxDtS], 异常大的 dt 不进入
 *      LQI 积分，避免异常周期推动角度目标与积分。
 *   2. outputAllowed = rc::IsOnline() && safety::ShouldOutput(); 任一条件
 *      不满足则两轴 Disable、目标复位 (安全侧默认, 重新使能后重新同步)。
 *   3. UpdateAxis(): 每轴按当前模式改写 motor_settings:
 *        - MotorAngleTarget 积分遥控位置目标；LQI 用启动时求出的 K 计算电流 A。
 *        - pitch 端点仅接受向内退回，临时使用 K_speed 反馈，不保持位置。
 *   4. CURRENT_LOOP 绕过 MCU PID，驱动应用电机方向并将两轴合成一帧
 *      0x1FE 电流指令；GM6020 内部电流环必须开启。求解只发生在 Init。
 *
 * 线程模型:
 *   - 本任务是 motor_settings 的唯一写者, 无需加锁;
 *   - 电机反馈由 FDCAN 中断填充, 读取时用临界区做快照
 *     (FeedbackSnapshot), 避免读到"角度已更新而转速未更新"的混合值;
 *   - 指令帧在任务上下文发送, CAN 发送 FIFO 满时由 dji 驱动自然丢弃,
 *     下个周期重发。
 */

#include "gimbal.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "motor/dji/dji.h"
#include "rc/rc.h"
#include "safety/safety.h"
#include "log/bsp_log.h"
#include "algorithm/lqi_controller.h"

#include "GimbalConfig.hpp"

namespace
{
	osThreadId_t taskHandle = nullptr;

	DJI_Motor_s* yawMotor   = nullptr;
	DJI_Motor_s* pitchMotor = nullptr;

	/** @brief 两轴电机句柄数组, 交给 DJI 驱动逐周期打包下发。 */
	DJI_Motor_s* gimbalMotors[2] = {nullptr, nullptr};

	/** @brief 角度目标在失联或重新使能时会同步到当前反馈。 */
	MotorAngleTarget             yawTarget{};                //yaw的目标
	MotorAngleTarget             pitchTarget{};              //pitch的目标
	LQIController                yawLqi{};                   //yaw的lqi
	LQIController                pitchLqi{};                 //pitch的lqi
	Motor_Feedback_s             yawControlFeedback{};       //yaw的控制反馈
	uint32_t                     yawControlTick{};           //yaw的控制滴答
	float                        yawControlTargetSpeedDps{}; //位置模式下为摇杆给定的目标角度变化率，仅诊断。
	serialTasks::EncoderSnapshot yawControlSnapshot{};       //yaw编码器快照
	serialTasks::EncoderSnapshot pitchControlSnapshot{};	 //pitch编码器快照
	bool                         yawPulseArmed     = false;//yaw测试脉冲就绪标志
	bool                         yawPulseActive    = false;
	uint32_t                     yawPulseStartTick = 0U;
	float                        yawPulseDirection = 0.0F;
	gimbal::yawonetest    remoteCommandTest{};
	static_assert(!kYawRemoteCommandTestEnabled || !kYawFixedCurrentTestEnabled,
				  "Remote command test and fixed current test are mutually exclusive");

	constexpr osThreadAttr_t taskAttrs = {
		.name       = "GimbalTask",
		.stack_size = 2048U,
		.priority   = osPriorityHigh,
	};

	/**
 * @brief 构造一个云台电机的初始化配置。
 * @param motor_type 仅用于标记型号, 必须为 GM6020。
 * @param feedback_id 该电机的反馈 ID, 决定它在指令帧中的槽位。
 * @param reverse 输出方向是否取反。
 * @note LQI 直接写入电流指令，MCU 上的三层 PID 均不参与，电机内部电流环保持开启。
 */
	Motor_Init_Config_s MakeMotorConfig(const Motor_Type_e motor_type, const uint32_t feedback_id,
										const bool         reverse)
	{
		Motor_Init_Config_s config{};
		config.motor_type = motor_type;

		// 使用电机自身反馈，不使用外部反馈与前馈通道。
		config.controller_param_init_config.other_angle_feedback_ptr = nullptr; //其他角度反馈
		config.controller_param_init_config.other_speed_feedback_ptr = nullptr; //其他速度反馈
		config.controller_param_init_config.speed_feedforward_ptr    = nullptr; //速度前馈
		config.controller_param_init_config.current_feedforward_ptr  = nullptr; //电流前馈

		config.controller_setting_init_config.close_loop_type    = CURRENT_LOOP;
		config.controller_setting_init_config.outer_loop_type    = CURRENT_LOOP;
		config.controller_setting_init_config.motor_reverse_flag =
			reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
		config.controller_setting_init_config.feedback_reverse_flag =
			reverse ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL;
		config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
		config.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
		config.controller_setting_init_config.feedforward_flag      = FEEDFORWARD_NONE;

		config.can_init_config.can_handle = kGimbalCanHandle;
		config.can_init_config.rx_id      = feedback_id;
		return config;
	}

	/** @brief 在任务中读取一次完整反馈，避免中断更新期间读到混合值。 */
	Motor_Feedback_s FeedbackSnapshot(const DJI_Motor_s& motor)
	{
		taskENTER_CRITICAL();
		const Motor_Feedback_s feedback = motor.feedback;
		taskEXIT_CRITICAL();
		return feedback;
	}

	/** @brief 上电按各轴模式装载手填 K 或求解一次；非法配置时该轴保持禁用。 */
	void InitializeLqi(const char*              axis, const LQI_Model_s& model,
					   const LQI_Init_Config_s& source, LQIController&   controller,
					   GimbalLqiGainSource      gainSource)
	{
		controller.Disable();
		if (gainSource == GimbalLqiGainSource::Manual)
		{
			if (!controller.Configure(source))
			{
				LOGERROR("[gimbal] %s manual LQI K/limits invalid; axis disabled", axis);
				return;
			}
			LOGINFO("[gimbal] %s LQI ready, manual K", axis);
			return;
		}
		if (gainSource != GimbalLqiGainSource::BootSolve)
		{
			LOGERROR("[gimbal] %s LQI gain source invalid; axis disabled", axis);
			return;
		}
		LQI_Solve_Result_s result{};
		const auto         status = LQI_Solve(model, source, result, kGimbalLqiSolverOptions);
		if (status != LQI_Solve_Status::Success)
		{
			LOGERROR("[gimbal] %s LQI solve failed: status=%u, iterations=%u; axis disabled",
					 axis, static_cast<unsigned>(status), static_cast<unsigned>(result.iterations));
			return;
		}
		auto config       = source;
		config.k_angle    = result.gain[0];
		config.k_speed    = result.gain[1];
		config.k_integral = result.gain[2];
		if (!controller.Configure(config))
		{
			LOGERROR("[gimbal] %s LQI limits/config invalid; axis disabled", axis);
			return;
		}
		LOGINFO("[gimbal] %s LQI ready, iterations=%u", axis, static_cast<unsigned>(result.iterations));
	}

	/** @brief 停止单轴并清位置目标和 LQI 历史；保留已求出的 K，不重复求解。 */
	void StopAxis(DJI_Motor_s& motor, MotorAngleTarget& target, LQIController& lqi)
	{
		DJI_Motor_Disable(&motor);
		target.Reset();
		lqi.Reset();
	}

	/**
 * @brief 更新单轴目标；急停或遥控失联后发送零输出，测试模式不检查电机反馈超时。
 * @note 位置模式重新使能时先将目标同步到当前角度，避免重新追逐旧目标。
 * todo read
 */
	void UpdateAxis(DJI_Motor_s* motor, MotorAngleTarget& target, LQIController& lqi,
					bool         outputAllowed, const MotorAngleLimits& limits,
					float        stick, float targetRateDps, float dt_s, bool positionMode = true)
	{
		if (motor == nullptr)
			return;
		const Motor_Feedback_s feedback = FeedbackSnapshot(*motor);
		if (motor == yawMotor)
		{
			yawControlFeedback       = feedback;
			yawControlTick           = osKernelGetTickCount();
			yawControlTargetSpeedDps = 0.0F;
		}
		if (!outputAllowed || !lqi.IsConfigured() || !feedback.angle_initialized)
		{
			StopAxis(*motor, target, lqi);
			return;
		}

		if (!limits.continuous)
		{
			const bool reverse = motor->motor_settings.feedback_reverse_flag ==
								 FEEDBACK_DIRECTION_REVERSE;
			const auto raw           = static_cast<uint16_t>(feedback.raw_angle);
			const bool allowPositive = limits.AllowsSpeed(raw, reverse, 1.0F);
			const bool allowNegative = limits.AllowsSpeed(raw, reverse, -1.0F);
			if (!allowPositive && !allowNegative)
			{
				StopAxis(*motor, target, lqi);
				return;
			}
			if (!allowPositive || !allowNegative)
			{
				const float recoverySpeed = stick * targetRateDps;
				if (!limits.AllowsSpeed(raw, reverse, recoverySpeed))
				{
					StopAxis(*motor, target, lqi);
					return;
				}
				// 保护区撤销位置保持，仅用 LQI 的速度反馈增益执行手动向内退回。
				target.Reset();
				const float currentA                  = lqi.UpdateRecoverySpeed(recoverySpeed, feedback.speed, dt_s);
				motor->motor_settings.close_loop_type = CURRENT_LOOP;
				motor->motor_settings.current         = currentA * kDjiGm6020CommandUnitsPerAmp;
				if (!motor->enabled)
					DJI_Motor_Enable(motor);
				return;
			}
		}

		if (!positionMode)
		{
			// 速度测试不累积角度目标；松杆后仅低增益速度制动，不保持位置。
			target.Reset();
			const float speedTarget = std::isfinite(stick)
										  ? std::fmax(-1.0F, std::fmin(1.0F, stick)) * targetRateDps
										  : 0.0F;
			if (motor == yawMotor) yawControlTargetSpeedDps = speedTarget;
			motor->motor_settings.close_loop_type = CURRENT_LOOP;
			motor->motor_settings.speed           = speedTarget;
			motor->motor_settings.angle           = feedback.total_angle;
			motor->motor_settings.current         = lqi.UpdateRecoverySpeed(speedTarget, feedback.speed, dt_s) *
											kDjiGm6020CommandUnitsPerAmp;
			if (!motor->enabled)
				DJI_Motor_Enable(motor);
			return;
		}

		if (!target.Ready() || target.Epoch() != feedback.angle_epoch)
		{
			const bool reverse = motor->motor_settings.feedback_reverse_flag ==
								 FEEDBACK_DIRECTION_REVERSE;
			if (!target.Sync(limits, feedback.total_angle,
							 static_cast<uint16_t>(feedback.raw_angle), feedback.angle_epoch, reverse))
			{
				StopAxis(*motor, target, lqi);
				return;
			}
			lqi.Reset();
			if (motor->enabled)
				DJI_Motor_Disable(motor);
		}
		if (!target.Contains(limits, feedback.total_angle))
		{
			StopAxis(*motor, target, lqi);
			return;
		}

		motor->motor_settings.close_loop_type = CURRENT_LOOP;
		motor->motor_settings.angle           = target.Step(limits, stick, targetRateDps, dt_s);
		if (motor == yawMotor && std::isfinite(stick) && std::isfinite(targetRateDps) &&
			targetRateDps >= 0.0F && std::isfinite(dt_s) && dt_s > 0.0F)
			yawControlTargetSpeedDps = std::fmax(-1.0F, std::fmin(1.0F, stick)) * targetRateDps;
		motor->motor_settings.angle_epoch = feedback.angle_epoch;
		const float feedforwardA          = motor == yawMotor
									   ? kYawTargetSpeedFeedforwardAperDps * yawControlTargetSpeedDps
									   : 0.0F;
		const float currentA = lqi.Update(motor->motor_settings.angle, feedback.total_angle,
										  feedback.speed, dt_s, feedforwardA);
		motor->motor_settings.current = currentA * kDjiGm6020CommandUnitsPerAmp;
		if (!motor->enabled)
			DJI_Motor_Enable(motor);
	}

	/** @brief 固定电流单次测试；松杆/急停/超时撤销指令，不运行 MCU 速度环。
	 *  @param outputAllowed
	 *  @param stick 摇杆输入
	 *  @param now
	 *
	 */
	void UpdateYawFixedCurrentTest(bool outputAllowed, float stick, uint32_t now)
	{
		if (yawMotor == nullptr)
		{
			yawPulseActive = yawPulseArmed = false;
			return;
		}
		const auto feedback      = FeedbackSnapshot(*yawMotor);
		yawControlFeedback       = feedback;
		yawControlTick           = osKernelGetTickCount();
		yawControlTargetSpeedDps = 0.0F;
		if (!outputAllowed || !feedback.angle_initialized || !std::isfinite(stick))
		{
			yawPulseActive = yawPulseArmed = false;
			StopAxis(*yawMotor, yawTarget, yawLqi);
			return;
		}
		const float direction = std::fabs(stick) >= kYawFixedCurrentTestStickThreshold
									? (stick > 0.0F ? 1.0F : -1.0F)
									: 0.0F;

		if (direction == 0.0F)
		{
			yawPulseActive = false;
			yawPulseArmed  = true;
			StopAxis(*yawMotor, yawTarget, yawLqi);
			return;
		}

		if (yawPulseArmed)
		{
			yawPulseArmed     = false;
			yawPulseActive    = true;
			yawPulseDirection = direction;
			yawPulseStartTick = now;
		}

		if (!yawPulseActive ||                                         //当前是否正在进行yaw脉冲测试
			direction != yawPulseDirection ||                          //摇杆方向动了(意图改变)
			now - yawPulseStartTick >= kYawFixedCurrentTestDurationMs) //持续时间到了
		{
			yawPulseActive = false;
			StopAxis(*yawMotor, yawTarget, yawLqi);
			return;
		}

		yawTarget.Reset();
		yawLqi.Reset();
		yawMotor->motor_settings.close_loop_type = CURRENT_LOOP;
		yawMotor->motor_settings.speed           = 0.0F; // 无速度目标。
		yawMotor->motor_settings.angle           = feedback.total_angle;
		yawMotor->motor_settings.current         = yawPulseDirection * kYawFixedCurrentTestA *//电流设置
										   kDjiGm6020CommandUnitsPerAmp;

		if (!yawMotor->enabled)
			DJI_Motor_Enable(yawMotor);
	}

	/** @brief 使用本次计算反馈发布诊断；禁止在此重新读取最新电机反馈。 */
	void PublishYawControlSnapshot(float dt_s)
	{
		if (yawMotor == nullptr) return;
		serialTasks::EncoderSnapshot snapshot{};
		snapshot.control_tick     = yawControlTick;
		snapshot.control_sequence = yawControlSnapshot.control_sequence + 1U;
		snapshot.control_dt_s     = dt_s;
		snapshot.yaw              = {
			.raw  = static_cast<uint16_t>(yawControlFeedback.raw_angle),
			.seen = yawControlFeedback.angle_initialized, .last_update_tick = yawControlFeedback.last_update_tick
		};
		snapshot.yaw_position_actual_deg = yawControlFeedback.total_angle;
		snapshot.yaw_position_target_deg = yawTarget.Ready() ? yawTarget.Target() : yawControlFeedback.total_angle;
		snapshot.yaw_speed_dps           = yawControlFeedback.speed;
		snapshot.yaw_target_speed_dps    = yawMotor->enabled ? yawControlTargetSpeedDps : 0.0F;
		snapshot.yaw_feedback_current_a  = yawControlFeedback.current;
		snapshot.yaw_output_enabled      = yawMotor->enabled;
		snapshot.yaw_command_current_a   = yawMotor->enabled
											 ? yawMotor->motor_settings.current / kDjiGm6020CommandUnitsPerAmp
											 : 0.0F;
		snapshot.yaw_position_current_a = yawMotor->enabled ? yawLqi.PositionCurrent() : 0.0F;
		snapshot.yaw_speed_current_a    = yawMotor->enabled ? yawLqi.SpeedCurrent() : 0.0F;
		snapshot.yaw_tx_attempts        = yawMotor->tx_enqueue_attempts;
		snapshot.yaw_tx_failures        = yawMotor->tx_enqueue_failures;
		taskENTER_CRITICAL();
		yawControlSnapshot = snapshot;
		taskEXIT_CRITICAL();
	}

	/** @brief 云台控制任务：读取遥控器、更新两轴目标，并发送统一 CAN 指令帧。 */
	void Task(void*)
	{
		uint32_t lastTick = osKernelGetTickCount();

		for (;;)
		{
			const uint32_t now  = osKernelGetTickCount();
			float          dt_s = static_cast<float>(now - lastTick) / 1000.0F;
			lastTick            = now;

			// 调度被长时间占用时不使用异常 dt, 避免积分项和微分项出现尖峰。
			if (dt_s <= 0.0F)
				dt_s = kGimbalControlPeriodS;
			//time是否可信
			const bool timingValid = dt_s <= kGimbalMaxDtS;
			if (!timingValid)
				dt_s = kGimbalMaxDtS;

			const auto remote        = rc::Snapshot();
			const bool outputAllowed =timingValid &&
									   rc::IsOnline() && safety::ShouldOutput();
			float yawStick   = rc::Normalize(remote.channels[0]);//yaw摇杆值
			float pitchStick = rc::Normalize(remote.channels[1]);//pitch摇杆值
			if constexpr (kYawRemoteCommandTestEnabled)
			{
				const bool allowed = outputAllowed && kEnableYaw &&
									 yawMotor != nullptr && FeedbackSnapshot(*yawMotor).angle_initialized &&
									 remote.switch_left == rc::Left_Switch_e::MIDDLE;
				const bool requested = remote.switch_right == rc::Right_Switch_e::UP;
				yawStick             = remoteCommandTest.Update(now, allowed, requested,
													kYawRemoteCommandTestConfig);
				pitchStick = 0.0F;
			}
			if constexpr (kYawFixedCurrentTestEnabled)
				UpdateYawFixedCurrentTest(outputAllowed && kEnableYaw,
										  yawStick, now);
			else
				UpdateAxis(yawMotor, yawTarget, yawLqi, outputAllowed && kEnableYaw,
						   kYawAngleLimits, yawStick,
						   kYawPositionMode ? kYawTargetRateDps : kYawSpeedTestMaxDps,
						   dt_s, kYawPositionMode);
			UpdateAxis(pitchMotor, pitchTarget, pitchLqi, outputAllowed && kEnablePitch,
					   kPitchAngleLimits, pitchStick,
					   kPitchTargetRateDps, dt_s);

			DJI_Motor_Control(gimbalMotors, 2U, dt_s);
			PublishYawControlSnapshot(dt_s);

			const uint32_t next = now + 1U;
			if (static_cast<int32_t>(next - osKernelGetTickCount()) > 0)
				osDelayUntil(next);
			else
				osDelay(1U);
		}
	}
} // namespace

void gimbal::Update()
{
	// 控制逻辑集中在云台任务中执行; 保留此入口供后续由统一调度器调用。
}

serialTasks::EncoderSnapshot gimbal::GetEncoderSnapshot()
{
	serialTasks::EncoderSnapshot snapshot{};
	taskENTER_CRITICAL();
	if (pitchMotor != nullptr)
	{
		snapshot.pitch = {
			static_cast<uint16_t>(pitchMotor->feedback.raw_angle),
			pitchMotor->feedback.angle_initialized,
			pitchMotor->feedback.last_update_tick
		};
		snapshot.pitch_current_a = pitchMotor->feedback.current;
		if (pitchMotor->feedback.angle_initialized)
		{
			const bool reverse = pitchMotor->motor_settings.feedback_reverse_flag ==
								 FEEDBACK_DIRECTION_REVERSE;
			const float position_deg = static_cast<float>(MotorEncoderDelta(
																			kPitchZeroRaw,
																			static_cast<uint16_t>(pitchMotor->feedback.
																											  raw_angle)))
									   *
									   (360.0F / 8192.0F) * (reverse ? -1.0F : 1.0F);
			snapshot.pitch_position_actual_deg = position_deg;
			if (pitchMotor->enabled &&
				pitchTarget.Ready() &&
				pitchMotor->motor_settings.angle_epoch == pitchMotor->feedback.angle_epoch)
				snapshot.pitch_position_target_deg = pitchMotor->motor_settings.angle -
													 pitchMotor->feedback.total_angle + position_deg;
		}
	}
	if (yawMotor != nullptr)
	{
		snapshot.yaw = {
			.raw              = static_cast<uint16_t>(yawMotor->feedback.raw_angle),
			.seen             = yawMotor->feedback.angle_initialized,
			.last_update_tick = yawMotor->feedback.last_update_tick
		};
		snapshot.yaw_position_actual_deg = yawMotor->feedback.total_angle;
		snapshot.yaw_position_target_deg = yawTarget.Ready()
											   ? yawMotor->motor_settings.angle
											   : yawMotor->feedback.total_angle;
		snapshot.yaw_speed_dps          = yawMotor->feedback.speed;
		snapshot.yaw_feedback_current_a = yawMotor->feedback.current;
		snapshot.yaw_target_speed_dps   = yawMotor->enabled ? yawMotor->motor_settings.speed : 0.0F;
		snapshot.yaw_command_current_a  = yawMotor->enabled
											 ? yawMotor->motor_settings.current / kDjiGm6020CommandUnitsPerAmp
											 : 0.0F;
	}
	taskEXIT_CRITICAL();
	return snapshot;
}

serialTasks::EncoderSnapshot gimbal::GetYawControlSnapshot()
{
	taskENTER_CRITICAL();
	const auto snapshot = yawControlSnapshot;
	taskEXIT_CRITICAL();
	return snapshot;
}

serialTasks::EncoderSnapshot gimbal::GetPitchControlSnapshot()
{
	taskENTER_CRITICAL();
	const auto snapshot = pitchControlSnapshot;
	taskEXIT_CRITICAL();
	return snapshot;
}

void gimbal::Init()
{
	if (taskHandle != nullptr)
		return;

	// 反馈 ID 必须在 0x205..0x208 范围内, 对应 GM6020 的指令帧槽位。
	const Motor_Init_Config_s yawConfig   = MakeMotorConfig(GM6020, kYawMotorFeedbackId, kYawMotorReverse);
	const Motor_Init_Config_s pitchConfig = MakeMotorConfig(GM6020, kPitchMotorFeedbackId, kPitchMotorReverse);

	if constexpr (kEnableYaw || kReadYawEncoder)
		yawMotor = DJI_Motor_Init(yawConfig);
	if constexpr (kEnablePitch)
		pitchMotor = DJI_Motor_Init(pitchConfig);
	if ((yawMotor == nullptr) ||
		(kEnablePitch && pitchMotor == nullptr))
		Error_Handler(); // 只校验当前启用的轴。

	gimbalMotors[0] = yawMotor;
	gimbalMotors[1] = pitchMotor;
	DJI_Motor_Disable(yawMotor);
	DJI_Motor_Disable(pitchMotor);

	if constexpr (kEnableYaw)
		InitializeLqi("yaw", kYawLqiModel, kYawLqiConfig, yawLqi, kYawLqiGainSource);
	if constexpr (kEnablePitch)
		InitializeLqi("pitch", kPitchLqiModel, kPitchLqiConfig, pitchLqi, kPitchLqiGainSource);

	taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
	if (taskHandle == nullptr)
		Error_Handler();
}
