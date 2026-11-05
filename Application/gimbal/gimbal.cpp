/**
 * @file gimbal.cpp
 * @brief 用右摇杆控制云台 yaw/pitch 两轴 DM 电机的 MIT 转矩目标。
 *
 * 控制管线 (GimbalTask, 周期约 1ms):
 *   1. 计算 dt 并检查遥控、安全和反馈时效；异常时停止输出并复位目标。
 *   2. MotorAngleTarget 在度坐标中积分位置目标，LQI 计算转矩指令 (N·m)。
 *   3. DMMotorSetMITTorque() 设置目标，DMMotorTask() 按 DM 协议定时发送。
 *   4. MIT 的 Kp/Kd 均为零，位置/速度闭环由应用层 LQI 完成。
 *
 * 线程模型:
 *   - 本任务是 MIT 目标引用的唯一写者;
 *   - DM 反馈由 FDCAN 中断更新，读取时用临界区复制后转换单位;
 *   - 驱动负责检查反馈超时和电机故障，故障后必须显式重新使能。
 */

#include "gimbal.h"

#include <algorithm>
#include <cmath>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "algorithm/lqi_controller.h"
#include "log/bsp_log.h"
#include "rc/rc.h"
#include "safety/safety.h"

#include "GimbalConfig.hpp"
#include "RobotCommand.hpp"
#include "RobotConfig.hpp"
#include "chassis/ChassisConfig.h"
#include "communication/interboard.h"
#include "message_center/message_center.h"
#include "motor/dm/dmmotor.h"

namespace
{
	osThreadId_t taskHandle = nullptr;

	Publisher_t* chassisCommandPublisher = nullptr;
	Publisher_t* chassisSafetyPublisher  = nullptr;
	uint16_t     chassisCommandSequence  = 0U;
	uint16_t     chassisSafetySequence   = 0U;
	uint32_t     lastChassisPublishTick  = 0U;

	/** @brief 与底盘原有摇杆映射一致的连续死区。 */
	float CommandDeadband(float stick)
	{
		if (std::fabs(stick) <= kChassisStickDeadband) return 0.0F;
		return std::copysign((std::fabs(stick) - kChassisStickDeadband) /
							 (1.0F - kChassisStickDeadband), stick);
	}

	/** @brief 物理量转 16 位定点，饱和而不是回绕。 */
	std::int16_t QuantizeCommand(float value, float scale)
	{
		const long scaled = std::lround(static_cast<double>(value) * scale);
		return static_cast<std::int16_t>(std::clamp(scaled, -32768L, 32767L));
	}

	/**
	 * @brief 底盘模式触发占位。
	 * @note 当前只有 NoFollow/Disabled 生效；FollowGimbal/Spin/Direct 的触发源
	 *       尚未定义，确定拨杆、上位机或状态机后再在这里接入。
	 */
	chassis::Mode SelectChassisMode(const rc::RC_Data_s& remote, bool outputEnabled)
	{
		if (!outputEnabled) return chassis::Mode::Disabled;

		// TODO: 在这里实现模式触发，例如：
		// if (remote.switch_left == rc::Left_Switch_e::UP) return chassis::Mode::Spin;
		// if (跟随云台条件) return chassis::Mode::FollowGimbal;
		// if (直接速度入口) return chassis::Mode::Direct;
		(void)remote;
		return chassis::Mode::NoFollow;
	}

	DMMotorInstance* yawMotor          = nullptr;
	DMMotorInstance* aimpitchMotor     = nullptr;
	DMMotorInstance* foldingpitchMotor = nullptr;

	//单轴反馈数据结构体
	struct AxisFeedback
	{
		uint16_t raw_angle{};
		float    total_angle_deg{};
		float    speed_dps{};
		float    torque_nm{};
		uint32_t angle_epoch{};
		uint32_t last_update_tick{};
		bool     angle_initialized{};
	};

	/** @brief 角度目标在失联或重新使能时会同步到当前反馈。 */
	MotorAngleTarget          yawTarget{};                    //yaw的目标
	MotorAngleTarget          aim_pitchTarget{};              //aimpitch的目标
	MotorAngleTarget          folding_pitchTarget{};          //foldingpitch的目标
	LQIController             yawLqi{};                       //yaw的lqi
	LQIController             aim_pitchLqi{};                 //aimpitch的lqi
	LQIController             folding_pitchLqi{};             //foldingpitch的lqi
	AxisFeedback              yawControlFeedback{};           //yaw的控制反馈
	AxisFeedback              aimControlFeedback{};				//
	AxisFeedback              foldingControlFeedback{};
	float                     yawControlTargetSpeedDps{};     //位置模式下为摇杆给定的目标角度变化率，仅诊断。
	float                     aimControlTargetSpeedDps{};
	float						foldingControlTargetSpeedDps{};
	serialTasks::AxisSnapshot yawControlSnapshot{};           //yaw控制快照
	serialTasks::AxisSnapshot aim_pitchControlSnapshot{};     //aimpitch控制快照
	serialTasks::AxisSnapshot folding_pitchControlSnapshot{}; //foldingpitch控制快照
	bool                      yawPulseArmed     = false;      //yaw测试脉冲就绪标志
	bool                      yawPulseActive    = false;
	uint32_t                  yawPulseStartTick = 0U;
	float                     yawPulseDirection = 0.0F;
	gimbal::yawonetest        remoteCommandTest{};
	static_assert(!kYawRemoteCommandTestEnabled || !kYawFixedTorqueTestEnabled,
				  "Remote command test and fixed torque test are mutually exclusive");

	constexpr osThreadAttr_t taskAttrs = {
		.name       = "GimbalTask",
		.stack_size = 2048U,
		.priority   = osPriorityHigh,
	};

	/**
	 * @brief 构造一个 DM 电机的纯转矩 MIT 初始化配置。
	 * @param motor_type
	 * @param master_id MIT 命令 ID，同时也用于核对反馈帧中的电机 ID。
	 * @param feedback_id 电机反馈帧的 CAN ID。
	 * @param reverse
	 * @note protocol_limits 必须与达妙调试助手中的实际量程一致，由 LimitsFor() 按型号给出。
	 * @note 纯转矩模式下 Kp=Kd=0，电机只用 T_ff；但量程仍必须正确，否则力矩指令按比例失真。
	 */
	MIT_Init_Config_s MakeMotorConfig(DMMotor_Type_e motor_type, uint32_t master_id,
									  uint32_t       feedback_id, bool    reverse)
	{
		MIT_Init_Config_s config{};
		config.Motor_Type                      = motor_type;
		config.can_init_config.can_handle      = kGimbalCanHandle;
		config.can_init_config.master_id       = master_id;
		config.can_init_config.can_id          = feedback_id;
		config.protocol_limits                 = LimitsFor(motor_type);
		config.controller_param_init_config.Kp = 0.0F;
		config.controller_param_init_config.Kd = 0.0F;
		config.motor_reverse_flag              = reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
		return config;
	}

	constexpr float kRadToDeg = 57.29577951308232F;

	/** @brief 将 DM 单圈位置映射为现有机械限位算法使用的 13 位计数。 */
	uint16_t RawAngleFromPosition(float position_rad)
	{
		constexpr float kTwoPi = 6.283185307179586F;
		float           turns  = position_rad / kTwoPi;
		turns                  -= std::floor(turns);
		return static_cast<uint16_t>(turns * 8192.0F) & 0x1FFFU;
	}

	/** @brief 复制 DM 反馈，再转换为 LQI 使用的度、度/秒和 N·m。 */
	AxisFeedback FeedbackSnapshot(const DMMotorInstance& motor)
	{
		//关中断,保障操作原子性
		taskENTER_CRITICAL();
		const DM_Motor_Measure_s measure      = motor.measure;
		const int                rx_init_flag = motor.rx_init_flag;
		taskEXIT_CRITICAL();

		AxisFeedback feedback{};
		// raw_angle 还原到电机物理方向；连续角和速度已在 DM 驱动中完成方向处理。
		const float  physical_position = motor.motor_reverse_flag == MOTOR_DIRECTION_REVERSE
											 ? -measure.position
											 : measure.position;
		feedback.raw_angle         = RawAngleFromPosition(physical_position);
		feedback.total_angle_deg   = measure.total_position * kRadToDeg;
		feedback.speed_dps         = measure.velocity * kRadToDeg;
		feedback.torque_nm         = measure.torque;
		feedback.angle_epoch       = static_cast<uint32_t>(rx_init_flag);
		feedback.last_update_tick  = measure.last_update_tick;
		feedback.angle_initialized = rx_init_flag != 0;
		return feedback;
	}

	bool OutputRequested(const DMMotorInstance& motor)
	{
		return motor.stop_flag == MOTOR_ENABLE;
	}

	void EnableAxis(DMMotorInstance* motor)
	{
		if (!OutputRequested(*motor))
			DMMotorEnable(motor);
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

	/** @brief 软件停止单轴并清位置目标和 LQI 历史；保留已求出的 K。 */
	void StopAxis(DMMotorInstance& motor, MotorAngleTarget& target, LQIController& lqi)
	{
		DMMotorStop(&motor);
		target.Reset();
		lqi.Reset();
	}

	/**
	* @brief 更新单轴目标；急停、遥控失联或反馈超时后停止输出并清除目标。
	* @note 位置模式重新使能时先将目标同步到当前角度，避免重新追逐旧目标。
	* @param motor 电机实例
	* todo read
	* @param target
	* @param lqi
	* @param outputAllowed
	* @param limits
	* @param stick
	* @param targetRateDps
	* @param dt_s
	* @param positionMode
	*/
	void UpdateAxis(DMMotorInstance* motor, MotorAngleTarget&               target, LQIController& lqi,
					bool             outputAllowed, const MotorAngleLimits& limits, float          stick,
					float            targetRateDps, float                   dt_s, bool             positionMode = true)
	{
		if (motor == nullptr)
			return;
		//保存当前电机反馈
		const AxisFeedback feedback = FeedbackSnapshot(*motor);
		if (motor == yawMotor)
		{
			yawControlFeedback       = feedback;
			yawControlTargetSpeedDps = 0.0F;
		}

		const bool feedbackFresh = feedback.angle_initialized &&
								   osKernelGetTickCount() - feedback.last_update_tick <=
								   motor->feedback_timeout_ms;
		if (!outputAllowed || !lqi.IsConfigured())
		{
			StopAxis(*motor, target, lqi);
			return;
		}

		const uint32_t now = osKernelGetTickCount(); // 只读一次
		// 反馈不可信：撤转矩、保留目标与积分；长时间失联才重新锚定
		if (!feedbackFresh)
		{
			if (OutputRequested(*motor))
				DMMotorStop(motor);                    // 避免每 1ms 重复发零帧
			if (now - feedback.last_update_tick > 500) // 如 500ms
			{
				target.Reset();
				lqi.Reset();
			}
			return;
		}

		//防御代码
		if (!limits.continuous)
		{
			const bool reverse       = motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE;
			const bool allowPositive = limits.AllowsSpeed(feedback.raw_angle, reverse, 1.0F);
			const bool allowNegative = limits.AllowsSpeed(feedback.raw_angle, reverse, -1.0F);
			if (!allowPositive && !allowNegative)
			{
				StopAxis(*motor, target, lqi);
				return;
			}
			if (!allowPositive || !allowNegative)
			{
				const float recoverySpeed = stick * targetRateDps;
				if (!limits.AllowsSpeed(feedback.raw_angle, reverse, recoverySpeed))
				{
					StopAxis(*motor, target, lqi);
					return;
				}
				// 保护区撤销位置保持，仅用速度反馈增益执行手动向内退回。
				target.Reset();
				const float torqueNm = lqi.UpdateRecoverySpeed(recoverySpeed, feedback.speed_dps, dt_s);
				DMMotorSetMITTorque(motor, torqueNm);
				EnableAxis(motor);
				return;
			}
		}

		//防御代码
		if (!positionMode)
		{
			// 速度测试不累积角度目标；松杆后仅低增益速度制动，不保持位置。
			target.Reset();
			const float speedTarget = std::isfinite(stick)
										  ? std::fmax(-1.0F, std::fmin(1.0F, stick)) * targetRateDps
										  : 0.0F;
			if (motor == yawMotor) yawControlTargetSpeedDps = speedTarget;
			const float torqueNm = lqi.UpdateRecoverySpeed(speedTarget, feedback.speed_dps, dt_s);
			DMMotorSetMITTorque(motor, torqueNm);
			EnableAxis(motor);
			return;
		}

		//防御代码
		if (!target.Ready() || target.Epoch() != feedback.angle_epoch)
		{
			const bool reverse = motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE;
			if (!target.Sync(limits, feedback.total_angle_deg, feedback.raw_angle,
							 feedback.angle_epoch, reverse))
			{
				StopAxis(*motor, target, lqi);
				return;
			}
			lqi.Reset();
			if (OutputRequested(*motor)) DMMotorStop(motor);
		}

		//防御代码
		if (!target.Contains(limits, feedback.total_angle_deg))
		{
			StopAxis(*motor, target, lqi);
			return;
		}

		const float targetAngleDeg = target.Step(limits, stick, targetRateDps, dt_s);
		if (motor == yawMotor && std::isfinite(stick) && std::isfinite(targetRateDps) &&
			targetRateDps >= 0.0F && std::isfinite(dt_s) && dt_s > 0.0F)
			yawControlTargetSpeedDps = std::fmax(-1.0F, std::fmin(1.0F, stick)) * targetRateDps;
		const float feedforwardNm = motor == yawMotor
										? kYawTargetSpeedFeedforwardNmPerDps * yawControlTargetSpeedDps
										: 0.0F;
		const float torqueNm = lqi.Update(targetAngleDeg, feedback.total_angle_deg,
										  feedback.speed_dps, dt_s, feedforwardNm);
		DMMotorSetMITTorque(motor, torqueNm); //设置扭矩
		EnableAxis(motor);
	}

	/** @brief 固定转矩单次测试；松杆/急停/超时撤销指令，不运行速度环。 */
	void UpdateYawFixedTorqueTest(bool outputAllowed, float stick, uint32_t now)
	{
		if (yawMotor == nullptr)
		{
			yawPulseActive = yawPulseArmed = false;
			return;
		}
		const auto feedback      = FeedbackSnapshot(*yawMotor);
		yawControlFeedback       = feedback;
		yawControlTargetSpeedDps = 0.0F;
		const bool feedbackFresh = feedback.angle_initialized &&
								   now - feedback.last_update_tick <= yawMotor->feedback_timeout_ms;
		if (!outputAllowed || !feedbackFresh || !std::isfinite(stick))
		{
			yawPulseActive = yawPulseArmed = false;
			StopAxis(*yawMotor, yawTarget, yawLqi);
			return;
		}
		const float direction = std::fabs(stick) >= kYawFixedTorqueTestStickThreshold
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

		if (!yawPulseActive ||                                        //当前是否正在进行yaw脉冲测试
			direction != yawPulseDirection ||                         //摇杆方向动了(意图改变)
			now - yawPulseStartTick >= kYawFixedTorqueTestDurationMs) //持续时间到了
		{
			yawPulseActive = false;
			StopAxis(*yawMotor, yawTarget, yawLqi);
			return;
		}

		yawTarget.Reset();
		yawLqi.Reset();
		DMMotorSetMITTorque(yawMotor, yawPulseDirection * kYawFixedTorqueTestNm);
		EnableAxis(yawMotor);
	}

	/** @brief 使用本次计算反馈发布诊断；禁止在此重新读取最新电机反馈。 */
	void PublishYawControlSnapshot()
	{
		if (yawMotor == nullptr)
			return;
		serialTasks::AxisSnapshot snapshot{};
		snapshot.encoder = {
			.raw              = yawControlFeedback.raw_angle,
			.seen             = yawControlFeedback.angle_initialized,
			.last_update_tick = yawControlFeedback.last_update_tick
		};
		const bool outputRequested   = OutputRequested(*yawMotor);
		snapshot.position_actual_deg = yawControlFeedback.total_angle_deg;
		snapshot.position_target_deg = yawTarget.Ready()
										   ? yawTarget.Target()
										   : yawControlFeedback.total_angle_deg;
		snapshot.speed_actual_dps        = yawControlFeedback.speed_dps;
		snapshot.speed_target_dps        = outputRequested ? yawControlTargetSpeedDps : 0.0F;
		snapshot.feedback_torque_nm      = yawControlFeedback.torque_nm;
		snapshot.output_enabled          = outputRequested;
		snapshot.command_torque_nm       = outputRequested ? yawMotor->MIT_controller.ref_torque : 0.0F;
		snapshot.position_loop_torque_nm = outputRequested ? yawLqi.PositionCurrent() : 0.0F;
		snapshot.speed_loop_torque_nm    = outputRequested ? yawLqi.SpeedCurrent() : 0.0F;
		snapshot.tx_attempts             = yawMotor->tx_enqueue_attempts;
		snapshot.tx_failures             = yawMotor->tx_enqueue_failures;
		taskENTER_CRITICAL();
		yawControlSnapshot = snapshot;
		taskEXIT_CRITICAL();
	}

	void PublishAimPitchControlSnapshot()
	{
		if (aimpitchMotor == nullptr)
			return;
		serialTasks::AxisSnapshot snapshot{};
		snapshot.encoder = {
		.raw              = aimControlFeedback.raw_angle,
		.seen             = aimControlFeedback.angle_initialized,
		.last_update_tick = aimControlFeedback.last_update_tick
		};
		const bool outputRequested   = OutputRequested(*aimpitchMotor);
		snapshot.position_actual_deg = aimControlFeedback.total_angle_deg;
		snapshot.position_target_deg = aim_pitchTarget.Ready()
										   ? aim_pitchTarget.Target()
										   : aimControlFeedback.total_angle_deg;
		snapshot.speed_actual_dps        = aimControlFeedback.speed_dps;
		snapshot.speed_target_dps        = outputRequested ? aimControlTargetSpeedDps : 0.0F;
		snapshot.feedback_torque_nm      = aimControlFeedback.torque_nm;
		snapshot.output_enabled          = outputRequested;
		snapshot.command_torque_nm       = outputRequested ? aimpitchMotor->MIT_controller.ref_torque : 0.0F;
		snapshot.position_loop_torque_nm = outputRequested ? aim_pitchLqi.PositionCurrent() : 0.0F;
		snapshot.speed_loop_torque_nm    = outputRequested ? aim_pitchLqi.SpeedCurrent() : 0.0F;
		snapshot.tx_attempts             = aimpitchMotor->tx_enqueue_attempts;
		snapshot.tx_failures             = aimpitchMotor->tx_enqueue_failures;
		taskENTER_CRITICAL();
		yawControlSnapshot = snapshot;
		taskEXIT_CRITICAL();
	}
	//todo:publishfoldingpitchcontrolsnapshot()
	/** @brief 云台控制任务：读取遥控器、更新两轴目标，并推进 DM MIT 发送。 */
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

			//获取遥控快照
			const auto remote        = rc::Snapshot();
			const bool outputAllowed = timingValid &&
									   rc::IsOnline() && safety::ShouldOutput();

			if constexpr (InfantryInitialization::ENABLE_INTERBOARD)
			{
				if ((chassisCommandPublisher != nullptr || chassisSafetyPublisher != nullptr) &&
					now - lastChassisPublishTick >= 10U)
				{
					lastChassisPublishTick            = now;
					constexpr bool kChassisOutputMode =
						!kYawRemoteCommandTestEnabled && !kYawFixedTorqueTestEnabled;
					const bool rcOnline      = rc::IsOnline();
					const bool eStopped      = safety::IsEStopped();
					const bool bothDown      = safety::BothDown();
					const bool outputEnabled = kChassisOutputMode && safety::ShouldOutput();
					const bool runRequested  = kChassisOutputMode &&
											   remote.switch_left == kChassisRunSwitch;

					float vx = 0.0F, vy = 0.0F, yaw = 0.0F;
					if (outputEnabled)
					{
						vx = CommandDeadband(rc::Normalize(remote.channels[kChassisForwardChannel])) *
							 kChassisForwardSign * kChassisMotionLimits.max_translation_mps;
						vy = CommandDeadband(rc::Normalize(remote.channels[kChassisLeftChannel])) *
							 kChassisLeftSign * kChassisMotionLimits.max_translation_mps;
						yaw = CommandDeadband(rc::Normalize(remote.channels[kChassisYawChannel])) *
							  kChassisYawSign * kChassisMotionLimits.max_yaw_dps;
					}

					if (chassisCommandPublisher != nullptr)
					{
						chassis::Command command{};
						command.com_vx_mmps  = QuantizeCommand(vx, 1000.0F);
						command.com_vy_mmps  = QuantizeCommand(vy, 1000.0F);
						command.com_yaw_ddps = QuantizeCommand(yaw, 10.0F);
						command.mode         = static_cast<std::uint8_t>(
							SelectChassisMode(remote, outputEnabled));
						command.sequence = ++chassisCommandSequence;
						(void)PubPushMessage(chassisCommandPublisher, &command);
					}
					if (chassisSafetyPublisher != nullptr)
					{
						chassis::SafetyState state{};
						if (outputEnabled) state.flags |= chassis::kSafetyOutputEnabled;
						if (rcOnline) state.flags |= chassis::kSafetyRcOnline;
						if (eStopped) state.flags |= chassis::kSafetyEStopped;
						if (bothDown) state.flags |= chassis::kSafetyBothDown;
						if (runRequested) state.flags |= chassis::kSafetyRunRequested;
						state.sequence = ++chassisSafetySequence;
						(void)PubPushMessage(chassisSafetyPublisher, &state);
					}
				}
			}

			//对命令归一化
			float yawStick   = rc::Normalize(remote.channels[0]); //yaw摇杆值
			float pitchStick = rc::Normalize(remote.channels[1]); //pitch摇杆值

			//YAW的调试模式
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
			if constexpr (kYawFixedTorqueTestEnabled)
				UpdateYawFixedTorqueTest(outputAllowed && kEnableYaw, yawStick, now);
			else //更新三轴目标
				UpdateAxis(yawMotor, yawTarget, yawLqi, outputAllowed && kEnableYaw,
						   kYawAngleLimits, yawStick,
						   kYawPositionMode ? kYawTargetRateDps : kYawSpeedTestMaxDps,
						   dt_s, kYawPositionMode);
			UpdateAxis(aimpitchMotor, aim_pitchTarget, aim_pitchLqi, outputAllowed && kEnableAimPitch,
					   kAimPitchAngleLimits, pitchStick, kAimPitchTargetRateDps, dt_s);
			UpdateAxis(foldingpitchMotor, folding_pitchTarget, folding_pitchLqi, outputAllowed && kEnableFoldingPitch,
					   kFoldingPitchAngleLimits, pitchStick, kFoldingPitchTargetRateDps, dt_s);

			DMMotorTask();
			PublishYawControlSnapshot(); //todo:read
			PublishAimPitchControlSnapshot();

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

serialTasks::GimbalControlSnapshot gimbal::GetEncoderSnapshot()
{
	serialTasks::GimbalControlSnapshot snapshot{};
	taskENTER_CRITICAL();
	snapshot.control_tick = osKernelGetTickCount();

	if (aimpitchMotor != nullptr)
	{
		auto&              axis     = snapshot.aim_pitch;
		const AxisFeedback feedback = FeedbackSnapshot(*aimpitchMotor);
		const bool         enabled  = OutputRequested(*aimpitchMotor);
		axis.encoder                = {
			.raw              = feedback.raw_angle,
			.seen             = feedback.angle_initialized,
			.last_update_tick = feedback.last_update_tick
		};
		axis.feedback_torque_nm      = feedback.torque_nm;
		axis.speed_actual_dps        = feedback.speed_dps;
		axis.output_enabled          = enabled;
		axis.command_torque_nm       = enabled ? aimpitchMotor->MIT_controller.ref_torque : 0.0F;
		axis.position_loop_torque_nm = enabled ? aim_pitchLqi.PositionCurrent() : 0.0F;
		axis.speed_loop_torque_nm    = enabled ? aim_pitchLqi.SpeedCurrent() : 0.0F;
		axis.tx_attempts             = aimpitchMotor->tx_enqueue_attempts;
		axis.tx_failures             = aimpitchMotor->tx_enqueue_failures;
		if (feedback.angle_initialized)
		{
			const bool  reverse     = aimpitchMotor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE;
			const float positionDeg = static_cast<float>(MotorEncoderDelta(kAimPitchZeroRaw,
																		   feedback.raw_angle)) *
									  (360.0F / 8192.0F) * (reverse ? -1.0F : 1.0F);
			axis.position_actual_deg = positionDeg;
			if (enabled && aim_pitchTarget.Ready() &&
				aim_pitchTarget.Epoch() == feedback.angle_epoch)
				axis.position_target_deg = aim_pitchTarget.Target() -
										   feedback.total_angle_deg + positionDeg;
		}
	}

	if (foldingpitchMotor != nullptr)
	{
		auto&              axis     = snapshot.folding_pitch;
		const AxisFeedback feedback = FeedbackSnapshot(*foldingpitchMotor);
		const bool         enabled  = OutputRequested(*foldingpitchMotor);
		axis.encoder                = {
			.raw              = feedback.raw_angle,
			.seen             = feedback.angle_initialized,
			.last_update_tick = feedback.last_update_tick
		};
		axis.feedback_torque_nm      = feedback.torque_nm;
		axis.speed_actual_dps        = feedback.speed_dps;
		axis.output_enabled          = enabled;
		axis.command_torque_nm       = enabled ? foldingpitchMotor->MIT_controller.ref_torque : 0.0F;
		axis.position_loop_torque_nm = enabled ? folding_pitchLqi.PositionCurrent() : 0.0F;
		axis.speed_loop_torque_nm    = enabled ? folding_pitchLqi.SpeedCurrent() : 0.0F;
		axis.tx_attempts             = foldingpitchMotor->tx_enqueue_attempts;
		axis.tx_failures             = foldingpitchMotor->tx_enqueue_failures;
		if (feedback.angle_initialized)
		{
			const bool  reverse     = foldingpitchMotor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE;
			const float positionDeg = static_cast<float>(MotorEncoderDelta(kFoldingPitchZeroRaw,
																		   feedback.raw_angle)) *
									  (360.0F / 8192.0F) * (reverse ? -1.0F : 1.0F);
			axis.position_actual_deg = positionDeg;
			if (enabled && folding_pitchTarget.Ready() &&
				folding_pitchTarget.Epoch() == feedback.angle_epoch)
				axis.position_target_deg = folding_pitchTarget.Target() -
										   feedback.total_angle_deg + positionDeg;
		}
	}

	if (yawMotor != nullptr)
	{
		auto&              axis     = snapshot.yaw;
		const AxisFeedback feedback = FeedbackSnapshot(*yawMotor);
		const bool         enabled  = OutputRequested(*yawMotor);
		axis.encoder                = {
			.raw              = feedback.raw_angle,
			.seen             = feedback.angle_initialized,
			.last_update_tick = feedback.last_update_tick
		};
		axis.position_actual_deg     = feedback.total_angle_deg;
		axis.position_target_deg     = yawTarget.Ready() ? yawTarget.Target() : feedback.total_angle_deg;
		axis.speed_actual_dps        = feedback.speed_dps;
		axis.speed_target_dps        = enabled ? yawControlTargetSpeedDps : 0.0F;
		axis.feedback_torque_nm      = feedback.torque_nm;
		axis.output_enabled          = enabled;
		axis.command_torque_nm       = enabled ? yawMotor->MIT_controller.ref_torque : 0.0F;
		axis.position_loop_torque_nm = enabled ? yawLqi.PositionCurrent() : 0.0F;
		axis.speed_loop_torque_nm    = enabled ? yawLqi.SpeedCurrent() : 0.0F;
		axis.tx_attempts             = yawMotor->tx_enqueue_attempts;
		axis.tx_failures             = yawMotor->tx_enqueue_failures;
	}
	taskEXIT_CRITICAL();
	return snapshot;
}

serialTasks::AxisSnapshot gimbal::GetYawControlSnapshot() //若要打印电机数据,Robot.cpp调用这个函数
{
	taskENTER_CRITICAL();
	const auto snapshot = yawControlSnapshot;
	taskEXIT_CRITICAL();
	return snapshot;
}

serialTasks::AxisSnapshot gimbal::GetAimPitchControlSnapshot() //或者这个
{
	taskENTER_CRITICAL();
	const auto aimpitchsnapshot = aim_pitchControlSnapshot;
	taskEXIT_CRITICAL();
	return aimpitchsnapshot;
}

serialTasks::AxisSnapshot gimbal::GetFoldingPitchMotorFeedbackSnapshot() //或者这个
{
	taskENTER_CRITICAL();
	const auto foldingpitchsnapshot = folding_pitchControlSnapshot;
	taskEXIT_CRITICAL();
	return foldingpitchsnapshot;
}

void gimbal::Init()
{
	if (taskHandle != nullptr)
		return;

	const MIT_Init_Config_s yawConfig = MakeMotorConfig(
														dm4310, kYawMotorMasterId, kYawMotorFeedbackId,
														kYawMotorReverse);
	const MIT_Init_Config_s aimpitchConfig = MakeMotorConfig(
															 dm4310, kAimPitchMotorMasterId, kAimPitchMotorFeedbackId,
															 kAimPitchMotorReverse);
	const MIT_Init_Config_s foldingpitchConfig = MakeMotorConfig(
																 dm4340, kFoldingPitchMotorMasterId,
																 kFoldingPitchMotorFeedbackId,
																 kFoldingPitchMotorReverse
																);
	if constexpr (kEnableYaw || kReadYawEncoder)
		yawMotor = MITMotorInit(&yawConfig);
	if constexpr (kEnableAimPitch)
		aimpitchMotor = MITMotorInit(&aimpitchConfig);
	if constexpr (kEnableFoldingPitch)
		foldingpitchMotor = MITMotorInit(&foldingpitchConfig);
	if (((kEnableYaw || kReadYawEncoder) && yawMotor == nullptr) ||
		(kEnableAimPitch && aimpitchMotor == nullptr)|| (kEnableFoldingPitch && foldingpitchMotor == nullptr))
		Error_Handler();

	if constexpr (kEnableYaw)
		InitializeLqi("yaw", kYawLqiModel, kYawLqiConfig, yawLqi, kYawLqiGainSource);
	if constexpr (kEnableAimPitch)
		InitializeLqi("aimpitch", kAimPitchLqiModel, kAimPitchLqiConfig, aim_pitchLqi, kAimPitchLqiGainSource);
	if constexpr (kEnableFoldingPitch)
		InitializeLqi("foldingpitch", kFoldingPitchLqiModel,kFoldingPitchLqiConfig,folding_pitchLqi, kFoldingPitchLqiGainSource);

	if constexpr (InfantryInitialization::ENABLE_INTERBOARD)
	{
		const bool commandTopic = interboard::RegisterTxTopic(
															  chassis::kCommandTopicId, chassis::kCommandTopicName,
															  sizeof(chassis::Command));
		if (commandTopic)
			chassisCommandPublisher = PubRegister(chassis::kCommandTopicName, sizeof(chassis::Command));
		else
			LOGERROR("[gimbal] chassis command TX topic registration failed");

		const bool safetyTopic = interboard::RegisterTxTopic(
															 chassis::kSafetyTopicId, chassis::kSafetyTopicName,
															 sizeof(chassis::SafetyState));
		if (safetyTopic)
			chassisSafetyPublisher = PubRegister(chassis::kSafetyTopicName, sizeof(chassis::SafetyState));
		else
			LOGERROR("[gimbal] chassis safety TX topic registration failed");

		if (chassisCommandPublisher == nullptr || chassisSafetyPublisher == nullptr)
			LOGERROR("[gimbal] chassis command/safety publisher registration incomplete");
	}

	taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
	if (taskHandle == nullptr)
		Error_Handler();
}
