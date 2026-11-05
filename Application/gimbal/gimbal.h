/**
 * @file gimbal.h
 * @brief 云台 yaw/pitch 两轴控制入口。
 *
 * 数据流 (每个控制周期, 约 1kHz):
 *   DT7 遥控器 -> UART5 中断解析 (rc 模块)
 *     -> GimbalTask 读取右摇杆归一化值 ([-1, 1])
 *     -> 按位置/速度模式计算两轴目标，LQI 输出转矩 (N·m)
 *     -> DMMotorSetMITTorque() 设置目标，DMMotorTask() 发送 MIT 帧。
 *
 * @note 输入: 遥控器右摇杆。右摇杆水平 -> yaw, 右摇杆垂直 -> pitch。
 * @note 两轴使用 LQI 位置控制，摇杆推动角度目标；当前 Manual 模式使用手填 K。
 *       原 GM6020 增益数值迁入 DM 后必须重新整定。
 * @note 安全: 遥控失联、急停或反馈超时时停止该轴并清除 MIT 目标。
 *       当前pitch/yaw均为位置模式；pitch进入端点保护区时仅接受手动向零位退回。
 * @note 依赖: 必须先完成 rc::Init() 与 safety::Init(), 本模块每个周期都会
 *       查询两者状态; 二者未初始化时查询结果按"离线/禁止输出"处理。
 */

#ifndef INFANTRY_H723_GIMBAL_H
#define INFANTRY_H723_GIMBAL_H

#include "communication/encoder_telemetry.h"

namespace gimbal
{
	/** @brief 初始化 yaw/pitch 电机并启动控制任务; 重复调用不重复创建。
     *
     *  配置非法 (反馈 ID 越界、LQI 参数不合法等) 或任务创建失败时直接
     *  Error_Handler(): 云台是整车核心执行机构, 不允许"半个云台"运行。
     */
	void Init();

	/** @brief 计算一次控制量并下发, 由云台控制任务按固定周期调用。
     *
     *  当前控制逻辑集中在 GimbalTask 内部执行, 保留此入口供后续
     *  统一调度器 (单一高频任务轮询所有应用模块) 接管时使用。
     */
	void Update();

	/** @brief 读取三轴实时反馈并汇总为云台控制快照。 */
	serialTasks::GimbalControlSnapshot GetEncoderSnapshot();
	/** @brief 同一控制周期目标/反馈/输出快照，供 UART 诊断。 */
	serialTasks::AxisSnapshot GetYawControlSnapshot();
	serialTasks::AxisSnapshot GetAimPitchControlSnapshot();
	serialTasks::AxisSnapshot GetFoldingPitchMotorFeedbackSnapshot();
} // namespace gimbal

#endif // INFANTRY_H723_GIMBAL_H
