/**
 * @file gimbal.h
 * @brief 云台 yaw/pitch 两轴控制入口。
 *
 * 数据流 (每个控制周期, 约 1kHz):
 *   DT7 遥控器 -> UART5 中断解析 (rc 模块)
 *     -> GimbalTask 读取右摇杆归一化值 ([-1, 1])
 *     -> 按速度/位置模式计算两轴目标并写入 motor_settings
 *     -> LQI 计算电流，DJI_Motor_Control() 打包为一帧 0x1FE 电流指令,
 *        发送到 GimbalConfig.hpp 指定的 FDCAN (当前 FDCAN1)。
 *
 * @note 输入: 遥控器右摇杆。右摇杆水平 -> yaw 角速度, 右摇杆垂直 -> pitch 角速度。
 * @note 两轴使用 LQI 位置控制，摇杆推动角度目标；当前 Manual 模式使用手填 K。
 *       可改为 BootSolve 上电各求解一次，届时须先填写当前零占位的 A/B。
 * @note 安全: 遥控失联、急停 (双拨杆 DOWN) 时该轴立即 Disable 并输出零。
 *       当前pitch/yaw均为位置模式；pitch进入端点保护区时仅接受手动向零位退回，
 *       松杆或向外指令时Disable，退回安全区后同步当前位置并恢复位置保持。
 *       位置控制需首帧定位及有效机械限位；未标定不能编译启用。
 *       收到首帧后仍不做电机反馈超时停机；摩擦轮继续使用原控制逻辑。
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

	/** @brief 读取编码器反馈和 pitch 相对水平零位的位置目标、实际位置与电流。 */
	serialTasks::EncoderSnapshot GetEncoderSnapshot();
	/** @brief 同一控制周期目标/反馈/输出快照，供 UART 诊断。 */
	serialTasks::EncoderSnapshot GetYawControlSnapshot();
} // namespace gimbal

#endif // INFANTRY_H723_GIMBAL_H
