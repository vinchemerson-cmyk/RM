/**
 * @file Bmi088Heater.h
 * @brief 使用通用 PID 和 TIM3_CH4 控制 BMI088 加热片。
 */

#ifndef INFANTRY_H723_BMI088_HEATER_H
#define INFANTRY_H723_BMI088_HEATER_H

#include "algorithm/pid.h"

namespace bmi088Heater
{
	/** @brief 加热目标、过温阈值以及输出范围为 0 到 1 的 PID 参数。 */
	struct Config
	{
		PID_Init_Config_s pid{};
		float             target_temper{}; //目标温度
		float             cutoff_temper{};
	};

	/**
 * @brief 验证传感器与参数，等待第一笔有效温度后启动零占空比 PWM。
 * @param config 加热参数；目标温度必须低于截止温度且截止温度不高于 85°C。
 * @return 全部初始化成功时为 true。
 * @note 必须在 FreeRTOS 任务中调用；该函数会等待约 1.3 秒。
 */
	bool Start(const Config& config);

	/**
 * @brief 创建约每 1.3 秒更新一次的独立加热任务。
 * @return 任务创建成功时为 true；传感器或 PWM 初始化结果可通过 IsRunning() 查询。
 * @note 可在调度器启动前调用；加热参数必须先由整车应用明确提供。
 */
	bool CreateTask(const Config& config);

	/**
 * @brief 读取温度并更新一次 PWM 占空比。
 * @param dt_s 距离上一次更新的实际秒数；温度寄存器每约 1.28 秒刷新。
 * @return 正常更新时为 true；读取失败、无效输入或过温时关闭加热并返回 false。
 * @note 手动调用模式下，调用方必须保证周期执行；CreateTask() 可提供固定周期。
 */
	bool Update(float dt_s);

	/** @brief 将加热 PWM 置零并清空 PID 状态。 */
	void Stop();

	/** @brief 查询加热控制是否已启动。 */
	bool IsRunning();
} // namespace bmi088Heater

#endif // INFANTRY_H723_BMI088_HEATER_H
