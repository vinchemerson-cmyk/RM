/** @file encoder_serial_task.h @brief UART7 FireWater 遥测任务入口；按角色输出云台或底盘快照。 */
#pragma once
#include "encoder_telemetry.h"

namespace serialTasks
{
	/** @brief 创建 UART7 TX-only 任务；provider 提供同步反馈快照，UART7 不得再注册回环任务。 */
	void CreateEncoderSerialTask(EncoderProvider provider);

	/** @brief 创建 UART7 底盘遥测任务；与 CreateEncoderSerialTask 共用同一个任务句柄。 */
	void CreateChassisSerialTask(ChassisProvider provider);
}
