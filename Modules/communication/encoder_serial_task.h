/** @file encoder_serial_task.h @brief UART7 yaw FireWater 五通道诊断任务入口。 */
#pragma once
#include "encoder_telemetry.h"

namespace serialTasks
{
/** @brief 创建 UART7 TX-only 任务；provider 提供同步反馈快照，UART7 不得再注册回环任务。 */
void CreateEncoderSerialTask(EncoderProvider provider);
}
