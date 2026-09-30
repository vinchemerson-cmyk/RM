/**
 * @file ImuTask.h
 * @brief BMI088 周期采样任务及最新姿态快照接口。
 */

#ifndef INFANTRY_H723_IMU_TASK_H
#define INFANTRY_H723_IMU_TASK_H

#include "AttitudeEstimator.h"

namespace imu
{
    /** @brief 创建采样与姿态解算任务；传感器初始化在任务启动后进行。 */
    void CreateTask();

    /**
     * @brief 读取最新有效姿态。
     * @param output 成功时写入 Roll、Pitch、Yaw 和采样时间戳。
     * @return 尚无有效采样或最近 100 ms 无新样本时返回 false。
     * @note 仅从任务上下文调用。
     */
    bool Latest(AttitudeSample* output);
}

#endif // INFANTRY_H723_IMU_TASK_H
