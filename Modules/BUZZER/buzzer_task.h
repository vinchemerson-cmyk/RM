/**
 * @file buzzer_task.h
 * @brief 声明蜂鸣器任务创建入口。
 */

//
// Created by 19923 on 2026/9/24.
//

#ifndef INFANTRY_H723_BUZZER_TASK_H
#define INFANTRY_H723_BUZZER_TASK_H

namespace appTasks
{

    /** @brief 初始化蜂鸣器并创建周期鸣响任务；重复调用不会创建第二个任务。 */
    void CreateBuzzerTask();
}
#endif //INFANTRY_H723_BUZZER_TASK_H
