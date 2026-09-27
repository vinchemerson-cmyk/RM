/**
 * @file Robot.h
 * @brief 提供整车应用初始化的 C 语言入口。
 */

//
// Created by 19923 on 2026/9/24.
//

#ifndef INFANTRY_H723_ROBOT_H
#define INFANTRY_H723_ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 根据 RobotConfig.hpp 的开关创建整车应用所需的通信及控制任务。 */
void InfantryInit();

#ifdef __cplusplus
}
#endif

#endif
