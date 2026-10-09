/**
 * @file Robot.h
 * @brief 提供整车应用初始化的 C 语言入口。
 *
 * 该头文件是 Application 层与 CubeMX 生成代码之间的唯一边界:
 * Core/Src/freertos.c 在调度器启动前以 C 方式调用 InfantryInit(),
 * 由它进入各 C++ 模块完成装配。因此本头文件不暴露任何 C++ 类型,
 * 只保留一个无参无返回值的 C 接口, 便于在纯 C 编译单元中链接。
 */

//
// Created by 19923 on 2026/9/24.
//

#ifndef INFANTRY_H723_ROBOT_H
#define INFANTRY_H723_ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

	/** @brief 根据 RobotConfig.hpp 的开关创建整车应用所需的通信及控制任务。
 *
 *  调用约定:
 *   - 必须在 FreeRTOS 调度器启动前调用 (当前由 freertos.c 的
 *     MX_FREERTOS_Init() 调用), 因为部分模块要求话题注册等操作
 *     发生在 osKernelStart() 之前。
 *   - 内部有幂等保护, 重复调用只生效一次。
 *   - 装配失败 (如双板链路启动失败) 会直接进入 Error_Handler(),
 *     不存在"带病运行"的中间状态。
 */
	void InfantryInit();

#ifdef __cplusplus
}
#endif

#endif // INFANTRY_H723_ROBOT_H
