#pragma once
#include <cstdint>
uint32_t osKernelGetTickCount();
using osThreadId_t                  = void*;
inline constexpr int osPriorityHigh = 40;

struct osThreadAttr_t
{
	const char* name{};
	uint32_t    stack_size{};
	int         priority{};
};

osThreadId_t osThreadNew(void (*)(void*), void*, const osThreadAttr_t*);
void         osDelay(uint32_t);
void         osDelayUntil(uint32_t);
