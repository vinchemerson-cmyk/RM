#ifndef DWT_HPP
#define DWT_HPP

#include <cstdint>

extern "C"
{
#include "main.h"
}

class dwt
{
	uint32_t _last_time{}; // 上一次读取的 CYCCNT
	uint32_t _now_time{};  // 本次读取的 CYCCNT

	static inline bool _first_use	= false; // 硬件是否已初始化
	static inline uint32_t _cpu_mhz = 0;	 // 核心频率（MHz）

	static void _enable(const uint32_t core_mhz)
	{
		/* Initializes the DWT peripheral */
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

		/* clear the DWT CYCCNT register */
		DWT->CYCCNT = 0u;

		/* Enable Cortex-M DWT CYCCNT register */
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

		/* update the clock frequence */
		_cpu_mhz = core_mhz;
	}

	// 读取当前 CYCCNT，跟上次的差值作为原始周期数返回
	float _delta_raw()
	{
		_now_time = DWT->CYCCNT; // 读硬件计数器

		const float dt = static_cast<float>(_now_time - _last_time); // 无符号减法，自动处理溢出

		_last_time = _now_time; // 更新基准点

		return dt;
	}

public:
	// 构造函数：初始化 DWT（仅首次）并记录初始时间戳
	explicit dwt(const uint32_t core_mhz)
	{
		if (!_first_use) // 第一次构造时才配硬件
		{
			_enable(core_mhz);
			_first_use = true;
		}
		_last_time = DWT->CYCCNT; // 以当前时刻为基准点
	}

	~dwt() = default;

	// 距上次调用的微秒数
	float getDeltaUs()
	{
		return _delta_raw() / static_cast<float>(_cpu_mhz); // 周期数 ÷ MHz = 微秒
	}

	// 距上次调用的毫秒数
	float getDeltaMs() { return getDeltaUs() * 0.001f; }

	// 距上次调用的秒数
	float getDeltaS() { return getDeltaUs() * 0.000001f; }
};

#endif
