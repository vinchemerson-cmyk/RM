/**
 * @file delay.hpp
 * @brief 基于 HAL 时基提供微秒和毫秒级等待工具。
 */

#ifndef DELAY_HPP
#define DELAY_HPP

#include <cstdint>

extern "C"
{
#include "main.h"
}

/**
 * @brief 基于 HAL 微秒时基的非阻塞计时工具。
 *
 * 当前工程使用 1 MHz 的 TIM6 作为 HAL 时基，因此 TIM6 的 CNT 每计数一次
 * 代表 1 us，并且每 1 ms 回卷一次。wait()、us() 和 ms() 只创建计时器，
 * 不会在函数内部循环等待。
 *
 * @code
 * auto timer = delay::ms(80U);
 *
 * for (;;)
 * {
 *     if (timer.poll())
 *     {
 *         // 80 ms 到期后执行一次
 *         timer.restartMs(80U);
 *     }
 *
 *     osDelay(1U); // 调用者仍应主动让出 CPU
 * }
 * @endcode
 */
namespace delay
{
	using bspTimerTypeDef = TIM_TypeDef;

	constexpr uint32_t MICROSECONDS_PER_MILLISECOND = 1000U;
	constexpr uint32_t MAX_INTERVAL_US              = 0x7FFFFFFFU;

	// 当前工程的 HAL 时基为 TIM6；也可通过 init() 换成相同配置的定时器。
	inline bspTimerTypeDef *time_base = TIM6;

	/**
	 * @brief 指定用于微秒时间戳的定时器。
	 * @param tim 以 1 MHz 计数、每 1 ms 回卷一次的定时器。
	 */
	inline void init(bspTimerTypeDef *tim)
	{
		if (tim == nullptr)
			Error_Handler();

		time_base = tim;
	}

	/**
	 * @brief 获取当前 32 位微秒时间戳。
	 * @return 以微秒为单位的时间戳；无符号差值可自动处理回卷。
	 * @note 应在 HAL_Init() 完成、HAL 时基开始运行后调用。
	 */
	inline uint32_t now()
	{
		uint32_t milliseconds_before = 0U;
		uint32_t milliseconds_after  = 0U;
		uint32_t microseconds         = 0U;

		do
		{
			milliseconds_before = HAL_GetTick();
			microseconds         = time_base->CNT;
			milliseconds_after  = HAL_GetTick();
		} while (milliseconds_before != milliseconds_after);

		return milliseconds_after * MICROSECONDS_PER_MILLISECOND + microseconds;
	}

	/**
	 * @brief 调用者独占的非阻塞计时器。
	 */
	class Timer
	{
		uint32_t _start_time_us{};
		uint32_t _interval_us{};
		bool     _active{};

		static uint32_t _millisecondsToMicroseconds(const uint32_t milliseconds)
		{
			const uint32_t maximum_milliseconds = MAX_INTERVAL_US / MICROSECONDS_PER_MILLISECOND;

			if (milliseconds > maximum_milliseconds)
				return MAX_INTERVAL_US;

			return milliseconds * MICROSECONDS_PER_MILLISECOND;
		}

	public:
		Timer() = default;

		explicit Timer(const uint32_t interval_us)
		{
			restartUs(interval_us);
		}

		/**
		 * @brief 从当前时刻重新开始微秒计时。
		 * @param interval_us 计时间隔，最大约 35 分钟。
		 */
		void restartUs(const uint32_t interval_us)
		{
			_start_time_us = now();
			_interval_us   = interval_us > MAX_INTERVAL_US ? MAX_INTERVAL_US : interval_us;
			_active        = true;
		}

		/**
		 * @brief 从当前时刻重新开始毫秒计时。
		 * @param interval_ms 计时间隔，超出范围时限制为约 35 分钟。
		 */
		void restartMs(const uint32_t interval_ms)
		{
			restartUs(_millisecondsToMicroseconds(interval_ms));
		}

		/**
		 * @brief 查询计时是否到期，不会等待，也不会清除到期状态。
		 * @return 已启动且到期时返回 true，否则返回 false。
		 */
		[[nodiscard]] bool expired() const
		{
			return _active && static_cast<uint32_t>(now() - _start_time_us) >= _interval_us;
		}

		/**
		 * @brief 非阻塞地查询一次到期事件。
		 * @return 本次检测到到期时返回 true，并将计时器变为未启动状态。
		 */
		[[nodiscard]] bool poll()
		{
			if (!expired())
				return false;

			_active = false;
			return true;
		}

		/**
		 * @brief 查询计时器是否已经启动。
		 * @return 已启动返回 true，否则返回 false。
		 */
		[[nodiscard]] bool active() const { return _active; }

		/**
		 * @brief 取消当前计时。
		 */
		void cancel() { _active = false; }
	};

	/**
	 * @brief 启动微秒级非阻塞等待。
	 * @param us 等待时间，单位为微秒。
	 * @return 已经启动的独立计时器对象。
	 */
	[[nodiscard]] inline Timer wait(const uint32_t us)
	{
		return Timer(us);
	}

	/**
	 * @brief 启动微秒级非阻塞等待。
	 * @param us 等待时间，单位为微秒。
	 * @return 已经启动的独立计时器对象。
	 */
	[[nodiscard]] inline Timer us(const uint32_t us)
	{
		return wait(us);
	}

	/**
	 * @brief 启动毫秒级非阻塞等待。
	 * @param ms 等待时间，单位为毫秒。
	 * @return 已经启动的独立计时器对象。
	 */
	[[nodiscard]] inline Timer ms(const uint32_t ms)
	{
		Timer timer;
		timer.restartMs(ms);
		return timer;
	}
} // namespace delay

#endif
