/**
 * @file safety.cpp
 * @brief 监控遥控拨杆与掉线状态, 维护输出使能标志。
 */

#include "safety.h"

#include "cmsis_os2.h"
#include "main.h"
#include "rc/rc.h"

namespace
{
	//输出联锁
	class OutputInterlock
	{
		public:
			/** @brief 急停立即生效；在线、摇杆居中且双 DOWN 连续三次后允许解除急停。 */
			bool Update(bool online, bool both_down, bool sticks_neutral)
			{
				if (!online)
				{
					armed_       = false; //是否允许输出
					down_cycles_ = 0U;    //连续满足解除条件的周期计数
					return false;
				}
				if (both_down)
				{
					if (!sticks_neutral)
						down_cycles_ = 0U;
					else if (down_cycles_ < 3U)
						++down_cycles_;
					if (down_cycles_ >= 3U)
						armed_ = true;
					return false;
				}
				down_cycles_ = 0U;
				return armed_;
			}

			[[nodiscard]] bool Armed() const
			{
				return armed_;
			}

		private:
			bool    armed_{};
			uint8_t down_cycles_{};
	};
}

namespace
{
	osThreadId_t taskHandle = nullptr;

	constexpr osThreadAttr_t taskAttrs = {
		.name       = "SafetyTask",
		.stack_size = 1024U,
		// 高于普通应用任务, 保证急停判定不被低的控制任务拖延。
		.priority   = osPriorityAboveNormal,
	};

	/** @brief 判定后的急停状态: 两个拨杆同时在 DOWN。 */
	volatile bool e_stopped = false;

	/** @brief 最近一次判定结果是否为遥控掉线。 */
	volatile bool offline = true;

	OutputInterlock interlock{};
	volatile bool           armed = false;//是否武装

	/**
 * @brief 原始急停手势: 左右拨杆是否同时处于 DOWN。
 */
	bool RawBothDown()
	{
		const auto state = rc::Snapshot();
		return state.switch_left == rc::Left_Switch_e::DOWN &&
			   state.switch_right == rc::Right_Switch_e::DOWN;
	}

	/**
 * @brief 安全监控任务。
 * @note 上电和遥控失联后均需双 DOWN、摇杆居中解锁，不自动恢复动作。
 */
	void Task(void*)
	{
		for (;;)
		{
			offline = !rc::IsOnline();

			const auto state    = rc::Snapshot();
			const bool bothDown = state.switch_left == rc::Left_Switch_e::DOWN &&
								  state.switch_right == rc::Right_Switch_e::DOWN;
			bool neutral = true;
			for (uint16_t channel : state.channels)
				neutral = neutral && rc::Normalize(channel) == 0.0F;
			e_stopped = !interlock.Update(!offline, bothDown, neutral);
			armed     = interlock.Armed();

			osDelay(safety::TASK_PERIOD_MS);
		}
	}
} // namespace


bool safety::ShouldOutput()
{
	// 安全任务完成首次在线判定前保持关闭；掉线也立即失效,主要检测
	//todo:底盘不能用offline
	return armed && !offline && !e_stopped && !RawBothDown() && rc::IsOnline();
}

bool safety::IsEStopped()
{
	return e_stopped;
}

bool safety::BothDown()
{
	return RawBothDown();
}

void safety::Init()
{
	if (taskHandle != nullptr)
		return;
	taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
	if (taskHandle == nullptr)
		Error_Handler();
}
