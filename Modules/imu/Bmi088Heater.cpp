/**
 * @file Bmi088Heater.cpp
 * @brief BMI088 温度反馈到 PID，再到 PB1/TIM3_CH4 PWM 的控制链路。
 */

#include "Bmi088Heater.h"

#include "cmsis_os2.h"
#include "tim.h"
#include "../../BSP/imu/Bmi088.h"
#include "pwm/pwm.hpp"

#include <cmath>

namespace bmi088Heater
{
	namespace
	{
		PIDInstance  controller{};         //BMI088的加热器PID实例
		pwmType*     heaterPwm = nullptr;  //加热器pwm类型
		float        targetCelsius{};      //目标摄氏度
		float        cutoffCelsius{};      //过温阈值
		bool         running{};            //是否运行
		Config       taskConfig{};         //加热器配置
		osThreadId_t taskHandle = nullptr; //任务句柄

		constexpr osThreadAttr_t taskAttributes = {
			.name       = "Bmi088Heater",
			.stack_size = 2048U,
			.priority   = osPriorityBelowNormal,
		};

		//配置是否合法
		bool ValidConfig(const Config& config)
		{
			//非法判定
			if (!std::isfinite(config.target_temper) || !std::isfinite(config.cutoff_temper) ||
				config.target_temper <= 40.0F || config.target_temper >= config.cutoff_temper ||
				config.cutoff_temper > 60.0F || config.pid.output_min != 0.0F ||
				config.pid.output_max > 1.0F)
				return false;
			PIDInstance candidate;//候选
			return candidate.Configure(config.pid);
		}

		//零输出
		void ZeroOutput()
		{
			if (heaterPwm != nullptr)
				pwm::setDutyCycle(heaterPwm, 0.0F);
		}

		void Task(void*)
		{
			if (!Start(taskConfig))
			{
				taskHandle = nullptr;
				return;
			}

			uint32_t       previousTick  = osKernelGetTickCount();
			const uint32_t intervalTicks = (1300U * osKernelGetTickFreq() + 999U) / 1000U;
			for (;;)
			{
				osDelay(intervalTicks);
				const uint32_t now = osKernelGetTickCount();
				const float    dt  = static_cast<float>(now - previousTick) /
								 static_cast<float>(osKernelGetTickFreq());
				previousTick = now;
				if (!Update(dt))
					break;
			}
			taskHandle = nullptr;
		}
	} // namespace

	bool Start(const Config& config)
	{
		if (running || osKernelGetState() != osKernelRunning ||
			!ValidConfig(config) || !controller.Configure(config.pid))
			return false;

		if (BMI088_Init() != HAL_OK || BMI088_EnableTemperature() != HAL_OK)
		{
			controller.Disable();
			return false;
		}

		// 温度数据的更新周期约为 1.28 秒；期间保持 PWM 为零。
		ZeroOutput();
		osDelay((1300U * osKernelGetTickFreq() + 999U) / 1000U);
		float initialTemperature{};//温度下限
		if (BMI088_ReadTemperature(&initialTemperature) != HAL_OK ||
			!std::isfinite(initialTemperature) || initialTemperature < 30.0F ||//todo: 加热器初始温度应该略小于环境温度
			initialTemperature >= config.cutoff_temper)
		{
			controller.Disable();
			return false;
		}

		if (heaterPwm == nullptr)
		{
			constexpr pwmConfig::Config pwmConfig{.tim_handle = &htim3, .use_channel = pwmConfig::channel::CHANNEL_4};
			heaterPwm = pwm::Register(pwmConfig);
		}
		ZeroOutput();
		if (pwm::Start(heaterPwm) != HAL_OK)
		{
			controller.Disable();
			return false;
		}

		targetCelsius = config.target_temper;
		cutoffCelsius = config.cutoff_temper;
		running       = true;
		return true;
	}

	bool Update(float dt_s)
	{
		if (!running || !std::isfinite(dt_s) || dt_s <= 0.0F || dt_s > 3.0F)
		{
			Stop();
			return false;
		}

		float temperature{};
		if (BMI088_ReadTemperature(&temperature) != HAL_OK ||
			!std::isfinite(temperature) || temperature < -10.0F ||
			temperature >= cutoffCelsius)
		{
			Stop();
			return false;
		}

		pwm::setDutyCycle( heaterPwm, controller.Update(targetCelsius, temperature, dt_s));
		return true;
	}

	bool CreateTask(const Config& config)
	{
		if (taskHandle != nullptr || running || !ValidConfig(config))
			return false;
		taskConfig = config;
		taskHandle = osThreadNew(Task, nullptr, &taskAttributes);
		return taskHandle != nullptr;
	}

	void Stop()
	{
		ZeroOutput();
		controller.Disable();
		running = false;
	}

	bool IsRunning()
	{
		return running;
	}
} // namespace bmi088Heater
