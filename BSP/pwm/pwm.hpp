/**
 * @file pwm.hpp
 * @brief 封装定时器 PWM 通道的注册、输出和资源管理。
 */

#ifndef PWM_HPP
#define PWM_HPP

#include <array>
#include <cstdint>

#include "pwmConfig.hpp"

class pwm
{
	private:
		using pwmAttributeType  = pwmConfig::Attribute;  // PWM 持久属性类型.
		using pwmConfigType     = pwmConfig::Config;     // PWM 注册配置类型.
		using pwmDependencyType = pwmConfig::Dependency; // PWM HAL 依赖类型.

		// 单个 PWM 逻辑实例的全部运行时对象.
		struct Instance
		{
			pwmAttributeType  attribute{};  // 通道属性.
			pwmDependencyType dependency{}; // HAL 定时器依赖.
			bool              is_started{}; // 当前 PWM 通道是否已启动.
		};

		// 固定 PWM 注册表, 避免在控制周期中动态分配内存.
		static std::array<Instance, pwmConfig::MAX_INSTANCE> _pwm_instance_table;
		static uint8_t                                       _pwm_instance_amount;

		/**
	 * @brief 检查 PWM 通道是否为 HAL 支持的标准通道.
	 * @param use_channel 待检查的 PWM 通道.
	 * @return 通道合法时返回 true.
	 */
		static bool _isChannelValid(const pwmConfig::channel use_channel)
		{
			switch (use_channel)
			{
				case pwmConfig::channel::CHANNEL_1:
				case pwmConfig::channel::CHANNEL_2:
				case pwmConfig::channel::CHANNEL_3:
				case pwmConfig::channel::CHANNEL_4:
					return true;

				default:
					return false;
			}
		}

		/**
	 * @brief 通过 Attribute 指针查找完整 PWM 实例.
	 * @param attribute_instance 已注册 PWM 实例的 Attribute 指针.
	 * @return 对应的完整 PWM 实例.
	 */
		static Instance* _getInstance(const pwmAttributeType* attribute_instance)
		{
			if (attribute_instance == nullptr)
			{
				Error_Handler();
				return nullptr;
			}

			for (uint8_t i = 0; i < _pwm_instance_amount; i++)
			{
				if (&_pwm_instance_table[i].attribute == attribute_instance)
					return &_pwm_instance_table[i];
			}

			Error_Handler();
			return nullptr;
		}

		/**
	 * @brief 获取当前定时器一个 PWM 周期对应的计数值.
	 * @param instance 已注册的 PWM 完整实例.
	 * @return ARR + 1.
	 */
		static uint64_t _getPeriodCount(const Instance* instance)
		{
			if (instance == nullptr || instance->dependency.tim_handle == nullptr)
			{
				Error_Handler();
				return 0;
			}

			return static_cast<uint64_t>(__HAL_TIM_GET_AUTORELOAD(instance->dependency.tim_handle)) + 1U;
		}

	public:
		/**
	 * @brief 注册一个 PWM 逻辑实例.
	 * @param config_instance 已初始化 TIM 句柄和 PWM 通道配置.
	 * @return 已注册 PWM 实例的 Attribute 指针.
	 * @note 注册时只将比较值清零, 不会启动 PWM 输出.
	 */
		static pwmType* Register(const pwmConfigType& config_instance)
		{
			if (config_instance.tim_handle == nullptr || !_isChannelValid(config_instance.use_channel))
				Error_Handler();

			if (_pwm_instance_amount >= pwmConfig::MAX_INSTANCE)
				Error_Handler();

			for (uint8_t i = 0; i < _pwm_instance_amount; i++)
			{
				if (_pwm_instance_table[i].dependency.tim_handle == config_instance.tim_handle &&
					_pwm_instance_table[i].attribute.use_channel == config_instance.use_channel)
				{
					Error_Handler();
				}
			}

			auto* instance = &_pwm_instance_table[_pwm_instance_amount++];
			*instance      = Instance{};

			instance->attribute.use_channel = config_instance.use_channel;
			instance->dependency.tim_handle = config_instance.tim_handle;

			// 在输出启动前先写入 0 比较值, 保持默认安全电平.
			__HAL_TIM_SET_COMPARE(instance->dependency.tim_handle,
								  static_cast<uint32_t>(instance->attribute.use_channel),
								  0);

			return &instance->attribute;
		}

		/**
	 * @brief 启动 PWM 输出.
	 * @param attribute_instance 已注册 PWM 实例的 Attribute 指针.
	 * @return HAL_OK 或 HAL_ERROR.
	 * @note 已启动的实例重复调用不会再次启动.
	 */
		static HAL_StatusTypeDef Start(const pwmType* attribute_instance)
		{
			auto* instance = _getInstance(attribute_instance);

			if (instance->is_started)
				return HAL_OK;

			const HAL_StatusTypeDef status = HAL_TIM_PWM_Start(instance->dependency.tim_handle,
															   static_cast<uint32_t>(instance->attribute.use_channel));

			if (status == HAL_OK)
				instance->is_started = true;

			return status;
		}

		/**
	 * @brief 停止 PWM 输出并清零比较值.
	 * @param attribute_instance 已注册 PWM 实例的 Attribute 指针.
	 * @return HAL_OK 或 HAL_ERROR.
	 * @note 停止操作不应在中断中调用.
	 */
		static HAL_StatusTypeDef Stop(const pwmType* attribute_instance)
		{
			auto* instance = _getInstance(attribute_instance);

			if (!instance->is_started)
				return HAL_OK;

			setCompare(attribute_instance, 0);

			const HAL_StatusTypeDef status = HAL_TIM_PWM_Stop(instance->dependency.tim_handle,
															  static_cast<uint32_t>(instance->attribute.use_channel));

			if (status == HAL_OK)
				instance->is_started = false;

			return status;
		}

		/**
	 * @brief 设置 PWM 比较值.
	 * @param attribute_instance 已注册 PWM 实例的 Attribute 指针.
	 * @param compare 比较值, 范围为 0 到 ARR + 1.
	 */
		static void setCompare(const pwmType* attribute_instance, const uint32_t compare)
		{
			auto* instance = _getInstance(attribute_instance);

			if (static_cast<uint64_t>(compare) > _getPeriodCount(instance))
				Error_Handler();

			__HAL_TIM_SET_COMPARE(instance->dependency.tim_handle,
								  static_cast<uint32_t>(instance->attribute.use_channel),
								  compare);
		}

		/**
	 * @brief 设置归一化 PWM 占空比.
	 * @param attribute_instance 已注册 PWM 实例的 Attribute 指针.
	 * @param duty_cycle 占空比, 合法范围为 0.0 到 1.0.
	 */
		static void setDutyCycle(const pwmType* attribute_instance, const float duty_cycle)
		{
			if (!(duty_cycle >= 0.0F && duty_cycle <= 1.0F))
			{
				Error_Handler();
				return;
			}

			auto*          instance     = _getInstance(attribute_instance);
			const uint64_t period_count = _getPeriodCount(instance);
			const auto     compare      = static_cast<uint32_t>(
				static_cast<float>(period_count) * duty_cycle + 0.5F);
			setCompare(attribute_instance, compare);
		}
};

inline std::array<pwm::Instance, pwmConfig::MAX_INSTANCE> pwm::_pwm_instance_table{};
inline uint8_t                                            pwm::_pwm_instance_amount{};

#endif
