#ifndef KALMAN_FILTER_CONFIG_HPP
#define KALMAN_FILTER_CONFIG_HPP

#include <array>
#include <cstdint>
#include <span>

extern "C" {
#include "arm_math.h"
}

/**
 * @brief Kalman 滤波器配置和数据结构命名空间.
 */
namespace kalmanFilterConfig
{
	/**
	 * @brief Kalman 滤波器持久属性.
	 */
	struct Attribute
	{
		uint8_t xhat_size{}; // 状态向量维度.
		uint8_t u_size{};    // 控制向量维度.
		uint8_t z_size{};    // 观测向量维度.
	};

	/**
	 * @brief Kalman 滤波器单次更新输入.
	 */
	struct Input
	{
		float dt{}; // 本次滤波周期.

		std::span<const float> measured_vector{}; // 本次观测向量, 长度必须等于 z_size.
		std::span<const float> control_vector{};  // 本次控制向量, 长度必须等于 u_size.
	};

	/**
	 * @brief Kalman 滤波器输出.
	 */
	struct Output
	{
		std::span<const float> state{};                  // 最近一次后验状态估计, 长度等于 xhat_size.
		arm_status             status{ARM_MATH_SUCCESS}; // 最近一次矩阵运算状态.
	};

	/**
	 * @brief 卡方检测运行状态.
	 */
	struct ChiSquareTest
	{
		bool                    test_flag{};          // 卡方检测使能标志.
		arm_matrix_instance_f32 chi_square_matrix{};  // 卡方检测矩阵.
		float                   chi_square_data[1]{}; // 卡方检测矩阵数据.
		float                   threshold{};          // 卡方检测阈值.
		uint8_t                 count{};              // 连续检测计数.
		bool                    result{};             // 卡方检测结果.
	};

	/**
	 * @brief CMSIS 矩阵实例集合.
	 *
	 * @note 这些矩阵只描述维度和数据指针, 实际数据由 Data 保存.
	 */
	struct Mat
	{
		arm_matrix_instance_f32 xhat{};            // 后验状态估计.
		arm_matrix_instance_f32 xhatminus{};       // 先验状态估计.
		arm_matrix_instance_f32 u{};               // 控制向量.
		arm_matrix_instance_f32 z{};               // 观测向量.
		arm_matrix_instance_f32 b{};               // 控制矩阵.
		arm_matrix_instance_f32 a{};               // 状态转移矩阵.
		arm_matrix_instance_f32 at{};              // 状态转移矩阵转置.
		arm_matrix_instance_f32 h{};               // 观测矩阵.
		arm_matrix_instance_f32 ht{};              // 观测矩阵转置.
		arm_matrix_instance_f32 p{};               // 后验协方差.
		arm_matrix_instance_f32 pminus{};          // 先验协方差.
		arm_matrix_instance_f32 q{};               // 过程噪声协方差.
		arm_matrix_instance_f32 r{};               // 观测噪声协方差.
		arm_matrix_instance_f32 k{};               // Kalman 增益.
		arm_matrix_instance_f32 k_denominator{};   // 增益分母矩阵.
		arm_matrix_instance_f32 cache_matrix[2]{}; // 临时矩阵缓存.
		arm_matrix_instance_f32 cache_vector[2]{}; // 临时向量缓存.
	};

	/**
	 * @brief Kalman 滤波器数据缓冲区.
	 *
	 * @note 这些指针由 kalmanFilter 使用 new[] 分配, 析构时释放.
	 */
	struct Data
	{
		float* xhat{};            // 后验状态估计数据.
		float* xhatminus{};       // 先验状态估计数据.
		float* u{};               // 控制向量数据.
		float* z{};               // 观测向量数据.
		float* b{};               // 控制矩阵数据.
		float* a{};               // 状态转移矩阵数据.
		float* at{};              // 状态转移矩阵转置数据.
		float* h{};               // 观测矩阵数据.
		float* ht{};              // 观测矩阵转置数据.
		float* p{};               // 后验协方差数据.
		float* pminus{};          // 先验协方差数据.
		float* q{};               // 过程噪声协方差数据.
		float* r{};               // 观测噪声协方差数据.
		float* k{};               // Kalman 增益数据.
		float* k_denominator{};   // 增益分母数据.
		float* cache_matrix[2]{}; // 临时矩阵缓存数据.
		float* cache_vector[2]{}; // 临时向量缓存数据.
	};

	/**
	 * @brief Kalman 滤波器运行时依赖项.
	 *
	 * @note 自定义 EKF 步骤通过 UserFunc 访问该结构体中的矩阵与状态.
	 */
	struct Dependency
	{
		using UserFunc = void (*)(const Attribute&, Dependency&); // 用户步骤函数类型.

		float dt{};           // 当前滤波周期.
		void* user_context{}; // 自定义 Kalman 步骤的调用上下文.

		ChiSquareTest chi_square_test{}; // 卡方检测状态.
		Mat           mat{};             // CMSIS 矩阵实例.
		Data          data{};            // 矩阵数据缓冲.

		arm_status status{ARM_MATH_SUCCESS}; // 最近一次矩阵运算状态.

		std::array<UserFunc, 7> user_function{}; // 用户步骤函数.
		std::array<bool, 5>     skip_step{};     // 标准 Kalman 步骤跳过标志.

		uint8_t  max_dim{};    // 状态和观测中的最大维度.
		uint32_t max_square{}; // 最大缓存矩阵元素数.
		bool     valid{};      // 初始化完成标志.
	};

	/**
	 * @brief Kalman 滤波器初始化配置.
	 */
	struct Config
	{
		uint8_t xhat_size{}; // 状态向量维度.
		uint8_t u_size{};    // 控制向量维度, 无控制量时填 0.
		uint8_t z_size{};    // 观测向量维度.

		void* user_context{}; // 自定义 Kalman 步骤的调用上下文.

		std::array<Dependency::UserFunc, 7> user_function{}; // 自定义 Kalman 步骤函数.
		std::array<bool, 5>                 skip_step{};     // 标准 Kalman 步骤跳过标志.
	};
} // namespace kalmanFilterConfig

using kalmanFilterConfigType     = kalmanFilterConfig::Config;     // Kalman 滤波器配置类型.
using kalmanFilterAttributeType  = kalmanFilterConfig::Attribute;  // Kalman 滤波器持久属性类型.
using kalmanFilterInputType      = kalmanFilterConfig::Input;      // Kalman 滤波器输入类型.
using kalmanFilterOutputType     = kalmanFilterConfig::Output;     // Kalman 滤波器输出类型.
using kalmanFilterDependencyType = kalmanFilterConfig::Dependency; // Kalman 滤波器运行时依赖项类型.

#endif // KALMAN_FILTER_CONFIG_HPP
