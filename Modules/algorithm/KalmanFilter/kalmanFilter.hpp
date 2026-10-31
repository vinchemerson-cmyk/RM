#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <cstdint>
#include <cstring>
#include <new>

#include "kalmanFilterConfig.hpp"

/**
 * @brief 通用 Kalman 滤波器.
 *
 * @note 自定义 EKF 步骤通过 Dependency 访问矩阵和运行状态.
 */
class kalmanFilter
{
public:
	using Config	 = kalmanFilterConfig::Config;	   // 初始化配置类型.
	using Attribute	 = kalmanFilterConfig::Attribute;  // 持久属性类型.
	using Input		 = kalmanFilterConfig::Input;	   // 单次更新输入类型.
	using Output	 = kalmanFilterConfig::Output;	   // 输出类型.
	using Dependency = kalmanFilterConfig::Dependency; // 运行时依赖项类型.

	/**
	 * @brief 构造并初始化 Kalman 滤波器.
	 * @param config_instance 初始化配置.
	 */
	explicit kalmanFilter(const Config &config_instance)
	{
		_init(config_instance);
		_instance.output.status = _instance.dependency.status;
	}

	/**
	 * @brief 析构并释放内部缓冲区.
	 */
	~kalmanFilter()
	{
		// 释放矩阵数据缓冲.
		_releaseData(_instance.dependency.data.xhat);
		_releaseData(_instance.dependency.data.xhatminus);
		_releaseData(_instance.dependency.data.u);
		_releaseData(_instance.dependency.data.z);
		_releaseData(_instance.dependency.data.b);
		_releaseData(_instance.dependency.data.a);
		_releaseData(_instance.dependency.data.at);
		_releaseData(_instance.dependency.data.h);
		_releaseData(_instance.dependency.data.ht);
		_releaseData(_instance.dependency.data.p);
		_releaseData(_instance.dependency.data.pminus);
		_releaseData(_instance.dependency.data.q);
		_releaseData(_instance.dependency.data.r);
		_releaseData(_instance.dependency.data.k);
		_releaseData(_instance.dependency.data.k_denominator);
		_releaseData(_instance.dependency.data.cache_matrix[0]);
		_releaseData(_instance.dependency.data.cache_matrix[1]);
		_releaseData(_instance.dependency.data.cache_vector[0]);
		_releaseData(_instance.dependency.data.cache_vector[1]);

		// 清空内部矩阵实例和输出视图.
		_instance.output		   = {};
		_instance.dependency.data  = {};
		_instance.dependency.mat   = {};
		_instance.dependency.valid = false;
	}

	kalmanFilter(const kalmanFilter &)			  = delete;
	kalmanFilter &operator=(const kalmanFilter &) = delete;

	/**
	 * @brief 更新一次 Kalman 滤波器.
	 *
	 * @param input_instance 本次观测、控制和滤波周期.
	 */
	void Update(const Input &input_instance)
	{
		// 在标准 Kalman 步骤后执行对应的用户步骤.
		const auto run_user_function = [this](const uint8_t index) {
			auto &dependency = _instance.dependency;

			if (dependency.valid &&
				dependency.status == ARM_MATH_SUCCESS &&
				dependency.user_function[index] != nullptr)
			{
				dependency.user_function[index](_instance.attribute, dependency);
			}
		};

		// 初始化失败时保持最近一次输出.
		if (!_instance.dependency.valid)
		{
			_instance.output.status = _instance.dependency.status;
			return;
		}

		// 观测和控制向量长度必须与构造配置一致.
		if ((input_instance.measured_vector.size() != _instance.attribute.z_size) ||
			(input_instance.control_vector.size() != _instance.attribute.u_size))
		{
			_instance.dependency.status = ARM_MATH_LENGTH_ERROR;
			_instance.output.status = _instance.dependency.status;
			return;
		}

		// 每次更新前清除上一帧运算状态.
		_instance.dependency.status = ARM_MATH_SUCCESS;
		_instance.dependency.dt		= input_instance.dt;

		// 读取外部观测和控制输入.
		_measurementUpdate(input_instance);
		run_user_function(0U);

		// 更新先验状态.
		_xhatminusUpdate();
		run_user_function(1U);

		// 更新先验协方差.
		_pminusUpdate();
		run_user_function(2U);

		// 更新 Kalman 增益.
		_kUpdate();
		run_user_function(3U);

		// 更新后验状态.
		_xhatUpdate();
		run_user_function(4U);

		// 更新后验协方差.
		_pUpdate();
		run_user_function(5U);

		// 预留最后一个用户步骤.
		run_user_function(6U);

		_instance.output.status = _instance.dependency.status;
	}

	/**
	 * @brief 获取最近一次后验状态估计.
	 *
	 * @return 最近一次滤波输出.
	 */
	[[nodiscard]] const Output &Receive() const
	{
		return _instance.output;
	}

private:
	struct Instance
	{
		Attribute attribute{};	 // 构造配置的持久副本.
		Output output{};		 // 最近一次后验状态输出视图.
		Dependency dependency{}; // CMSIS 矩阵、缓冲和运行状态.
	};

	Instance _instance{};

	/**
	 * @brief 初始化矩阵数据和 CMSIS 矩阵实例.
	 * @param config_instance 初始化配置.
	 */
	void _init(const Config &config_instance)
	{
		// 保存向量维度.
		_instance.attribute.xhat_size = config_instance.xhat_size;
		_instance.attribute.u_size	  = config_instance.u_size;
		_instance.attribute.z_size	  = config_instance.z_size;
		_instance.dependency.user_context = config_instance.user_context;
		_instance.dependency.user_function = config_instance.user_function;
		_instance.dependency.skip_step	  = config_instance.skip_step;

		// 状态和观测维度必须非零.
		if (_instance.attribute.xhat_size == 0 || _instance.attribute.z_size == 0)
		{
			_instance.dependency.status = ARM_MATH_LENGTH_ERROR;
			return;
		}

		// 计算缓存矩阵需要的最大尺寸.
		_instance.dependency.max_dim = (_instance.attribute.xhat_size > _instance.attribute.z_size) ? _instance.attribute.xhat_size : _instance.attribute.z_size;

		const uint32_t x_square			= static_cast<uint32_t>(_instance.attribute.xhat_size) * _instance.attribute.xhat_size;
		const uint32_t z_square			= static_cast<uint32_t>(_instance.attribute.z_size) * _instance.attribute.z_size;
		_instance.dependency.max_square = static_cast<uint32_t>(_instance.dependency.max_dim) * _instance.dependency.max_dim;

		// 初始化卡方检测矩阵.
		arm_mat_init_f32(&_instance.dependency.chi_square_test.chi_square_matrix, 1, 1, _instance.dependency.chi_square_test.chi_square_data);

		// 分配基础向量缓冲.
		_instance.dependency.data.xhat		= _newData(_instance.attribute.xhat_size);
		_instance.dependency.data.xhatminus = _newData(_instance.attribute.xhat_size);
		_instance.dependency.data.z			= _newData(_instance.attribute.z_size);

		// 有控制量时分配控制相关缓冲.
		if (_instance.attribute.u_size > 0)
		{
			_instance.dependency.data.u = _newData(_instance.attribute.u_size);
			_instance.dependency.data.b = _newData(static_cast<uint32_t>(_instance.attribute.xhat_size) * _instance.attribute.u_size);
		}

		// 分配矩阵缓冲.
		_instance.dependency.data.a				  = _newData(x_square);
		_instance.dependency.data.at			  = _newData(x_square);
		_instance.dependency.data.h				  = _newData(static_cast<uint32_t>(_instance.attribute.z_size) * _instance.attribute.xhat_size);
		_instance.dependency.data.ht			  = _newData(static_cast<uint32_t>(_instance.attribute.xhat_size) * _instance.attribute.z_size);
		_instance.dependency.data.p				  = _newData(x_square);
		_instance.dependency.data.pminus		  = _newData(x_square);
		_instance.dependency.data.q				  = _newData(x_square);
		_instance.dependency.data.r				  = _newData(z_square);
		_instance.dependency.data.k				  = _newData(static_cast<uint32_t>(_instance.attribute.xhat_size) * _instance.attribute.z_size);
		_instance.dependency.data.k_denominator	  = _newData(z_square);
		_instance.dependency.data.cache_matrix[0] = _newData(_instance.dependency.max_square);
		_instance.dependency.data.cache_matrix[1] = _newData(_instance.dependency.max_square);
		_instance.dependency.data.cache_vector[0] = _newData(_instance.dependency.max_dim);
		_instance.dependency.data.cache_vector[1] = _newData(_instance.dependency.max_dim);

		// 任意分配失败时停止初始化.
		if (_instance.dependency.status != ARM_MATH_SUCCESS)
			return;

		// 绑定基础向量矩阵.
		arm_mat_init_f32(&_instance.dependency.mat.xhat, _instance.attribute.xhat_size, 1, _instance.dependency.data.xhat);
		arm_mat_init_f32(&_instance.dependency.mat.xhatminus, _instance.attribute.xhat_size, 1, _instance.dependency.data.xhatminus);
		arm_mat_init_f32(&_instance.dependency.mat.z, _instance.attribute.z_size, 1, _instance.dependency.data.z);

		// 绑定控制相关矩阵.
		if (_instance.attribute.u_size > 0)
		{
			arm_mat_init_f32(&_instance.dependency.mat.u, _instance.attribute.u_size, 1, _instance.dependency.data.u);
			arm_mat_init_f32(&_instance.dependency.mat.b, _instance.attribute.xhat_size, _instance.attribute.u_size, _instance.dependency.data.b);
		}

		// 绑定 Kalman 标准矩阵.
		arm_mat_init_f32(&_instance.dependency.mat.a, _instance.attribute.xhat_size, _instance.attribute.xhat_size, _instance.dependency.data.a);
		arm_mat_init_f32(&_instance.dependency.mat.at, _instance.attribute.xhat_size, _instance.attribute.xhat_size, _instance.dependency.data.at);
		arm_mat_init_f32(&_instance.dependency.mat.h, _instance.attribute.z_size, _instance.attribute.xhat_size, _instance.dependency.data.h);
		arm_mat_init_f32(&_instance.dependency.mat.ht, _instance.attribute.xhat_size, _instance.attribute.z_size, _instance.dependency.data.ht);
		arm_mat_init_f32(&_instance.dependency.mat.p, _instance.attribute.xhat_size, _instance.attribute.xhat_size, _instance.dependency.data.p);
		arm_mat_init_f32(&_instance.dependency.mat.pminus, _instance.attribute.xhat_size, _instance.attribute.xhat_size, _instance.dependency.data.pminus);
		arm_mat_init_f32(&_instance.dependency.mat.q, _instance.attribute.xhat_size, _instance.attribute.xhat_size, _instance.dependency.data.q);
		arm_mat_init_f32(&_instance.dependency.mat.r, _instance.attribute.z_size, _instance.attribute.z_size, _instance.dependency.data.r);
		arm_mat_init_f32(&_instance.dependency.mat.k, _instance.attribute.xhat_size, _instance.attribute.z_size, _instance.dependency.data.k);
		arm_mat_init_f32(&_instance.dependency.mat.k_denominator, _instance.attribute.z_size, _instance.attribute.z_size, _instance.dependency.data.k_denominator);

		// 绑定缓存矩阵.
		arm_mat_init_f32(&_instance.dependency.mat.cache_matrix[0], _instance.dependency.max_dim, _instance.dependency.max_dim, _instance.dependency.data.cache_matrix[0]);
		arm_mat_init_f32(&_instance.dependency.mat.cache_matrix[1], _instance.dependency.max_dim, _instance.dependency.max_dim, _instance.dependency.data.cache_matrix[1]);
		arm_mat_init_f32(&_instance.dependency.mat.cache_vector[0], _instance.dependency.max_dim, 1, _instance.dependency.data.cache_vector[0]);
		arm_mat_init_f32(&_instance.dependency.mat.cache_vector[1], _instance.dependency.max_dim, 1, _instance.dependency.data.cache_vector[1]);

		// 标记初始化成功.
		_instance.output.state	   = std::span<const float>(_instance.dependency.data.xhat, _instance.attribute.xhat_size);
		_instance.dependency.valid = true;
	}

	/**
	 * @brief 分配并清零 float 缓冲区.
	 * @param size 元素数量.
	 * @return 缓冲区指针.
	 */
	float *_newData(const uint32_t size)
	{
		// 零长度缓冲不分配.
		if (size == 0)
			return nullptr;

		// 使用 nothrow 避免异常.
		auto *buffer = new (std::nothrow) float[size]{};

		// 分配失败时记录错误.
		if (buffer == nullptr)
			_instance.dependency.status = ARM_MATH_ARGUMENT_ERROR;

		return buffer;
	}

	/**
	 * @brief 释放单个 float 缓冲区并清空指针.
	 * @param data 待释放的缓冲区指针.
	 */
	static void _releaseData(float *&data)
	{
		delete[] data;
		data = nullptr;
	}

	/**
	 * @brief 设置运算状态.
	 * @param new_status 新的 CMSIS 状态.
	 * @return 当前是否仍成功.
	 */
	bool _setStatus(const arm_status new_status)
	{
		// 保留第一次错误.
		if (_instance.dependency.status != ARM_MATH_SUCCESS)
			return false;

		// 记录本次运算状态.
		_instance.dependency.status = new_status;

		return _instance.dependency.status == ARM_MATH_SUCCESS;
	}

	/**
	 * @brief 更新观测向量和控制向量.
	 *
	 * @param input_instance 本次观测和控制向量.
	 */
	void _measurementUpdate(const Input &input_instance)
	{
		// 错误状态下跳过.
		if (!_instance.dependency.valid ||
			_instance.dependency.status != ARM_MATH_SUCCESS)
			return;

		// 拷贝观测输入到内部 Z.
		std::memcpy(_instance.dependency.data.z, input_instance.measured_vector.data(), sizeof(float) * _instance.attribute.z_size);

		// 有控制量时拷贝控制输入.
		if (_instance.attribute.u_size > 0)
			std::memcpy(_instance.dependency.data.u, input_instance.control_vector.data(), sizeof(float) * _instance.attribute.u_size);
	}

	/**
	 * @brief 更新先验状态 Xhatminus.
	 */
	void _xhatminusUpdate()
	{
		// 跳过错误状态或用户替代步骤.
		if (!_instance.dependency.valid ||
			_instance.dependency.status != ARM_MATH_SUCCESS ||
			_instance.dependency.skip_step[0])
			return;

		if (_instance.attribute.u_size > 0)
		{
			// Cache_Vector[0] = A * Xhat.
			_instance.dependency.mat.cache_vector[0].numRows = _instance.attribute.xhat_size;
			_instance.dependency.mat.cache_vector[0].numCols = 1;
			if (!_setStatus(arm_mat_mult_f32(&_instance.dependency.mat.a, &_instance.dependency.mat.xhat, &_instance.dependency.mat.cache_vector[0])))
				return;

			// Cache_Vector[1] = B * U.
			_instance.dependency.mat.cache_vector[1].numRows = _instance.attribute.xhat_size;
			_instance.dependency.mat.cache_vector[1].numCols = 1;
			if (!_setStatus(arm_mat_mult_f32(&_instance.dependency.mat.b, &_instance.dependency.mat.u, &_instance.dependency.mat.cache_vector[1])))
				return;

			// Xhatminus = A * Xhat + B * U.
			_setStatus(arm_mat_add_f32(&_instance.dependency.mat.cache_vector[0], &_instance.dependency.mat.cache_vector[1], &_instance.dependency.mat.xhatminus));
		}
		else
		{
			// Xhatminus = A * Xhat.
			_setStatus(arm_mat_mult_f32(&_instance.dependency.mat.a, &_instance.dependency.mat.xhat, &_instance.dependency.mat.xhatminus));
		}
	}

	/**
	 * @brief 更新先验协方差 Pminus.
	 */
	void _pminusUpdate()
	{
		// 跳过错误状态或用户替代步骤.
		if (!_instance.dependency.valid ||
			_instance.dependency.status != ARM_MATH_SUCCESS ||
			_instance.dependency.skip_step[1])
			return;

		// AT = A^T.
		if (!_setStatus(arm_mat_trans_f32(&_instance.dependency.mat.a, &_instance.dependency.mat.at)))
			return;

		// Pminus = A * P.
		if (!_setStatus(arm_mat_mult_f32(&_instance.dependency.mat.a, &_instance.dependency.mat.p, &_instance.dependency.mat.pminus)))
			return;

		// Cache_Matrix[0] = A * P * AT.
		_instance.dependency.mat.cache_matrix[0].numRows = _instance.dependency.mat.pminus.numRows;
		_instance.dependency.mat.cache_matrix[0].numCols = _instance.dependency.mat.at.numCols;
		if (!_setStatus(arm_mat_mult_f32(&_instance.dependency.mat.pminus, &_instance.dependency.mat.at, &_instance.dependency.mat.cache_matrix[0])))
			return;

		// Pminus = A * P * AT + Q.
		_setStatus(arm_mat_add_f32(&_instance.dependency.mat.cache_matrix[0], &_instance.dependency.mat.q, &_instance.dependency.mat.pminus));
	}

	/**
	 * @brief 更新 Kalman 增益 K.
	 */
	void _kUpdate()
	{
		// 跳过错误状态或用户替代步骤.
		if (!_instance.dependency.valid ||
			_instance.dependency.status != ARM_MATH_SUCCESS ||
			_instance.dependency.skip_step[2])
			return;

		// HT = H^T.
		if (!_setStatus(arm_mat_trans_f32(&_instance.dependency.mat.h, &_instance.dependency.mat.ht)))
			return;

		// Cache_Matrix[0] = H * Pminus.
		_instance.dependency.mat.cache_matrix[0].numRows = _instance.dependency.mat.h.numRows;
		_instance.dependency.mat.cache_matrix[0].numCols = _instance.dependency.mat.pminus.numCols;
		if (!_setStatus(arm_mat_mult_f32(&_instance.dependency.mat.h, &_instance.dependency.mat.pminus, &_instance.dependency.mat.cache_matrix[0])))
			return;

		// Cache_Matrix[1] = H * Pminus * HT.
		_instance.dependency.mat.cache_matrix[1].numRows = _instance.dependency.mat.cache_matrix[0].numRows;
		_instance.dependency.mat.cache_matrix[1].numCols = _instance.dependency.mat.ht.numCols;
		if (!_setStatus(arm_mat_mult_f32(&_instance.dependency.mat.cache_matrix[0], &_instance.dependency.mat.ht, &_instance.dependency.mat.cache_matrix[1])))
			return;

		// K_denominator = H * Pminus * HT + R.
		_instance.dependency.mat.k_denominator.numRows = _instance.dependency.mat.r.numRows;
		_instance.dependency.mat.k_denominator.numCols = _instance.dependency.mat.r.numCols;
		if (!_setStatus(arm_mat_add_f32(&_instance.dependency.mat.cache_matrix[1], &_instance.dependency.mat.r, &_instance.dependency.mat.k_denominator)))
			return;

		// Cache_Matrix[1] = inverse(K_denominator).
		if (!_setStatus(arm_mat_inverse_f32(&_instance.dependency.mat.k_denominator, &_instance.dependency.mat.cache_matrix[1])))
		{
			_fallbackToPriori();
			return;
		}

		// Cache_Matrix[0] = Pminus * HT.
		_instance.dependency.mat.cache_matrix[0].numRows = _instance.dependency.mat.pminus.numRows;
		_instance.dependency.mat.cache_matrix[0].numCols = _instance.dependency.mat.ht.numCols;
		if (!_setStatus(arm_mat_mult_f32(&_instance.dependency.mat.pminus, &_instance.dependency.mat.ht, &_instance.dependency.mat.cache_matrix[0])))
			return;

		// K = Pminus * HT * inverse(K_denominator).
		_setStatus(arm_mat_mult_f32(&_instance.dependency.mat.cache_matrix[0], &_instance.dependency.mat.cache_matrix[1], &_instance.dependency.mat.k));
	}

	/**
	 * @brief 更新后验状态 Xhat.
	 */
	void _xhatUpdate()
	{
		// 跳过错误状态或用户替代步骤.
		if (!_instance.dependency.valid ||
			_instance.dependency.status != ARM_MATH_SUCCESS ||
			_instance.dependency.skip_step[3])
			return;

		// Cache_Vector[0] = H * Xhatminus.
		_instance.dependency.mat.cache_vector[0].numRows = _instance.dependency.mat.h.numRows;
		_instance.dependency.mat.cache_vector[0].numCols = 1;
		if (!_setStatus(arm_mat_mult_f32(&_instance.dependency.mat.h, &_instance.dependency.mat.xhatminus, &_instance.dependency.mat.cache_vector[0])))
			return;

		// Cache_Vector[1] = Z - H * Xhatminus.
		_instance.dependency.mat.cache_vector[1].numRows = _instance.dependency.mat.z.numRows;
		_instance.dependency.mat.cache_vector[1].numCols = 1;
		if (!_setStatus(arm_mat_sub_f32(&_instance.dependency.mat.z, &_instance.dependency.mat.cache_vector[0], &_instance.dependency.mat.cache_vector[1])))
			return;

		// Cache_Vector[0] = K * residual.
		_instance.dependency.mat.cache_vector[0].numRows = _instance.dependency.mat.k.numRows;
		_instance.dependency.mat.cache_vector[0].numCols = 1;
		if (!_setStatus(arm_mat_mult_f32(&_instance.dependency.mat.k, &_instance.dependency.mat.cache_vector[1], &_instance.dependency.mat.cache_vector[0])))
			return;

		// Xhat = Xhatminus + K * residual.
		_setStatus(arm_mat_add_f32(&_instance.dependency.mat.xhatminus, &_instance.dependency.mat.cache_vector[0], &_instance.dependency.mat.xhat));
	}

	/**
	 * @brief 更新后验协方差 P.
	 */
	void _pUpdate()
	{
		// 跳过错误状态或用户替代步骤.
		if (!_instance.dependency.valid ||
			_instance.dependency.status != ARM_MATH_SUCCESS ||
			_instance.dependency.skip_step[4])
			return;

		// Cache_Matrix[0] = K * H.
		_instance.dependency.mat.cache_matrix[0].numRows = _instance.dependency.mat.k.numRows;
		_instance.dependency.mat.cache_matrix[0].numCols = _instance.dependency.mat.h.numCols;
		if (!_setStatus(arm_mat_mult_f32(&_instance.dependency.mat.k, &_instance.dependency.mat.h, &_instance.dependency.mat.cache_matrix[0])))
			return;

		// Cache_Matrix[1] = K * H * Pminus.
		_instance.dependency.mat.cache_matrix[1].numRows = _instance.dependency.mat.cache_matrix[0].numRows;
		_instance.dependency.mat.cache_matrix[1].numCols = _instance.dependency.mat.pminus.numCols;
		if (!_setStatus(arm_mat_mult_f32(&_instance.dependency.mat.cache_matrix[0], &_instance.dependency.mat.pminus, &_instance.dependency.mat.cache_matrix[1])))
			return;

		// P = Pminus - K * H * Pminus.
		_setStatus(arm_mat_sub_f32(&_instance.dependency.mat.pminus, &_instance.dependency.mat.cache_matrix[1], &_instance.dependency.mat.p));
	}

	/**
	 * @brief 求逆失败时回退到先验估计.
	 */
	void _fallbackToPriori()
	{
		// 状态回退到先验状态.
		if (_instance.dependency.data.xhat != nullptr && _instance.dependency.data.xhatminus != nullptr)
			std::memcpy(_instance.dependency.data.xhat, _instance.dependency.data.xhatminus, sizeof(float) * _instance.attribute.xhat_size);

		// 协方差回退到先验协方差.
		if (_instance.dependency.data.p != nullptr && _instance.dependency.data.pminus != nullptr)
			std::memcpy(_instance.dependency.data.p, _instance.dependency.data.pminus, sizeof(float) * _instance.attribute.xhat_size * _instance.attribute.xhat_size);
	}
};

using kalmanFilterType = kalmanFilter; // Kalman 滤波器类型别名.

#endif // KALMAN_FILTER_HPP
