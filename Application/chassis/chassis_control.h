//
// Created by 19923 on 2026/10/21.
//

#ifndef INFANTRY_H723_CHASSIS_CONTROL_H
#define INFANTRY_H723_CHASSIS_CONTROL_H
#include <array>
#include <cmath>
#include <cstdint>

namespace chassis
{
	// 明确采用右手系：x前、y左、z上，逆时针yaw为正；与参考代码y右约定不同。
	// 下列模式/坐标系枚举是应用层接口定义；WheelVelocityPlanner 不负责模式切换，
	// 也不会自动执行云台跟随或坐标旋转，调用方须先生成底盘坐标系中的 vx/vy/yaw。
	enum class Mode : uint8_t
	{
		Disabled,     //失能
		NoFollow,     //不跟随
		FollowGimbal, //跟随
		Spin,         //小陀螺
		Direct
	};

	enum class chassis_mode : uint8_t
	{
		CHASSIS_NO_FOLLOW,         // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
		CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随云台,不单独设置pid,以误差角度平方为速度输出
		CHASSIS_ROTATE,            //自旋,同时保持全向机动;当前wz维持定值,todo后续增加不规则的变速策略
		poweroff
	};

	enum class CommandFrame : uint8_t
	{
		Chassis,
		Gimbal
	};

	inline constexpr unsigned kWheelCount = 4U; // 固定轮序：左前FL、右前FR、左后BL、右后BR。
	/** @brief 单个全向轮的固定机械参数，不是机器人在场地中的实时位置。 */
	struct WheelGeometry
	{
		float x_m = {};               //轮心相对底盘运动学中心的前后坐标
		float y_m = {};               // 轮心相对底盘运动学中心的左右坐标，单位m；前正、左正。
		float drive_direction_deg{};  // 逻辑正轮速对应的主动滚动方向相对+x的角度，逆时针为正。
		// 左前方向为+45°，但左前轮不一定填45°；须结合安装及电机reverse配置。
		float radius_m{};             // 实际车轮有效滚动半径，单位m，不是电机转子或齿轮的半径。
		float max_output_speed_dps{}; // 本轮输出轴最大角速度绝对值，单位°/s；转子速度换算在驱动边界进行。
	};

	/** @brief 车体运动能力限制；加速度限制也用于减速和反向，不包含电功率限制。 */
	struct MotionLimits
	{
		float max_translation_mps{};           // 平移合速度sqrt(vx²+vy²)上限，m/s；不是每个轴各自的上限。
		float max_yaw_dps{};                   // 车体偏航角速度绝对值上限，°/s；不是yaw电机转速。
		float translation_acceleration_mps2{}; // 平面速度矢量每秒最大变化量，m/s²。
		float yaw_acceleration_dps2{};         // 车体偏航角速度每秒最大变化量，°/s²。
	};

	// 以下周期换算函数目前被注释，保持调用方现状，本次不恢复或修改其逻辑。
	// 原设计：seconds为秒、frequency为每秒tick数，ticks通过引用返回整数调度周期。
	// seconds*frequency应接近整数；0.001 tick容差用于接受浮点表示误差，
	// 不是允许把任意小数周期四舍五入。上界2^31用于保留无符号tick差的半周期判定范围。
	inline bool ControlPeriodTicks(float seconds, uint32_t frequency, uint32_t& ticks)
	{
		ticks              = 0U;
		const double value = static_cast<double>(seconds) * frequency;
		if (!std::isfinite(value) || value < 0.5 || value >= 2147483648.0)
			return false;
		const double rounded = std::round(value);
		if (std::fabs(value - rounded) > 0.001)
			return false;
		ticks = static_cast<uint32_t>(rounded);
		return true;
	}

	/**
	 * @brief 将车体三自由度目标速度转换为四个全向轮的输出轴目标角速度。
	 * @note 只进行目标规划，不读取编码器、不计算PI、不发送CAN，也不判断遥控或急停。
	 *       处理顺序：车体速度限幅 -> 车体加速度斜坡 -> 全向轮逆解 -> 四轮同比例限幅。
	 *       斜坡只在车体层执行一次，避免叠加逐轮斜坡增加延迟或改变运动比例。
	 */
	class WheelVelocityPlanner
	{
		public:
			/**
			 * @brief 校验并缓存机械参数、运动限制及运动学矩阵，通常只在初始化时调用。
			 * @param geometry 按FL/FR/BL/BR排列的四轮参数；坐标m、方向deg、轮速deg/s。
			 * @param limits 车体速度/加速度上限；所有上限必须为有限正数。
			 * @return 全部参数和三自由度几何校验通过返回true，否则保持未配置状态。
			 * @note 这里只检查参数的数学有效性，无法确认实物尺寸、正转方向或轮胎打滑情况。
			 */
			bool Configure(const std::array<WheelGeometry, kWheelCount>& geometry, const MotionLimits& limits)
			{
				configured_ = false; // 先撤销旧配置的有效标志，避免本次配置失败后仍按旧配置输出。
				Reset();             // 清零车体速度斜坡的历史目标，不是清除机械参数或电机PI。
				float reference = 0.0F;
				for (const auto& [x_m, y_m, drive_direction_deg, radius_m, max_output_speed_dps] : geometry)
				{
					//每个轮子安装位置到机器人底盘坐标原点的距离，
					reference = std::fmax(reference, std::hypot(x_m, y_m));
					//最终取所有轮子中最大的安装半径
				}
				if (!std::isfinite(reference) || reference <= 0.0F) return false; //轮子几何参数校验
				// Gram矩阵G=A^T*A，初始全零；A的每一行对应一个轮子的运动学约束。
				// 这里仅用G检查几何是否能表达三自由度，不是在求动力学模型或控制增益。
				float gram[3][3]{};
				for (unsigned i = 0; i < kWheelCount; ++i) //遍历所有轮子
				{
					const auto& w      = geometry[i]; //取出当前轮子的几何参数

					//把轮子的安装角度转化为二维平面上的单位方向向量（X和Y的分量）
					const float theta  = std::remainder(w.drive_direction_deg, 360.0F) * kDegToRad;
					const float tx     = std::cos(theta), ty = std::sin(theta);
					//得到轮子的单位驱动方向
					// 轮心速度=(vx-omega*y, vy+omega*x)，沿(tx,ty)投影后，
					// 旋转项系数为(-y*tx+x*ty)，单位m；这是速度耦合系数，不是实际转矩。
					const float moment = -w.y_m * tx + w.x_m * ty;
					if (!std::isfinite(moment)) return false;
					rows_[i] = {tx, ty, moment}; // 缓存原始投影系数，Update直接复用，避免周期内重复三角计算。
					// 前两项无量纲，moment为长度；除以最大安装半径使三列尺度接近，
					// 减少几何尺寸对行列式阈值的影响。仅用于检查，实际轮速计算仍用原始moment。
					const float normalized[]{tx, ty, moment / reference};
					// 单轮行向量a的外积a^T*a累加到G；r、c是三自由度列的索引，不是轮子编号。
					for (unsigned r = 0; r < 3; ++r)
						for (unsigned c = 0; c < 3; ++c) gram[r][c] += normalized[r] * normalized[c];
				}
				//构建运动学矩阵与Gram矩阵
				// 三自由度必须满秩：A三列线性独立，G的行列式才为正。
				// 退化例子：所有轮子的驱动方向都平行，无法独立产生任意方向的平移。
				// 1e-5是数值判定阈值，也会拒绝部分接近退化的机构，并非精确的条件数评估。
				const float det = gram[0][0] * (gram[1][1] * gram[2][2] - gram[1][2] * gram[2][1]) -
								  gram[0][1] * (gram[1][0] * gram[2][2] - gram[1][2] * gram[2][0]) +
								  gram[0][2] * (gram[1][0] * gram[2][1] - gram[1][1] * gram[2][0]);
				if (!std::isfinite(det) || det <= 1e-5F) return false;
				geometry_   = geometry; // 值拷贝保存机械配置，不依赖调用方变量的生命周期。
				limits_     = limits;   // 保存后续Update需要的速度和加速度上限。
				configured_ = true;     // 全部检查通过后才允许Update生成目标。
				return true;
			}

			/** @brief 清零斜坡目标，使下次从静止起步；保留已验证的几何配置，不直接停硬件。 */
			void Reset()
			{
				vx_ = vy_ = yaw_ = 0.0F;
			}

			/**
			 * @brief 按本周期实际时间计算四轮目标；调用前须成功Configure。
			 * @param vx 前向目标速度，m/s，前正后负。
			 * @param vy 左向目标速度，m/s，左正右负。
			 * @param yaw 车体目标偏航角速度，°/s，俯视逆时针为正，不是目标角度。
			 * @param dt 距上次更新的实际秒数；任务层负责检查周期是否超时。
			 * @param output 返回FL/FR/BL/BR的输出轴目标轮速，°/s；错误时整组清零。
			 * @return true只表示目标计算成功，不代表电机已使能或CAN发送成功。
			 */
			bool Update(float vx, float vy, float yaw, float dt, std::array<float, kWheelCount>& output)
			{
				output = {}; // 先清除调用方数组内上周期的目标，失败路径不能残留旧指令。
				// 未配置、NaN/无穷大或非正dt都不能进入规划；Fail会清输出并复位斜坡。
				if (!configured_ || !std::isfinite(vx) || !std::isfinite(vy) ||
					!std::isfinite(yaw) || !std::isfinite(dt) || dt <= 0.0F)
					return Fail(output);
				const float norm = std::hypot(vx, vy); // 平移合速度sqrt(vx²+vy²)，单位m/s。
				if (!std::isfinite(norm)) return Fail(output);
				if (norm > limits_.max_translation_mps)
				{
					// vx/vy同时按相同比例缩小，保留操作者要求的平移方向。
					// 若分别截断两个分量，斜向运动的方向和合速度可能改变。
					vx *= limits_.max_translation_mps / norm;
					vy *= limits_.max_translation_mps / norm;
				}
				yaw                 = Limit(yaw, limits_.max_yaw_dps);            // 独立限制车体旋转速度，保留左右转符号。
				const float delta   = std::hypot(vx - vx_, vy - vy_);             // 新目标相对上次已执行目标的速度矢量差，m/s。
				const float step    = limits_.translation_acceleration_mps2 * dt; // 本周期允许的平移速度变化量，m/s。
				const float yawStep = limits_.yaw_acceleration_dps2 * dt;         // 本周期允许的偏航速度变化量，°/s。
				// 输入虽有限，乘法/减法仍可能溢出；中间值异常时立即撤销本次目标。
				if (!std::isfinite(delta) || !std::isfinite(step) || !std::isfinite(yawStep)) return Fail(output);
				if (delta > step)
				{
					// 沿“旧速度矢量 -> 新速度矢量”的方向只走step，等比例处理x/y变化量。
					// 对加速、减速、转向都生效；不是只限制速度绝对值增大时的加速度。
					vx = vx_ + (vx - vx_) * step / delta;
					vy = vy_ + (vy - vy_) * step / delta;
				}
				yaw         = yaw_ + Limit(yaw - yaw_, yawStep); // 旋转斜坡：在上次目标上叠加受限的变化量。
				float scale = 1.0F;                              // 四轮共用缩放系数，1表示当前无需进一步限速。
				for (unsigned i = 0; i < kWheelCount; ++i)
				{
					const auto& row = rows_[i];
					// 分子：tx*vx + ty*vy + moment*omega，是轮心速度沿主动方向的投影，单位m/s。
					// yaw*kDegToRad将车体°/s转为rad/s；除以轮半径得到轮轴rad/s，
					// 再除以kDegToRad得到轮轴°/s，因此分母合并为radius_m*kDegToRad。
					// 这里是全向轮投影模型，不是直接套用麦克纳姆轮的符号公式。
					output[i]       = (row[0] * vx + row[1] * vy + row[2] * yaw * kDegToRad) /
									  (geometry_[i].radius_m * kDegToRad);
					if (!std::isfinite(output[i])) return Fail(output);
					const float magnitude = std::fabs(output[i]);
					if (magnitude > geometry_[i].max_output_speed_dps)
						// 每个超速轮给出一个可接受比例，最终取最小值满足所有轮子的不同上限。
						scale = std::fmin(scale, geometry_[i].max_output_speed_dps / magnitude);
				}
				// 即使只有一轮超速，也同时缩放四轮，保留本周期要求的平移/旋转运动比例。
				// 这是轮速限制，不等同于电流或比赛功率限制，也可能使斜坡变化受硬限幅影响。
				for (float& speed : output) speed *= scale;
				// 保存限幅后的可执行车体目标，而非原始摇杆目标；它们不是编码器测量速度。
				// 下一周期斜坡从这里继续，避免内部目标一直超限、松杆后仍追逐过大的旧目标。
				vx_  = vx * scale;
				vy_  = vy * scale;
				yaw_ = yaw * scale;
				return true;
			}

		private:
			static constexpr float kDegToRad = 0.017453292519943295F; // pi/180，度到弧度的换算系数。
			// 对称限幅到[-limit,+limit]；limit来自已校验的上限或本周期允许变化量。
			static float           Limit(float v, float limit)
			{
				return std::fmax(-limit, std::fmin(limit, v));
			}

			// 统一失败出口：清目标和斜坡，但保留已配置参数；硬件禁用/零电流仍由Task执行。
			bool Fail(std::array<float, kWheelCount>& output)
			{
				output = {};
				Reset();
				return false;
			}

			std::array<WheelGeometry, kWheelCount>        geometry_{};          // 四轮机械参数的本地副本。
			std::array<std::array<float, 3>, kWheelCount> rows_{};              // 各轮[tx,ty,moment]，初始化时计算。
			MotionLimits                                  limits_{};            // 当前有效的车体运动限制。
			float                                         vx_{}, vy_{}, yaw_{}; // 上周期限幅后的目标：m/s、m/s、°/s。
			bool                                          configured_{};        // 成功配置后为true；Reset不会将其清零。
	};
}

#endif //INFANTRY_H723_CHASSIS_CONTROL_H
