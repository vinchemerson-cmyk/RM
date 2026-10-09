/**
 * @file pid_test.cpp
 * @brief PIDInstance 的宿主机 (host) 单元测试, 断言失败即以非零码退出。
 *
 * 编译运行 (无 CMake 目标, 直接用 g++ 手动编译, 必须同时链接 pid.cpp):
 *   g++ -std=c++17 -I Modules tests/pid_test.cpp Modules/algorithm/pid.cpp \
 *       -o build/pid_test && ./build/pid_test
 *
 * 覆盖场景:
 *   1. 电机型 PID (双向输出): P/I/D 叠加数值、Reset() 清历史;
 *   2. 加热器型 PID (单向 0~1 输出): 正向饱和、误差反向后退饱和;
 *   3. 非法参数拒绝 (输出区间不含 0)、Disable() 后输出恒为 0、
 *      dt<=0 与 NaN 输入的失败安全行为。
 */
#include "algorithm/pid.h"

#include <cassert>
#include <cmath>

static bool Near(float a, float b)
{
	return std::fabs(a - b) < 0.0001F;
}

int main()
{
	PIDInstance             motor;
	const PID_Init_Config_s motorConfig{2.0F, 1.0F, 0.5F, -10.0F, 10.0F, -5.0F, 5.0F};
	assert(motor.Configure(motorConfig));
	assert(Near(motor.Update(2.0F, 0.0F, 1.0F), 6.0F));
	assert(Near(motor.Update(2.0F, 1.0F, 1.0F), 4.5F));
	motor.Reset();
	assert(Near(motor.Update(2.0F, 0.0F, 1.0F), 6.0F));

	PIDInstance             heater;
	const PID_Init_Config_s heaterConfig{0.1F, 0.2F, 0.0F, 0.0F, 1.0F, 0.0F, 1.0F};
	assert(heater.Configure(heaterConfig));
	assert(Near(heater.Update(50.0F, 20.0F, 1.0F), 1.0F));
	assert(Near(heater.Update(50.0F, 49.0F, 1.0F), 0.3F));
	assert(Near(heater.Update(50.0F, 55.0F, 1.0F), 0.0F));
	heater.Disable();
	assert(!heater.IsConfigured());
	assert(Near(heater.Update(50.0F, 20.0F, 1.0F), 0.0F));

	PIDInstance invalid;
	assert(!invalid.Configure(PID_Init_Config_s{1.0F, 0.0F, 0.0F, 1.0F, 2.0F, 0.0F, 1.0F}));
	assert(invalid.Configure(motorConfig));
	assert(Near(invalid.Update(1.0F, 0.0F, 0.0F), 0.0F));
	assert(Near(invalid.Update(1.0F, 0.0F, 1.0F), 3.0F));
	assert(Near(invalid.Update(NAN, 0.0F, 1.0F), 0.0F));
}
