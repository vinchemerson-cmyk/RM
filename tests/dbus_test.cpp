/** @file dbus_test.cpp @brief DBUS 位布局、键鼠、残帧及失步重同步回归测试。 */
#include <cassert>
#include <array>
#include <algorithm>
#include <iostream>
#include "rc/dbus.h"

int main()
{
	const uint16_t          ch[4] = {364U, 1024U, 1684U, 1100U};
	std::array<uint8_t, 18> frame{};
	frame[0]  = ch[0];
	frame[1]  = (ch[0] >> 8U) | (ch[1] << 3U);
	frame[2]  = (ch[1] >> 5U) | (ch[2] << 6U);
	frame[3]  = ch[2] >> 2U;
	frame[4]  = (ch[2] >> 10U) | (ch[3] << 1U);
	frame[5]  = (ch[3] >> 7U) | (2U << 6U) | (3U << 4U);
	frame[6]  = 0xFEU;
	frame[7]  = 0xFFU; // mouse.x=-2
	frame[8]  = 0x2CU;
	frame[9]  = 0x01U; // mouse.y=300
	frame[10] = 0xFFU;
	frame[11] = 0xFFU; // mouse.z=-1
	frame[12] = 1U;
	frame[14] = 0xF1U;
	frame[15] = 0x80U; // W/Q/E/Shift/Ctrl + high bit
	frame[16] = 0x34U;
	frame[17] = 0x12U;
	rc::RC_Data_s result{};
	assert(rc::DecodeDbusFrame(frame.data(), frame.size(), result));
	for (size_t i = 0; i < 4U; ++i) assert(result.channels[i] == ch[i]);
	assert(result.switch_left == rc::Left_Switch_e::DOWN);
	assert(result.switch_right == rc::Right_Switch_e::MIDDLE);
	assert(result.mouse.x == -2 && result.mouse.y == 300 && result.mouse.z == -1);
	assert(result.mouse.left && !result.mouse.right);
	assert(result.keyboard.w && result.keyboard.q && result.keyboard.e);
	assert(result.keyboard.shift && result.keyboard.ctrl && !result.keyboard.s);
	assert(result.keyboard_mouse_raw == 0x80F1U && result.wheel_raw == 0x1234U);
	assert(!rc::DecodeDbusFrame(frame.data(), 17U, result));
	auto invalid = frame;
	invalid[5]   &= 0x3FU;
	assert(!rc::DecodeDbusFrame(invalid.data(), invalid.size(), result));
	invalid     = frame;
	invalid[12] = 2U;
	assert(!rc::DecodeDbusFrame(invalid.data(), invalid.size(), result));
	invalid.fill(0U);
	assert(!rc::DecodeDbusFrame(invalid.data(), invalid.size(), result));
	assert(result.channels[0] == ch[0]);
	rc::DbusStreamParser parser;
	assert(parser.Push(frame.data(), 7U, result) == 0U);
	assert(parser.Push(frame.data() + 7U, 11U, result) == 1U);
	std::array<uint8_t, 36> two{};
	std::copy(frame.begin(), frame.end(), two.begin());
	std::copy(frame.begin(), frame.end(), two.begin() + 18U);
	assert(parser.Push(two.data(), two.size(), result) == 2U);
	const uint8_t noise[3] = {0xFFU, 0xFFU, 0xFFU};
	assert(parser.Push(noise, sizeof(noise), result) == 0U);
	assert(parser.Push(frame.data(), frame.size(), result) == 1U);
	parser.Push(frame.data(), 8U, result);
	parser.Reset();
	assert(parser.Push(frame.data(), frame.size(), result) == 1U);
	std::cout << "DBUS decode, invalid frames, fragments and resync: PASS\n";
}
