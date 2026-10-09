/**
 * @file encoder_angle.h
 * @brief 将 DJI 13 位单圈编码器相邻采样转换为有符号增量。
 */

#ifndef INFANTRY_H723_ENCODER_ANGLE_H
#define INFANTRY_H723_ENCODER_ANGLE_H

#include <cstdint>

/**
 * @brief 计算跨越 0/8191 边界时的最短编码器增量。
 * @param previous 上次 13 位编码器读数。
 * @param current 当前 13 位编码器读数。
 * @return [-4095, 4096] 范围内的计数差。
 * @note 两次有效反馈之间转动必须小于半圈，否则无法判定真实转数。
 */
inline int16_t MotorEncoderDelta(uint16_t previous, uint16_t current)
{
	int32_t delta = static_cast<int32_t>(current & 0x1FFFU) -
					static_cast<int32_t>(previous & 0x1FFFU);
	if (delta > 4096)
		delta -= 8192;
	else if (delta < -4096)
		delta += 8192;
	return static_cast<int16_t>(delta);
}

#endif // INFANTRY_H723_ENCODER_ANGLE_H
