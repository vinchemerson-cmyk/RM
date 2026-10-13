/** @file clamp.h @brief 控制算法共用的浮点限幅；不承担类型转换或异常值处理。 */
#pragma once

namespace controlMath
{
    /** @brief 将 value 限制在闭区间内；调用方保证 minimum <= maximum。
     *  @note 保持原比较语义：NaN 原样返回，由调用方负责有效性检查。
     */
    constexpr float Clamp(float value, float minimum, float maximum)
    {
        return value < minimum ? minimum : (value > maximum ? maximum : value);
    }
}
