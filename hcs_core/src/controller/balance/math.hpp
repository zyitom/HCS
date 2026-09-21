#pragma once

// Helios module::user_lib 的纯数学工具，语义逐一对应（float、相同比较顺序）。

#include <cmath>

namespace hcs_core::controller::balance {

inline constexpr float kPi = 3.14159265358979f;
inline constexpr float kPi2 = kPi * 2.0f;

[[nodiscard]] constexpr float deg2rad(float deg) {
    return deg * 0.0174532925f;
}

template <typename T, typename U, typename V>
[[nodiscard]] constexpr auto clip(const T value, const U min, const V max) -> decltype(value) {
    return (value > max) ? max : ((value < min) ? min : value);
}

template <typename T, typename U>
[[nodiscard]] constexpr auto abs_clip(const T value, const U max) -> decltype(value) {
    return clip(value, -max, max);
}

template <typename T, typename U, typename V>
[[nodiscard]] constexpr auto loop_clip(T value, const U min_value, const V max_value)
    -> decltype(value) {
    T len = max_value - min_value;
    if (value > max_value) {
        value = min_value + std::fmod(value - min_value, len);
    } else if (value < min_value) {
        value = max_value - std::fmod(min_value - value, len);
    }
    return value;
}

[[nodiscard]] constexpr float rad_format(const float angle) {
    return loop_clip(angle, -kPi, kPi);
}

} // namespace hcs_core::controller::balance
