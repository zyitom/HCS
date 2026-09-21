// 对拍 harness 专用 stub：RC_ctrl_t（仅 v2 的 FSM 签名需要；NO_HEAD 路径不编译）。
#pragma once

#include <cstdint>

namespace module {

struct RC_ctrl_t {
    int16_t rocker_l_{};
    int16_t rocker_l1{};
    int16_t rocker_r_{};
    int16_t rocker_r1{};
    int16_t dial{};
    uint8_t switch_left = 2;
    uint8_t switch_right = 2;
    bool fn{};
    bool button{};
};

} // namespace module
