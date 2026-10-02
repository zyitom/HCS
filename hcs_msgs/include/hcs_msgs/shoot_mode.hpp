#pragma once

#include <cstdint>

namespace hcs_msgs {

enum class ShootMode : uint8_t {
    // 每点一下鼠标打一发。
    SINGLE = 0,

    // 按住鼠标连续发射。
    AUTOMATIC = 1,

    // 每点一下鼠标打一发，更准，代价是发射延迟更大。
    PRECISE = 2,

    // 每点一下鼠标打一发，输入延迟更小、响应更快，可能误发。
    LOW_LATENCY = 3,

    // 按住鼠标连续发射，不管热量上限，代价是扣血。
    OVERDRIVE = 4,
};

}