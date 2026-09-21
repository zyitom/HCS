#pragma once

#include <cstdint>

namespace hcs_msgs {

/// 三挡开关。UNKNOWN 对应协议里的失联/非法值。
enum class Switch : uint8_t { UNKNOWN = 0, UP = 1, DOWN = 2, MIDDLE = 3 };

} // namespace hcs_msgs
