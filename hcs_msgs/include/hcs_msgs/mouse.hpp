#pragma once

#include <bit>
#include <cstdint>

namespace hcs_msgs {

struct __attribute__((packed)) Mouse {
    constexpr static inline Mouse zero() {
        constexpr uint8_t zero = 0;
        return std::bit_cast<Mouse>(zero);
    }

    bool left  : 1;
    bool right : 1;
};

static_assert(sizeof(Mouse) == sizeof(uint8_t));

} // namespace hcs_msgs
