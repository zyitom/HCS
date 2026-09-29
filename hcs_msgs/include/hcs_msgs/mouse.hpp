#pragma once

#include <bit>
#include <cstdint>

namespace hcs_msgs {

// 位域依赖小端主机 + GCC/Clang LSB-first（left=bit0）。
static_assert(std::endian::native == std::endian::little, "bitmap assumes a LE host");

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
