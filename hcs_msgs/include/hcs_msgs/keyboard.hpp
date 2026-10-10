#pragma once

#include <bit>
#include <cstdint>

namespace hcs_msgs {

/// 16 个键盘按键位图，布局与 DJI 图传/DR16 键盘帧一致。
// 位域依赖小端主机 + GCC/Clang LSB-first（W 落 bit0）。
static_assert(std::endian::native == std::endian::little, "bitmap assumes a LE host");
struct __attribute__((packed)) Keyboard {
    constexpr static inline Keyboard zero() {
        constexpr uint16_t zero = 0;
        return std::bit_cast<Keyboard>(zero);
    }

    bool w     : 1;
    bool s     : 1;
    bool a     : 1;
    bool d     : 1;
    bool shift : 1;
    bool ctrl  : 1;
    bool q     : 1;
    bool e     : 1;
    bool r     : 1;
    bool f     : 1;
    bool g     : 1;
    bool z     : 1;
    bool x     : 1;
    bool c     : 1;
    bool v     : 1;
    bool b     : 1;
};

static_assert(sizeof(Keyboard) == sizeof(uint16_t));

} // namespace hcs_msgs
