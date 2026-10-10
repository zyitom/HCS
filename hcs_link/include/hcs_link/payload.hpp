#pragma once

#include <bit>
#include <concepts>
#include <cstdint>
#include <string_view>
#include <type_traits>

namespace hcs_link {

// 共享内存的另一头是另一个编译单元、可能是另一个编译器；两边只在这几条上达成一致。
static_assert(std::endian::native == std::endian::little, "hcs_link assumes little endian");
static_assert(sizeof(void*) == 8, "hcs_link assumes LP64");

/// FNV-1a，给载荷名字算身份。
[[nodiscard]] constexpr std::uint64_t fnv1a(std::string_view text) noexcept {
    std::uint64_t hash = 0xcbf29ce484222325ULL;
    for (const char c : text) {
        hash ^= static_cast<unsigned char>(c);
        hash *= 0x100000001b3ULL;
    }
    return hash;
}

/// 能放进通道的类型。
///
/// - 可平凡复制、标准布局：按字节搬过去就是同一个值；
/// - 大小是 8 的倍数、对齐不超过 8：整条按 64 位字逐个原子搬运，不留尾巴；
/// - 带 kName / kVersion：两端按名字和版本认人，不一致就拒绝接上。
///
/// 另外两条靠载荷自己守（概念表达不了）：不放指针；不用 bool、enum
/// （对端写进来的非法取值按 bool / enum 读是 UB，一律用定宽整数）。
/// 每个载荷紧跟一条 sizeof 的 static_assert，改布局必须同时改版本号。
template <typename T>
concept Payload = std::is_trivially_copyable_v<T> && std::is_standard_layout_v<T>
               && sizeof(T) > 0 && sizeof(T) % sizeof(std::uint64_t) == 0
               && alignof(T) <= alignof(std::uint64_t) && requires {
                      { T::kName } -> std::convertible_to<std::string_view>;
                      { T::kVersion } -> std::convertible_to<std::uint32_t>;
                  };

template <Payload T>
inline constexpr std::uint64_t payload_id = fnv1a(T::kName);

} // namespace hcs_link
