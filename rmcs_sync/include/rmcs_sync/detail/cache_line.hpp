#pragma once

#include <cstddef>

namespace rmcs_sync::detail {

/// 缓存行大小。写死 64 而不用 std::hardware_destructive_interference_size：
/// 后者在 GCC 上会触发 -Winterference-size（它的值参与 ABI，跨 -mtune 不稳定），
/// 而本工程只在 x86-64 / aarch64 上跑，两者的 L1 行都是 64 字节。
inline constexpr std::size_t kCacheLine = 64;

} // namespace rmcs_sync::detail
