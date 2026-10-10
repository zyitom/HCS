#pragma once

#include <cstddef>

namespace hcs_utility {

/// 一条 cache line 的字节数。
///
/// 为什么写死 64 而不用 std::hardware_destructive_interference_size：
/// GCC 把后者当成 ABI 敏感常量，头文件里一用就吃 -Winterference-size（本工作区要求零警告），
/// 而且它的取值会随 -mtune 变——一旦被烤进跨翻译单元共享的类布局里，就是一颗 ODR 地雷。
/// 目标平台（x86-64 / aarch64）的 L1 行都是 64 字节，写死反而更可控。
inline constexpr std::size_t kCacheLine = 64;

/// 让一个值独占整条 cache line，用来隔开被不同线程写的相邻字段。
///
/// 伪共享的代价不在"读到旧值"（不会），而在每次对端写入都让本端的行失效：
/// 一个本该常驻 L1 的原子变量会退化成每次都走 L3 / 跨核嗅探。
template <typename T>
struct alignas(kCacheLine) CacheAligned {
    T value;
};

} // namespace hcs_utility
