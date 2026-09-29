#pragma once

#include <cstddef>
#include <new>

namespace hcs_utility {

/// 定长裸存储:Size 字节、Align 对齐。不构造、不销毁 —— 那些归调用方
/// (placement new / std::destroy_at),这里只提供对齐正确的一块地方。
template <std::size_t Size, std::size_t Align>
struct RawBytes {
    alignas(Align) std::byte bytes[Size];
};

/// T 的裸存储,大小与对齐取自 T。
///
/// 两个访问器,职责严格分开:
///   - raw():placement new 之前用。存储里还没有 T 的生命周期,launder
///     一个未构造的存储在标准上是 UB,所以这一步**不**launder。
///   - ptr():构造之后用,launder 过的 T*。
/// RingBuffer / DoubleBuffer / MemoryPool / OutputInterface 的存储
/// 原来各自手写这套(存储结构 + launder 舞步),现在共用这一份。
template <typename T>
struct RawStorage : RawBytes<sizeof(T), alignof(T)> {
    T* raw() noexcept { return reinterpret_cast<T*>(this->bytes); }
    const T* raw() const noexcept { return reinterpret_cast<const T*>(this->bytes); }
    T* ptr() noexcept { return std::launder(raw()); }
    const T* ptr() const noexcept { return std::launder(raw()); }
};

} // namespace hcs_utility
