#pragma once

#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>

#include "hcs_base/cache_line.hpp"

namespace hcs_sync {

/// 周期域 → 事件域 / 尽力域的唯一通道：有界 SPSC 环。
///
/// 有界是故意的：周期域不许为了不丢事件而等消费者。满了就丢，丢弃是显式策略、
/// 只计数不打日志（打日志本身就会把周期域拖进 D3）。
template <typename T, std::size_t Capacity>
requires (Capacity >= 2 && std::has_single_bit(Capacity) && std::is_nothrow_destructible_v<T>)
class EventQueue {
public:
    static constexpr std::size_t capacity = Capacity;

    EventQueue() noexcept = default;

    /// 析构剩余元素：环里可能还压着没被消费的非平凡对象。
    ~EventQueue() {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        for (std::size_t index = tail_.load(std::memory_order_relaxed); index != head; ++index)
            std::destroy_at(slot(index));
    }

    EventQueue(const EventQueue&) = delete;
    EventQueue& operator=(const EventQueue&) = delete;

    /// 生产者侧。满则丢弃并 ++dropped_。noexcept。
    bool try_push(const T& value) noexcept requires std::is_nothrow_copy_constructible_v<T> {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        if (!reserve_slot(head))
            return false;
        ::new (raw_slot(head)) T(value);
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    bool try_push(T&& value) noexcept requires std::is_nothrow_move_constructible_v<T> {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        if (!reserve_slot(head))
            return false;
        ::new (raw_slot(head)) T(std::move(value));
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    /// 消费者侧。
    /// 取出用的是 T 的移动/拷贝赋值——它若会抛，本函数的 noexcept 就会变成 terminate，
    /// 所以放进队列的类型应当是无抛移动的。
    [[nodiscard]] bool try_pop(T& out) noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == cached_head_) {
            // 只有缓存说“空”时才去碰生产者那条 line。
            cached_head_ = head_.load(std::memory_order_acquire);
            if (tail == cached_head_)
                return false;
        }
        T* cell = slot(tail);
        out = std::move(*cell);
        std::destroy_at(cell);
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

    /// reporter 线程读。
    [[nodiscard]] std::uint64_t dropped() const noexcept {
        return dropped_.load(std::memory_order_relaxed);
    }

    /// 近似值：读的瞬间对端可能正在推进游标。
    [[nodiscard]] std::size_t size() const noexcept {
        // 先读 tail 再读 head：head 只增不减，这个次序保证差值不会下溢成天文数字。
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        return head_.load(std::memory_order_acquire) - tail;
    }

    [[nodiscard]] bool empty() const noexcept { return size() == 0; }

private:
    static constexpr std::size_t kIndexMask = Capacity - 1;

    [[nodiscard]] void* raw_slot(std::size_t index) noexcept {
        return static_cast<void*>(&storage_[sizeof(T) * (index & kIndexMask)]);
    }

    [[nodiscard]] T* slot(std::size_t index) noexcept {
        return std::launder(reinterpret_cast<T*>(raw_slot(index)));
    }

    /// 生产者侧判满：先信缓存，缓存说满了才去 load 一次真值。
    [[nodiscard]] bool reserve_slot(std::size_t head) noexcept {
        if (head - cached_tail_ >= Capacity) {
            cached_tail_ = tail_.load(std::memory_order_acquire);
            if (head - cached_tail_ >= Capacity) {
                dropped_.fetch_add(1, std::memory_order_relaxed);
                return false;
            }
        }
        return true;
    }

    // head_ 与 tail_ 各自独占一条 cache line，并各自缓存对端游标，
    // 让稳态下 push/pop 完全不碰对方的 line。
    alignas(hcs_utility::kCacheLine) std::atomic<std::size_t> head_{0}; // 生产者写
    std::size_t cached_tail_ = 0;                                  // 生产者私有
    alignas(hcs_utility::kCacheLine) std::atomic<std::size_t> tail_{0}; // 消费者写
    std::size_t cached_head_ = 0;                                  // 消费者私有
    alignas(hcs_utility::kCacheLine) std::atomic<std::uint64_t> dropped_{0};
    alignas(alignof(T)) std::byte storage_[sizeof(T) * Capacity];
};

} // namespace hcs_sync
