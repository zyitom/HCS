#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "rmcs_sync/detail/cache_line.hpp"

namespace rmcs_sync {

/// 隧道化的字节流：结构与 EventQueue 同构（两条 cache line + 各自缓存对端游标），
/// 元素类型固定为 std::byte，回绕用两段 memcpy 处理。
///
/// **不解析任何协议**——解析要在拿到字节的那个域里做，隧道只负责把字节原样搬过去。
template <std::size_t Capacity>
requires (Capacity >= 2 && std::has_single_bit(Capacity))
class ByteTunnel {
public:
    static constexpr std::size_t capacity = Capacity;

    ByteTunnel() noexcept = default;
    ByteTunnel(const ByteTunnel&) = delete;
    ByteTunnel& operator=(const ByteTunnel&) = delete;

    /// 生产者侧。写入尽可能多的字节；写不下的部分计入 overrun_bytes 并丢弃。
    /// @return 实际写入的字节数
    std::size_t write(std::span<const std::byte> data) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        std::size_t free_space = Capacity - (head - cached_tail_);
        if (free_space < data.size()) {
            // 只有缓存说“写不下”时才去碰消费者那条 line。
            cached_tail_ = tail_.load(std::memory_order_acquire);
            free_space = Capacity - (head - cached_tail_);
        }

        const std::size_t count = std::min(free_space, data.size());
        if (count < data.size())
            overrun_bytes_.fetch_add(
                static_cast<std::uint64_t>(data.size() - count), std::memory_order_relaxed);
        if (count == 0)
            return 0;

        const std::size_t offset = head & kIndexMask;
        const std::size_t first = std::min(count, Capacity - offset);
        std::memcpy(storage_ + offset, data.data(), first);
        if (count > first)
            std::memcpy(storage_, data.data() + first, count - first);

        head_.store(head + count, std::memory_order_release);
        return count;
    }

    /// 消费者侧。@return 实际读出的字节数
    std::size_t read(std::span<std::byte> out) noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        std::size_t available = cached_head_ - tail;
        if (available < out.size()) {
            cached_head_ = head_.load(std::memory_order_acquire);
            available = cached_head_ - tail;
        }

        const std::size_t count = std::min(available, out.size());
        if (count == 0)
            return 0;

        const std::size_t offset = tail & kIndexMask;
        const std::size_t first = std::min(count, Capacity - offset);
        std::memcpy(out.data(), storage_ + offset, first);
        if (count > first)
            std::memcpy(out.data() + first, storage_, count - first);

        tail_.store(tail + count, std::memory_order_release);
        return count;
    }

    [[nodiscard]] std::uint64_t overrun_bytes() const noexcept {
        return overrun_bytes_.load(std::memory_order_relaxed);
    }

    /// 消费者侧查询：tail 归它私有，所以结果精确（对端可能又多写了，只会偏小）。
    [[nodiscard]] std::size_t readable() const noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        return head_.load(std::memory_order_acquire) - tail;
    }

    /// 生产者侧查询：head 归它私有，所以结果精确（对端可能又多读了，只会偏小）。
    [[nodiscard]] std::size_t writable() const noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        return Capacity - (head - tail_.load(std::memory_order_acquire));
    }

private:
    static constexpr std::size_t kIndexMask = Capacity - 1;

    alignas(detail::kCacheLine) std::atomic<std::size_t> head_{0}; // 生产者写
    std::size_t cached_tail_ = 0;                                  // 生产者私有
    alignas(detail::kCacheLine) std::atomic<std::size_t> tail_{0}; // 消费者写
    std::size_t cached_head_ = 0;                                  // 消费者私有
    alignas(detail::kCacheLine) std::atomic<std::uint64_t> overrun_bytes_{0};
    std::byte storage_[Capacity];
};

} // namespace rmcs_sync
