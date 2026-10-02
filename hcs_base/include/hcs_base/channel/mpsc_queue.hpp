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

/// 多个生产者 → 一个消费者的有界队列。任何域的线程都可以往里放，只有一条线程往外取。
///
/// 和 EventQueue 的分工：EventQueue 是 SPSC，生产者一侧是 wait-free 的，一条通道只接
/// 一个写者时用它。这里是给"写者不止一个、而且事先数不清"的场合用的——日志就是：
/// 控制线程、各板的 IO 线程、发送线程、spin 线程都会写。
///
/// 做法是每格带一个序号的定长数组（D. Vyukov 的 bounded queue，消费侧收窄成单消费者）：
///
///   - 格子的序号 == 入队游标：这一格空着，轮到它被写。生产者 CAS 抢游标，抢到的人写。
///   - 格子的序号 == 出队游标 + 1：这一格写完了，可以取。
///   - 取走之后把序号推到"下一圈轮到它"的值，这一格重新可写。
///
/// 对生产者的保证，也是它能放进周期域的理由：
///   - **不阻塞、不进内核、不分配。** 满了立刻返回 false 并计数，绝不等消费者。
///   - lock-free，但不是 wait-free：两个生产者同时抢同一格，输的那个要重试一次 CAS。
///     重试次数以"同时在抢的生产者数"为界，没有无界的自旋。
///
/// 一个要知道的性质：生产者在"抢到格子"和"写完格子"之间被抢占的话，消费者会停在这一格
/// 等它（看上去像队列空了），排在后面的元素要等它回来才出得去。**没有任何人因此被阻塞**
/// ——别的生产者照常入队，消费者照常返回——只是输出晚一点。对日志这是可以接受的。
template <typename T, std::size_t Capacity>
requires(
    Capacity >= 2 && std::has_single_bit(Capacity) && std::is_nothrow_move_constructible_v<T>
    && std::is_nothrow_destructible_v<T>)
class MpscQueue {
public:
    static constexpr std::size_t capacity = Capacity;

    MpscQueue() noexcept {
        for (std::size_t index = 0; index < Capacity; ++index)
            cells_[index].sequence.store(index, std::memory_order_relaxed);
    }

    /// 析构剩余元素。此时已经没有别的线程在碰它。
    ~MpscQueue() {
        for (Cell* cell = front(); cell != nullptr; cell = front()) {
            std::destroy_at(cell->object());
            ++dequeue_position_;
        }
    }

    MpscQueue(const MpscQueue&) = delete;
    MpscQueue& operator=(const MpscQueue&) = delete;

    /// 生产者侧，任意多条线程。满则丢弃并 ++dropped。
    template <typename... Args>
    requires std::is_nothrow_constructible_v<T, Args...>
    bool try_emplace(Args&&... args) noexcept {
        std::size_t position = enqueue_position_.load(std::memory_order_relaxed);
        Cell* cell = nullptr;
        for (;;) {
            cell = &cells_[position & kIndexMask];
            // acquire：配消费者取走之后那次 release。看到"轮到这一格"时，
            // 消费者对它的析构必然已经完成，这块存储可以重新构造。
            const std::size_t sequence = cell->sequence.load(std::memory_order_acquire);
            const auto lag = static_cast<std::ptrdiff_t>(sequence - position);

            if (lag == 0) {
                // 游标之间不需要定序：谁抢到哪一格由这次 CAS 决定，数据的可见性由格子
                // 自己的序号（下面那次 release）承担。失败时 position 被更新成最新值。
                if (enqueue_position_.compare_exchange_weak(
                        position, position + 1, std::memory_order_relaxed))
                    break;
            } else if (lag < 0) {
                // 这一格还压着上一圈没被取走的元素：满了。
                dropped_.fetch_add(1, std::memory_order_relaxed);
                return false;
            } else {
                // 别的生产者已经把游标推过去了，追上去。
                position = enqueue_position_.load(std::memory_order_relaxed);
            }
        }

        ::new (static_cast<void*>(cell->storage)) T(std::forward<Args>(args)...);
        // release：把上面整个对象的构造钉在"这一格可取"之前。
        cell->sequence.store(position + 1, std::memory_order_release);
        return true;
    }

    bool try_push(const T& value) noexcept requires std::is_nothrow_copy_constructible_v<T> {
        return try_emplace(value);
    }

    bool try_push(T&& value) noexcept { return try_emplace(std::move(value)); }

    /// 消费者侧，**只能有一条线程**。空（或者队首那一格还没写完）返回 false，不动 out。
    [[nodiscard]] bool try_pop(T& out) noexcept requires std::is_nothrow_move_assignable_v<T> {
        Cell* cell = front();
        if (cell == nullptr)
            return false;

        out = std::move(*cell->object());
        std::destroy_at(cell->object());
        // release：把我们对这一格的最后一次访问钉在"它重新可写"之前。
        cell->sequence.store(dequeue_position_ + Capacity, std::memory_order_release);
        ++dequeue_position_;
        return true;
    }

    /// 因为满而被拒绝的次数。任何线程可读，只给诊断用。
    [[nodiscard]] std::uint64_t dropped() const noexcept {
        return dropped_.load(std::memory_order_relaxed);
    }

private:
    static constexpr std::size_t kIndexMask = Capacity - 1;

    struct Cell {
        std::atomic<std::size_t> sequence;
        alignas(T) std::byte storage[sizeof(T)];

        [[nodiscard]] T* object() noexcept {
            return std::launder(reinterpret_cast<T*>(storage));
        }
    };

    /// 队首那一格，写完了才给；否则 nullptr。只在消费者线程（和析构）里调。
    [[nodiscard]] Cell* front() noexcept {
        Cell& cell = cells_[dequeue_position_ & kIndexMask];
        // acquire：配生产者构造完之后那次 release。
        if (cell.sequence.load(std::memory_order_acquire) != dequeue_position_ + 1)
            return nullptr;
        return &cell;
    }

    // 布局同 Snapshot：一条 cache line 只放一类写者。
    alignas(hcs_utility::kCacheLine) std::atomic<std::size_t> enqueue_position_{0}; // 生产者们
    alignas(hcs_utility::kCacheLine) std::size_t dequeue_position_ = 0;             // 消费者私有
    alignas(hcs_utility::kCacheLine) std::atomic<std::uint64_t> dropped_{0};
    alignas(hcs_utility::kCacheLine) Cell cells_[Capacity];
};

} // namespace hcs_sync
