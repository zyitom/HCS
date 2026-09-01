#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

#include "rmcs_utility/atomic_futex.hpp"

namespace rmcs_utility {

/**
 * @brief 单振铃者 / 单等待者的门铃。
 *
 * 存在的理由是**周期域拍尾那一次唤醒**：控制线程算完命令、写进 `Snapshot` 之后，
 * 要把发送线程叫起来。这一下夹在两个互相拉扯的要求之间：
 *
 *   - **不能让控制线程等。** 所以不能用条件变量 / 互斥量——那是把一个
 *     "等别人做完某件事"塞进 1 kHz 回路，和用传感器包唤醒回路是同一类错误。
 *   - **常态下不该进内核。** 1 kHz × 一次无谓的系统调用是白付的。
 *
 * 做法：`ring()` 先无条件推进序号（一次 RMW，纯用户态），**只有真的有人在等**
 * 的时候才发 `FUTEX_WAKE`。发送线程只要还忙着发上一帧就压根不在等，
 * 这一拍连内核都不进。
 *
 * 语义是"**响过没有**"，不是"响了几次"：两次 `ring()` 之间只 `wait` 一次的话，
 * 醒来看到的是最新状态。这正是配 `Snapshot`（latest-wins）该有的行为——
 * 落后时要发的是最新那一帧，不是补齐一串旧指令。
 *
 * @warning 等待侧的"上次看到哪"是**成员状态**，所以只能有一个等待者。
 *          振铃者也只该有一个（周期域那条线程）。
 */
class Doorbell {
public:
    Doorbell() noexcept = default;

    Doorbell(const Doorbell&) = delete;
    Doorbell& operator=(const Doorbell&) = delete;
    Doorbell(Doorbell&&) = delete;
    Doorbell& operator=(Doorbell&&) = delete;

    /**
     * @brief 振铃者（周期域）调用。noexcept、不分配、不加锁、无等待者时不进内核。
     */
    void ring() noexcept {
        // 两个 seq_cst 的顺序不是装饰，是这段代码全部的正确性所在。
        //
        // `fetch_add` 是 RMW，同时也是全屏障，保证下面读 waiters_ 不会被提到它前面；
        // 等待侧对称地"先登记（RMW）再重查序号"。两边各有一个全屏障隔开
        // 自己的写和对另一方的读，所以不可能双方都看不见对方。
        //
        // 少了这条屏障就是经典的丢唤醒：等待者刚把 waiters_ 置位、还没睡下，
        // 振铃者读到旧的 0 于是不发 wake，等待者一直睡到超时。
        // 症状是"发送偶尔晚一个超时周期"，而且只在特定时序下复现。
        sequence_.fetch_add(1, std::memory_order_seq_cst);
        if (waiters_.load(std::memory_order_seq_cst) != 0)
            atomic_futex_notify_one(sequence_);
    }

    /**
     * @brief 等待者调用。
     * @param timeout 最长等多久。超时返回让调用方有机会检查停机标志。
     * @return true 表示上次 `wait_for()` 之后**响过**（可能不止一次，会合并成一次）。
     */
    [[nodiscard]] bool wait_for(std::chrono::steady_clock::duration timeout) noexcept {
        if (consume())
            return true;

        waiters_.fetch_add(1, std::memory_order_seq_cst);
        // 登记之后必须重查一次：登记之前那一刻振铃的话，上面的 consume() 看不到，
        // 而振铃者当时读到的 waiters_ 还是 0，也不会来叫我们。
        bool rang = consume();
        if (!rang) {
            rang = atomic_futex_wait_for(
                       sequence_, last_seen_, timeout, std::memory_order_acquire)
                && consume();
        }
        waiters_.fetch_sub(1, std::memory_order_seq_cst);
        return rang;
    }

    /// 振铃次数。只给诊断用，不参与任何同步。
    [[nodiscard]] std::uint32_t rings() const noexcept {
        return sequence_.load(std::memory_order_relaxed);
    }

private:
    /// 有新响就吞掉。等待者私有状态——这就是"只能有一个等待者"的来源。
    [[nodiscard]] bool consume() noexcept {
        const std::uint32_t current = sequence_.load(std::memory_order_acquire);
        if (current == last_seen_)
            return false;
        last_seen_ = current;
        return true;
    }

    /// futex 的键，必须是 32 位。回绕要 2^32 次振铃 —— 1 kHz 下 49 天，
    /// 而且只有"恰好在两次 wait 之间回绕整整一圈"才会漏，这个不管。
    std::atomic<std::uint32_t> sequence_{0};
    std::atomic<std::uint32_t> waiters_{0};
    std::uint32_t last_seen_ = 0;
};

} // namespace rmcs_utility
