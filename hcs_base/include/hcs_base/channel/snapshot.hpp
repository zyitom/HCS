#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <type_traits>

#include "hcs_base/cache_line.hpp"
#include "hcs_base/channel/tick.hpp"

namespace hcs_sync {

/// 事件域 → 周期域的唯一通道：三缓冲，两侧都 wait-free，读者永不重试、永不阻塞写者。
///
/// 有了它，周期域里读一个外部量就是一次普通的函数调用，域内代码可以当单线程写——
/// 这是它存在的唯一理由。所以 memory_order 只许出现在本文件这一处槽位交换里。
template <typename T>
requires std::is_trivially_copyable_v<T>
class Snapshot {
public:
    struct Reading {
        T value{};
        Duration age{};              ///< tick.scheduled − sampled_at；可以为负
        std::uint32_t sequence = 0;
        bool valid = false;          ///< 有没有过任何样本
        bool fresh = false;          ///< 上一次 read 之后有没有新的
    };

    Snapshot() noexcept = default;
    Snapshot(const Snapshot&) = delete;
    Snapshot& operator=(const Snapshot&) = delete;

    /// 序号的推进规则。0 保留给“从未发布”，所以回绕时跳过 0。
    /// 抽成公开静态函数只是为了让单测能直接打到回绕这一步（发布 2^32 次不现实）。
    [[nodiscard]] static constexpr std::uint32_t next_sequence(std::uint32_t current) noexcept {
        const std::uint32_t next = current + 1u;
        return next == 0u ? 1u : next;
    }

    /// 事件域调用。wait-free，不阻塞任何人。
    void publish(const T& value, Timestamp sampled_at) noexcept {
        const std::uint32_t sequence =
            next_sequence(writer_.sequence.load(std::memory_order_relaxed));
        writer_.sequence.store(sequence, std::memory_order_relaxed);

        cells_[writer_.index] = Cell{value, sampled_at, sequence};
        // 必须是 acq_rel，两半都要：
        //   release —— 把上面那次整格写钉在交换之前，读者 acquire 到这个槽位时格子必然完整；
        //   acquire —— 换回来的那个槽是读者刚还回来的，读者对它的最后一次读必须在
        //              下一轮我们重写它之前。少了这一半，「读者读完 X」与「写者重用 X」
        //              之间没有 happens-before，形式上就是数据竞争。
        //              x86 上 LOCK XCHG 本来就是全屏障、看不出来，aarch64 上会真撕。
        const unsigned old = back_.exchange(writer_.index | kDirty, std::memory_order_acq_rel);
        writer_.index = old & kMask;
    }

    /// 周期域调用。wait-free。**非 const**（推进读者私有状态）。
    [[nodiscard]] Reading read(Timestamp reference) noexcept {
        // 这次 load 只是「要不要换手」的提示，真正的定序全在下面那次 exchange 上。
        if (back_.load(std::memory_order_relaxed) & kDirty)
            // 同样两半都要：acquire 看见写者的整格写，release 把我们对旧槽的最后一次读
            // 钉在交还之前（写者随后就会重用那个槽）。参见 publish() 里的注释。
            reader_.index = back_.exchange(reader_.index, std::memory_order_acq_rel) & kMask;

        const Cell& cell = cells_[reader_.index];
        Reading reading;
        reading.value = cell.value;
        reading.sequence = cell.sequence;
        reading.valid = (cell.sequence != 0);
        reading.age = reading.valid
                        ? std::chrono::duration_cast<Duration>(reference - cell.sampled_at)
                        : Duration{};
        reading.fresh = reading.valid && (cell.sequence != reader_.last_seen);
        reader_.last_seen = cell.sequence;
        return reading;
    }

    /// 只读窥视，不推进 fresh 状态。给 reporter / 调试用。
    [[nodiscard]] std::uint32_t published_sequence() const noexcept {
        return writer_.sequence.load(std::memory_order_relaxed);
    }

    /// 三个槽位号。仅供单测验排列不变式用，产品代码不要碰——它不参与任何同步。
    struct SlotIndices {
        unsigned write;
        unsigned read;
        unsigned back;
    };

    [[nodiscard]] SlotIndices slot_indices_for_test() const noexcept {
        return SlotIndices{
            writer_.index, reader_.index, back_.load(std::memory_order_relaxed) & kMask};
    }

private:
    // 布局规则：**一条 cache line 只有一个写者**。两侧各自的状态、三个槽位、那个共享的
    // 交换字，各占自己的行。不这样的话，写者每推进一次自己的序号、读者每记一次 last_seen，
    // 都会把对方正要用的那条行从对方核上抢走——逻辑上互不相干，硬件上却在抢同一条行。

    /// 一个槽位。按 cache line 对齐（sizeof 随之取整到行的整数倍），
    /// 所以写者正在写的那格与读者正在读的那格永远不共行，无论 T 多小。
    struct alignas(hcs_utility::kCacheLine) Cell {
        T value;
        Timestamp sampled_at;
        std::uint32_t sequence;
    };

    /// 只有写者碰。
    struct alignas(hcs_utility::kCacheLine) WriterSide {
        unsigned index = 1;
        // 本该是普通 uint32；用 atomic 只为让 published_sequence() 能被 reporter 线程
        // 无 UB 地窥视，relaxed 不参与任何跨线程定序。
        std::atomic<std::uint32_t> sequence{0};
    };

    /// 只有读者碰。
    struct alignas(hcs_utility::kCacheLine) ReaderSide {
        unsigned index = 2;
        std::uint32_t last_seen = 0;
    };

    static constexpr unsigned kMask = 0b011u;
    static constexpr unsigned kDirty = 0b100u;

    // 不变式：任何时刻 {writer_.index, reader_.index, back_.load() & kMask} 都是 {0,1,2} 的
    // 一个排列。三个槽位分别是“写者正在写的”“读者正在读的”“已发布待取的”，两次 exchange
    // 只是换手，从不复制槽号，所以排列性由构造时的 {1,2,0} 一路保持下去。
    Cell cells_[3]{};
    /// 两侧唯一共享的一个字。换手时这条行必然在两核之间走一趟——那是传递数据本身的代价，
    /// 省不掉；独占一行是为了让它**只**在换手时走。
    alignas(hcs_utility::kCacheLine) std::atomic<unsigned> back_{0};
    WriterSide writer_;
    ReaderSide reader_;
};

} // namespace hcs_sync
