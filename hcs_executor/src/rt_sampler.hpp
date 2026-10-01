#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "hcs_base/cache_line.hpp"
#include "hcs_base/protocol/ring_buffer.hpp"

namespace hcs_utility {

/// 一拍一条。全部是 steady_clock epoch 起的纳秒——RT 侧不做任何单位换算，
/// 连 duration_cast 都留给 reporter，采样点上只剩一次 store。
struct TickSample {
    std::uint64_t sequence;     ///< Tick::sequence
    std::int64_t scheduled_ns;  ///< 本拍名义起点
    std::int64_t start_ns;      ///< 实际起拍
    std::int64_t end_ns;        ///< 本拍最后一个组件返回
    std::uint32_t skipped;      ///< 本拍之前跳过了几拍
    std::uint32_t failed_count; ///< 当前处于 failed 状态的组件数
};

/// 逐组件耗时。只在编译期开关打开时产生，且按 stride 抽样。
struct ComponentSample {
    std::uint64_t sequence;
    std::uint32_t component_index;
    std::uint32_t duration_ns;
};

/**
 * RT 线程只往预分配环里塞裸样本，直方图 / 格式化 / 打印全在 reporter 线程做。
 *
 * 为什么这个要第一个写：没有它，后面全是盲调。
 * 逐拍的原始样本能看相关性，两个 p99 数字不能——
 *   start_late 的尖峰紧跟在某个 update_duration 尖峰后面？是 → 问题在组件里，
 *   某一拍算超时把下一拍挤晚了；不是 → 问题在调度 / 中断 / 节能上，线程根本没被按时唤醒。
 * 这两种情况的修法完全相反（一个改代码，一个改系统配置），而"p99 = 1.8ms"对它们一视同仁。
 * 保留逐拍序列，就是保留把它们分开的那个维度。
 *
 * 底层是本包的 RingBuffer（构造期一次分配、SPSC 无锁）：一个生产者 = RT 线程，
 * 一个消费者 = reporter 线程。
 */
class RtSampler {
public:
    /**
     * @param tick_capacity 逐拍样本环的最小容量，实际容量向上取到 2 的幂。
     *        取值按 reporter 的 drain 间隔算：1kHz + 100ms drain，1024 条已经有 10 倍余量。
     * @param component_capacity 逐组件样本环的最小容量，同上。
     * @note 唯一会分配的地方，必须在封盘点（RealtimeArm 构造）之前调用。
     */
    RtSampler(std::size_t tick_capacity, std::size_t component_capacity)
        : ticks_(tick_capacity)
        , components_(component_capacity) {}

    RtSampler(const RtSampler&) = delete;
    RtSampler& operator=(const RtSampler&) = delete;
    RtSampler(RtSampler&&) = delete;
    RtSampler& operator=(RtSampler&&) = delete;

    /// RT 侧。满则丢弃并计数——采样掉几条是可接受的，为了采样阻塞控制线程不是。
    void record_tick(const TickSample& sample) noexcept {
        if (!ticks_.push_back(sample)) [[unlikely]]
            dropped_ticks_.fetch_add(1, std::memory_order::relaxed);
    }

    /// RT 侧。同上。
    void record_component(const ComponentSample& sample) noexcept {
        if (!components_.push_back(sample)) [[unlikely]]
            dropped_components_.fetch_add(1, std::memory_order::relaxed);
    }

    /**
     * reporter 侧。取走当前所有逐拍样本。
     *
     * @param callback 形如 `void(const TickSample&)`，**必须 noexcept**：
     *        它在 RingBuffer 的消费循环里跑，抛出去就是环的游标推到一半，直接 terminate。
     *        这里用 static_assert 把它拦在编译期，而不是留成运行时惊喜。
     * @return 实际取出的条数
     */
    template <typename F>
    std::size_t drain_ticks(F&& callback) {
        static_assert(
            std::is_nothrow_invocable_v<F&, const TickSample&>,
            "drain_ticks callback must be noexcept-invocable with const TickSample&");
        return ticks_.pop_front_n([&callback](TickSample&& sample) noexcept {
            callback(static_cast<const TickSample&>(sample));
        });
    }

    /// reporter 侧。@return 实际取出的条数
    template <typename F>
    std::size_t drain_components(F&& callback) {
        static_assert(
            std::is_nothrow_invocable_v<F&, const ComponentSample&>,
            "drain_components callback must be noexcept-invocable with const ComponentSample&");
        return components_.pop_front_n([&callback](ComponentSample&& sample) noexcept {
            callback(static_cast<const ComponentSample&>(sample));
        });
    }

    /// 丢弃计数只增不清：它是"这份统计有多不完整"的度量，清零会让报告看起来比实际干净。
    [[nodiscard]] std::uint64_t dropped_ticks() const noexcept {
        return dropped_ticks_.load(std::memory_order::relaxed);
    }

    [[nodiscard]] std::uint64_t dropped_components() const noexcept {
        return dropped_components_.load(std::memory_order::relaxed);
    }

private:
    RingBuffer<TickSample> ticks_;
    RingBuffer<ComponentSample> components_;

    // RT 侧 fetch_add、reporter 侧 load，各自独占一条 line，别去污染环的游标。
    alignas(kCacheLine) std::atomic<std::uint64_t> dropped_ticks_{0};
    alignas(kCacheLine) std::atomic<std::uint64_t> dropped_components_{0};
};

} // namespace hcs_utility
