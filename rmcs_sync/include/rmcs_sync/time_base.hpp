#pragma once

#include <chrono>
#include <cstdint>
#include <stdexcept>

#include "rmcs_sync/tick.hpp"

namespace rmcs_sync {

/// 自由运行的 1 kHz（或任意周期）定时器。周期域唯一的时基。
///
/// 这里**没有抽象基类**，这是有意的。曾经有一个 `TimeBase` 虚基类，理由只有两条，
/// 两条都已经作废：
///   - 「将来插锁相时基」—— 任何形式的时钟对齐都不做（不锁相、不外推、不跨板同步），
///     所以 `PhaseLockedTimeBase` 永远不会存在；
///   - 「确定性回放要换时基」—— 回放不做。
/// 一个只有一种实现、而且第二种实现已被明确取消的抽象，成本不是性能
/// （调用方持有的是具体类型，虚调用早被去掉了），成本是**它在说谎**：
/// 它会让下一个人以为这里有可插拔性，然后围着它设计。
///
/// 将来真要回放或接仿真（仿真也是一种换时基：比实时快、慢、或由物理引擎步进），
/// 把基类加回来是三十行的事，不是不可逆的决定。
///
/// 框架也**不提供**事件驱动的时基，而且那一条是结构性禁止、不会变：
/// 一旦回路由传感器包唤醒，「丢一包」就变成「电机这一拍没人发指令」，
/// 把传感器故障放大成了执行器故障，调优解决不了。
class FreeRunTimeBase final {
public:
    explicit FreeRunTimeBase(Duration period)
        : period_(period) {
        if (period_.count() <= 0)
            throw std::invalid_argument{"FreeRunTimeBase: period must be positive"};
    }

    FreeRunTimeBase(const FreeRunTimeBase&) = delete;
    FreeRunTimeBase& operator=(const FreeRunTimeBase&) = delete;

    /// 第一拍。`now` 是控制线程武装完成后读到的时刻。
    [[nodiscard]] Tick first(Timestamp now) noexcept {
        return Tick{.scheduled = now, .dt = period_, .sequence = 0};
    }

    /// 下一拍。`previous` 是刚跑完的那一拍，`actual_end` 是它实际结束的时刻。
    /// 超时的周期被**跳过**而不是往后累积漂移。
    [[nodiscard]] Tick advance(const Tick& previous, Timestamp actual_end) noexcept {
        Timestamp next = previous.scheduled + period_;

        // 向上取整的整数除法：落后的周期被整拍跳过，
        // 而不是让 scheduled 顺着实际耗时往后漂。
        std::uint64_t skipped = 0;
        if (actual_end > next) {
            const Duration overdue = std::chrono::duration_cast<Duration>(actual_end - next);
            skipped =
                static_cast<std::uint64_t>((overdue - Duration{1}).count() / period_.count()) + 1;
            next = next + period_ * skipped;
        }

        return Tick{
            .scheduled = next,
            .dt = std::chrono::duration_cast<Duration>(next - previous.scheduled),
            .sequence = previous.sequence + 1 + skipped};
    }

    [[nodiscard]] Duration nominal_period() const noexcept { return period_; }

private:
    Duration period_;
};

} // namespace rmcs_sync
