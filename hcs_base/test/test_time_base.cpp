// FreeRunTimeBase 的行为必须与改造前 Executor::calculate_next_iteration_time 逐位一致：
// 超时的周期是被"跳过"的，不是往后累积漂移。这里全部用假时刻断言，不依赖真实时间流逝。
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <type_traits>

#include <gtest/gtest.h>

#include <hcs_base/channel/time_base.hpp>

namespace {

using hcs_sync::Duration;
using hcs_sync::FreeRunTimeBase;
using hcs_sync::Tick;
using hcs_sync::Timestamp;

constexpr Duration  kPeriod = std::chrono::milliseconds{1};
constexpr Timestamp kStart{std::chrono::seconds{1234}};

/// 相对 kStart 的纳秒数。断言失败时打印整数，比 time_point 的字节 dump 有用。
[[nodiscard]] std::int64_t offset_ns(Timestamp t) noexcept {
    return std::chrono::duration_cast<Duration>(t - kStart).count();
}

[[nodiscard]] constexpr Duration periods(std::int64_t n) noexcept { return kPeriod * n; }

/// 拍数 n 的名义起点。零漂移的判据就是 scheduled 永远等于它。
[[nodiscard]] constexpr Timestamp scheduled_of(std::int64_t n) noexcept {
    return kStart + periods(n);
}

TEST(FreeRunTimeBase, RejectsNonPositivePeriod) {
    EXPECT_THROW((FreeRunTimeBase{Duration::zero()}), std::invalid_argument);
    EXPECT_THROW((FreeRunTimeBase{Duration{-1}}), std::invalid_argument);
    EXPECT_NO_THROW((FreeRunTimeBase{Duration{1}}));
}

TEST(FreeRunTimeBase, NominalPeriodIsWhatWasAskedFor) {
    FreeRunTimeBase base{kPeriod};
    EXPECT_EQ(base.nominal_period().count(), kPeriod.count());
}

TEST(FreeRunTimeBase, FirstTick) {
    FreeRunTimeBase base{kPeriod};
    const Tick tick = base.first(kStart);
    EXPECT_EQ(offset_ns(tick.scheduled), 0);
    EXPECT_EQ(tick.dt.count(), kPeriod.count()); // 首拍的 dt 是 period，不是 0
    EXPECT_EQ(tick.sequence, std::uint64_t{0});
    EXPECT_DOUBLE_EQ(tick.dt_seconds(), 0.001);
}

TEST(FreeRunTimeBase, OnTimeAdvancesOneSequencePerCall) {
    FreeRunTimeBase base{kPeriod};
    Tick tick = base.first(kStart);

    for (std::int64_t n = 1; n <= 1000; ++n) {
        // 本拍在半个周期内跑完 —— 不该跳拍。
        const Tick next = base.advance(tick, tick.scheduled + kPeriod / 2);
        ASSERT_EQ(next.sequence, static_cast<std::uint64_t>(n));
        ASSERT_EQ(next.dt.count(), kPeriod.count());
        ASSERT_EQ(offset_ns(next.scheduled), offset_ns(scheduled_of(n)));
        tick = next;
    }
}

TEST(FreeRunTimeBase, DeadlineExactlyMetDoesNotSkip) {
    FreeRunTimeBase base{kPeriod};
    const Tick tick = base.first(kStart);

    // actual_end == next 是"不跳"的边界；晚 1ns 就必须跳 1 拍。
    const Tick on_time = base.advance(tick, tick.scheduled + kPeriod);
    EXPECT_EQ(on_time.sequence, std::uint64_t{1});
    EXPECT_EQ(on_time.dt.count(), kPeriod.count());

    const Tick late_by_1ns = base.advance(tick, tick.scheduled + kPeriod + Duration{1});
    EXPECT_EQ(late_by_1ns.sequence, std::uint64_t{2});
    EXPECT_EQ(late_by_1ns.dt.count(), periods(2).count());
}

TEST(FreeRunTimeBase, Overrun1Point5PeriodsSkipsOneTick) {
    FreeRunTimeBase base{kPeriod};
    const Tick tick = base.first(kStart);

    // 本拍跑了 1.5 个周期 → 超期 0.5 个周期 → 跳 1 拍。
    const Tick next = base.advance(tick, tick.scheduled + periods(3) / 2);
    EXPECT_EQ(next.sequence, std::uint64_t{2}); // 0 → 2，中间那拍被跳过
    EXPECT_EQ(next.dt.count(), periods(2).count());
    EXPECT_EQ(offset_ns(next.scheduled), offset_ns(scheduled_of(2)));
    EXPECT_EQ(next.sequence - tick.sequence - 1, std::uint64_t{1}); // 调用方据此得到 skipped
}

TEST(FreeRunTimeBase, Overrun3Point2PeriodsSkipsThreeTicks) {
    FreeRunTimeBase base{kPeriod};
    const Tick tick = base.first(kStart);

    // 本拍跑了 3.2 个周期 → 超期 2.2 个周期 → 向上取整跳 3 拍。
    const Tick next = base.advance(tick, tick.scheduled + periods(32) / 10);
    EXPECT_EQ(next.sequence, std::uint64_t{4});
    EXPECT_EQ(next.dt.count(), periods(4).count());
    EXPECT_EQ(offset_ns(next.scheduled), offset_ns(scheduled_of(4)));
    EXPECT_EQ(next.sequence - tick.sequence - 1, std::uint64_t{3});
}

TEST(FreeRunTimeBase, WholePeriodOverdueRoundsUpExactly) {
    FreeRunTimeBase base{kPeriod};
    const Tick tick = base.first(kStart);

    // 超期整数个周期是向上取整公式最容易写错的地方：超期 1 个周期跳 1 拍，不是 2 拍。
    const Tick one = base.advance(tick, tick.scheduled + periods(2));
    EXPECT_EQ(one.sequence, std::uint64_t{2});
    EXPECT_EQ(one.dt.count(), periods(2).count());

    const Tick two = base.advance(tick, tick.scheduled + periods(3));
    EXPECT_EQ(two.sequence, std::uint64_t{3});
    EXPECT_EQ(two.dt.count(), periods(3).count());
}

TEST(FreeRunTimeBase, ScheduledNeverDriftsAcrossRandomOverruns) {
    FreeRunTimeBase base{kPeriod};
    Tick tick = base.first(kStart);

    // 固定种子的 LCG：失败可复现。
    std::uint32_t rng = 0x5f3759dfu;
    for (int i = 0; i < 2000; ++i) {
        rng = rng * 1103515245u + 12345u;
        // 本拍实际耗时 0.1 ~ 5.2 个周期，覆盖不跳 / 跳 1 / 跳多拍三种路径。
        const std::int64_t tenths = 1 + static_cast<std::int64_t>((rng >> 16) % 52u);
        const Tick next = base.advance(tick, tick.scheduled + periods(tenths) / 10);

        // 零漂移：scheduled 永远是 first().scheduled + sequence * period。
        ASSERT_EQ(offset_ns(next.scheduled),
                  offset_ns(scheduled_of(static_cast<std::int64_t>(next.sequence))));
        // dt 永远是跨过的拍数 × period，控制律靠它才能在跳拍后仍然对。
        ASSERT_EQ(next.dt.count(), periods(static_cast<std::int64_t>(
                                       next.sequence - tick.sequence)).count());
        ASSERT_GT(next.sequence, tick.sequence);
        tick = next;
    }
}

// 抽象基类已经删掉：锁相不做、回放不做，第二种实现永远不会出现。
// 钉住"它就是个具体类"，免得有人又把虚函数加回来。
static_assert(!std::is_polymorphic_v<FreeRunTimeBase>, "时基不该有 vtable");
static_assert(std::is_final_v<FreeRunTimeBase>);

} // namespace
