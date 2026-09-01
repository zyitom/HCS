// Snapshot 是事件域 → 周期域的唯一通道，它错了上面所有控制律都在读垃圾。
// 这里的重点是撕裂：单字段类型永远测不出"字段来自两次 publish 的组合"，所以用多字段探针。
#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <thread>

#include <gtest/gtest.h>

#include <rmcs_sync/snapshot.hpp>

namespace {

using rmcs_sync::Duration;
using rmcs_sync::Snapshot;
using rmcs_sync::Timestamp;

constexpr Timestamp kBase{std::chrono::seconds{4321}};

/// 多字段探针。不变式绑死四个字段，任何一个来自另一次 publish 都会被抓到。
struct Probe {
    std::uint64_t a, b, c, d;
};

[[nodiscard]] constexpr Probe make_probe(std::uint64_t base) noexcept {
    return Probe{base, base + 1, base + 2, base + 3};
}

[[nodiscard]] constexpr bool consistent(const Probe& p) noexcept {
    return p.b == p.a + 1 && p.c == p.a + 2 && p.d == p.a + 3;
}

// ── 撕裂检测 ────────────────────────────────────────────────────────────────

TEST(Snapshot, ConcurrentPublishIsNeverTorn) {
    constexpr std::uint64_t kPublishCount = 1'000'000; // 单次压测控制在 2s 以内

    Snapshot<Probe>  snapshot;
    const Timestamp  reference = kBase; // 固定参考时刻：这条测试不关心 age，也省掉读表开销

    std::atomic<bool>          writer_done{false};
    std::atomic<std::uint64_t> read_count{0};
    std::atomic<std::uint64_t> fresh_count{0};
    std::atomic<bool>          torn_seen{false};
    Probe                      torn{}; // 只留第一个撕裂样本，断言交给主线程（join 之后再读）

    std::thread reader([&] {
        for (;;) {
            // 先取标志再读：写者停了以后仍然多读一次，最后一个样本不会漏。
            const bool writer_finished = writer_done.load(std::memory_order_acquire);
            const auto reading = snapshot.read(reference);
            if (reading.valid) {
                if (!consistent(reading.value) && !torn_seen.load(std::memory_order_relaxed)) {
                    torn = reading.value;
                    torn_seen.store(true, std::memory_order_release);
                }
                read_count.fetch_add(1, std::memory_order_relaxed);
                if (reading.fresh)
                    fresh_count.fetch_add(1, std::memory_order_relaxed);
            }
            if (writer_finished)
                break;
        }
    });

    for (std::uint64_t i = 1; i <= kPublishCount; ++i)
        snapshot.publish(make_probe(i), reference);
    writer_done.store(true, std::memory_order_release);
    reader.join();

    EXPECT_FALSE(torn_seen.load(std::memory_order_acquire))
        << "读到的字段来自两次 publish: a=" << torn.a << " b=" << torn.b << " c=" << torn.c
        << " d=" << torn.d;
    EXPECT_GT(read_count.load(), std::uint64_t{0});
    // 读者一次变化都没看到的话，上面的不变式检查是空转，这条测试就没有意义了。
    EXPECT_GT(fresh_count.load(), std::uint64_t{0});

    const auto final_reading = snapshot.read(reference);
    EXPECT_TRUE(final_reading.valid);
    EXPECT_TRUE(consistent(final_reading.value));
    EXPECT_EQ(final_reading.value.a, kPublishCount); // 停下来以后必须是最后那次的值
}

// ── 排列不变式（黑盒判据）──────────────────────────────────────────────────

/// 三个槽位号互不相同。写者、读者、待取的格子必须始终是三个不同的格子，
/// 一旦重合，写者就会写进读者正在读的那格。
[[nodiscard]] bool is_permutation_of_012(const Snapshot<Probe>::SlotIndices& slots) noexcept {
    return slots.write < 3 && slots.read < 3 && slots.back < 3 && slots.write != slots.read
        && slots.write != slots.back && slots.read != slots.back;
}

TEST(Snapshot, SlotIndicesAreAlwaysAPermutation) {
    Snapshot<Probe> snapshot;
    EXPECT_TRUE(is_permutation_of_012(snapshot.slot_indices_for_test()));

    for (std::uint64_t i = 1; i <= 1000; ++i) {
        snapshot.publish(make_probe(i), kBase);
        ASSERT_TRUE(is_permutation_of_012(snapshot.slot_indices_for_test()))
            << "publish 之后槽位号重合了，i=" << i;
        (void)snapshot.read(kBase);
        ASSERT_TRUE(is_permutation_of_012(snapshot.slot_indices_for_test()))
            << "read 之后槽位号重合了，i=" << i;
    }

    // 连发不读：写者自己在两个格子之间倒手，也不许撞上读者持有的那格。
    for (std::uint64_t i = 1001; i <= 2000; ++i) {
        snapshot.publish(make_probe(i), kBase);
        ASSERT_TRUE(is_permutation_of_012(snapshot.slot_indices_for_test()))
            << "连续 publish 之后槽位号重合了，i=" << i;
    }
}

TEST(Snapshot, ThreeConsecutivePublishesStillReadTheNewest) {
    // {write_index_, read_index_, back_ & kMask} 恒为 {0,1,2} 的排列，但这三个是私有的。
    // 等价的黑盒判据：连续 publish 三次正好把三个格子轮一遍，索引一旦重合，
    // 读者要么拿到旧格子（值不是最后那次），要么拿到写者正在写的格子（撕裂）。
    Snapshot<Probe> snapshot;

    for (std::uint64_t round = 0; round < 1000; ++round) {
        const std::uint64_t base = round * 3 + 1;
        snapshot.publish(make_probe(base), kBase);
        snapshot.publish(make_probe(base + 1), kBase);
        snapshot.publish(make_probe(base + 2), kBase);

        const auto reading = snapshot.read(kBase);
        ASSERT_TRUE(reading.valid);
        ASSERT_TRUE(consistent(reading.value));
        ASSERT_EQ(reading.value.a, base + 2);
        ASSERT_TRUE(reading.fresh);
        ASSERT_EQ(reading.sequence, static_cast<std::uint32_t>(base + 2));
    }
}

TEST(Snapshot, AlternatingPublishReadAlwaysSeesTheLastValue) {
    Snapshot<Probe> snapshot;

    for (std::uint64_t i = 1; i <= 1000; ++i) {
        snapshot.publish(make_probe(i), kBase);
        const auto reading = snapshot.read(kBase);
        ASSERT_TRUE(reading.valid);
        ASSERT_TRUE(consistent(reading.value));
        ASSERT_EQ(reading.value.a, i);
        ASSERT_TRUE(reading.fresh);
        ASSERT_EQ(reading.sequence, static_cast<std::uint32_t>(i));
        ASSERT_EQ(snapshot.published_sequence(), static_cast<std::uint32_t>(i));
    }
}

TEST(Snapshot, RandomPublishBurstsPerReadStayConsistent) {
    // 每"拍"发 0~3 次再读一次：成品 IMU 的拍频造出来的就是这种不规则组合。
    Snapshot<Probe> snapshot;

    std::uint32_t rng = 0x9e3779b9u; // 固定种子，失败可复现
    std::uint64_t published = 0;
    for (int round = 0; round < 2000; ++round) {
        rng = rng * 1103515245u + 12345u;
        const int burst = static_cast<int>((rng >> 16) % 4u);
        for (int i = 0; i < burst; ++i)
            snapshot.publish(make_probe(++published), kBase);

        const auto reading = snapshot.read(kBase);
        ASSERT_EQ(reading.valid, published != 0);
        if (!reading.valid)
            continue;
        ASSERT_TRUE(consistent(reading.value));
        ASSERT_EQ(reading.value.a, published);
        ASSERT_EQ(reading.sequence, static_cast<std::uint32_t>(published));
        ASSERT_EQ(reading.fresh, burst != 0);
    }
}

// ── fresh / valid / age ─────────────────────────────────────────────────────

TEST(Snapshot, NoSampleInThisTickIsNotFresh) {
    Snapshot<int> snapshot;
    snapshot.publish(7, kBase);

    const auto first = snapshot.read(kBase);
    EXPECT_TRUE(first.fresh);
    EXPECT_EQ(first.value, 7);

    // 这一拍没有新样本：值还在（周期域照样要用），但 fresh 必须落回 false。
    const auto second = snapshot.read(kBase);
    EXPECT_FALSE(second.fresh);
    EXPECT_TRUE(second.valid);
    EXPECT_EQ(second.value, 7);
    EXPECT_EQ(second.sequence, first.sequence);
}

TEST(Snapshot, TwoSamplesInOneTickReportTheNewest) {
    Snapshot<int> snapshot;
    snapshot.publish(10, kBase);
    snapshot.publish(20, kBase);

    // 一拍来了两个样本：fresh 为 true，读到的必须是**最新**那个，中间那个直接丢。
    const auto reading = snapshot.read(kBase);
    EXPECT_TRUE(reading.fresh);
    EXPECT_TRUE(reading.valid);
    EXPECT_EQ(reading.value, 20);
    EXPECT_EQ(reading.sequence, std::uint32_t{2});
    EXPECT_EQ(snapshot.published_sequence(), std::uint32_t{2});
}

TEST(Snapshot, InvalidBeforeAnySample) {
    Snapshot<int> snapshot;

    const auto empty = snapshot.read(kBase);
    EXPECT_FALSE(empty.valid);
    EXPECT_FALSE(empty.fresh);
    EXPECT_EQ(empty.sequence, std::uint32_t{0}); // 0 保留给"从未发布"
    EXPECT_EQ(empty.age.count(), 0);             // 没有样本就没有年龄，不能拿参考时刻去减
    EXPECT_EQ(snapshot.published_sequence(), std::uint32_t{0});

    snapshot.publish(1, kBase);
    const auto reading = snapshot.read(kBase);
    EXPECT_TRUE(reading.valid);
    EXPECT_TRUE(reading.fresh);
    EXPECT_EQ(reading.sequence, std::uint32_t{1});
}

TEST(Snapshot, AgeIsReferenceMinusSampledAt) {
    Snapshot<int> snapshot;
    snapshot.publish(42, kBase);

    const auto later = snapshot.read(kBase + std::chrono::milliseconds{5});
    EXPECT_EQ(later.age.count(), std::chrono::nanoseconds{std::chrono::milliseconds{5}}.count());

    // 事件域可能比周期域的名义起点更晚采到样本，age 允许为负。
    const auto earlier = snapshot.read(kBase - std::chrono::microseconds{300});
    EXPECT_EQ(earlier.age.count(),
              -std::chrono::nanoseconds{std::chrono::microseconds{300}}.count());
}

TEST(Snapshot, PeekDoesNotConsumeFreshness) {
    Snapshot<int> snapshot;
    snapshot.publish(5, kBase);

    // published_sequence() 是给 reporter / 调试用的只读窥视，不许推进读者私有状态。
    EXPECT_EQ(snapshot.published_sequence(), std::uint32_t{1});
    EXPECT_EQ(snapshot.published_sequence(), std::uint32_t{1});
    EXPECT_TRUE(snapshot.read(kBase).fresh);
}

// ── 序号回绕 ────────────────────────────────────────────────────────────────

// 发布 2^32 次不现实，所以跳零逻辑被抽成公开静态函数，直接测它。
static_assert(Snapshot<int>::next_sequence(0) == 1);
static_assert(Snapshot<int>::next_sequence(1) == 2);
static_assert(Snapshot<int>::next_sequence(std::numeric_limits<std::uint32_t>::max() - 1)
              == std::numeric_limits<std::uint32_t>::max());
static_assert(Snapshot<int>::next_sequence(std::numeric_limits<std::uint32_t>::max()) == 1);

TEST(Snapshot, NextSequenceSkipsZeroOnWrap) {
    constexpr std::uint32_t kMax = std::numeric_limits<std::uint32_t>::max();
    EXPECT_EQ(Snapshot<int>::next_sequence(0), std::uint32_t{1});
    EXPECT_EQ(Snapshot<int>::next_sequence(12345), std::uint32_t{12346});
    EXPECT_EQ(Snapshot<int>::next_sequence(kMax - 1), kMax);
    // 回绕必须跳过 0：0 是"从未发布"的标记，撞上它 valid 会假性变 false。
    EXPECT_EQ(Snapshot<int>::next_sequence(kMax), std::uint32_t{1});

    // 连续推进一段，序号既不重复也不出现 0。
    std::uint32_t sequence = kMax - 3;
    for (int i = 0; i < 8; ++i) {
        const std::uint32_t next = Snapshot<int>::next_sequence(sequence);
        EXPECT_NE(next, std::uint32_t{0});
        EXPECT_NE(next, sequence);
        sequence = next;
    }
}

} // namespace
