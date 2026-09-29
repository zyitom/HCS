// RingBuffer 的单测。它是 RtSampler 的底层(RT 线程生产、reporter 消费),
// 内存序错了不会在单线程里发作 —— 所以除了语义用例,必须有一个真并发的
// SPSC 压力测试把顺序性和完整性都钉住。
//
// 注意:所有消费/访问回调都标了 noexcept —— 这是 RingBuffer 的 RT 契约
// (回调在消费循环里跑,抛出去就是游标推到一半),库侧用 static_assert 强制。

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <hcs_utility/ring_buffer.hpp>

namespace {
using hcs_utility::RingBuffer;
} // namespace

TEST(RingBuffer, FifoOrderAndExhaustion) {
    RingBuffer<int> buffer{4}; // 容量向上取 2 的幂 = 4

    for (int i = 0; i < 4; ++i)
        EXPECT_TRUE(buffer.push_back(i));
    EXPECT_FALSE(buffer.push_back(99)); // 满
    EXPECT_EQ(buffer.max_size(), 4u);
    EXPECT_EQ(buffer.readable(), 4u);

    int popped = -1;
    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(buffer.pop_front([&](int value) noexcept { popped = value; }));
        EXPECT_EQ(popped, i);
    }
    EXPECT_FALSE(buffer.pop_front([&](int) noexcept {}));
    EXPECT_EQ(buffer.readable(), 0u);
}

TEST(RingBuffer, WraparoundBeyondCapacity) {
    RingBuffer<int> buffer{2}; // 实际容量 2

    for (int round = 0; round < 100; ++round) {
        EXPECT_TRUE(buffer.push_back(2 * round));
        EXPECT_TRUE(buffer.push_back(2 * round + 1));

        std::size_t sum = 0;
        EXPECT_EQ(
            buffer.pop_front_n([&](int value) noexcept { sum += static_cast<std::size_t>(value); }),
            2u);
        EXPECT_EQ(sum, static_cast<std::size_t>(4 * round + 1));
    }
}

TEST(RingBuffer, SnapshotViewIsProducerIndependent) {
    RingBuffer<int> buffer{8};

    for (int i = 0; i < 3; ++i)
        buffer.push_back(i);

    const auto view = buffer.readable_view();
    ASSERT_EQ(view.size(), 3u);

    // 快照之后生产者再推,视图不延长 —— latest-wins 语义的另一半
    buffer.push_back(99);

    std::size_t total = 0;
    for (const auto& value : view)
        total += static_cast<std::size_t>(value);
    EXPECT_EQ(total, 3u); // 0+1+2,不含后推的 99
}

TEST(RingBuffer, PeekDoesNotConsume) {
    RingBuffer<int> buffer{8};

    for (int i = 0; i < 4; ++i)
        buffer.push_back(i);

    std::vector<int> seen;
    EXPECT_EQ(buffer.peek_front_n([&](int& value) noexcept { seen.push_back(value); }, 3), 3u);
    EXPECT_EQ(seen, (std::vector<int>{0, 1, 2}));
    EXPECT_EQ(buffer.readable(), 4u); // 没有被消费
}

/// 真正的并发测试:SPSC 无锁结构的全部意义所在。
/// 生产者推 [0, N),消费者按序收齐;任何一个乱序/丢失/重复都是内存序 bug。
TEST(RingBuffer, SpscStressPreservesOrderAndCompleteness) {
    constexpr int kCount = 200000;
    RingBuffer<std::uint64_t> buffer{1024};

    std::thread producer{[&] {
        for (std::uint64_t i = 0; i < kCount; ++i) {
            while (!buffer.push_back(i)) {
                // 满了就重试;环够大,不会活锁
            }
        }
    }};

    std::uint64_t expected = 0;
    std::uint64_t received = 0;
    while (received < static_cast<std::uint64_t>(kCount)) {
        received += buffer.pop_front_n([&](std::uint64_t value) noexcept {
            // 收到的值可能因环回绕"跳批",但绝不允许重复或回退
            if (value < expected)
                ADD_FAILURE() << "regression: " << value << " after " << expected;
            expected = value + 1;
        });
    }
    producer.join();

    EXPECT_EQ(received, static_cast<std::uint64_t>(kCount));
    EXPECT_EQ(expected, static_cast<std::uint64_t>(kCount));
}

/// noexcept 构造约束应当在使用点给出可读的编译期错误(static_assert)。
/// 这里是它的运行期对照:满足约束的非平凡类型一切照常。
TEST(RingBuffer, NonTrivialButNothrowTypesWork) {
    struct NothrowOnly {
        int payload;
        NothrowOnly(int p) noexcept
            : payload(p) {}
        NothrowOnly(const NothrowOnly&) noexcept = default;
        NothrowOnly& operator=(const NothrowOnly&) noexcept = default;
        ~NothrowOnly() = default;
    };

    RingBuffer<NothrowOnly> buffer{4};
    EXPECT_TRUE(buffer.emplace_back(7));

    int out = 0;
    EXPECT_TRUE(buffer.pop_front([&](NothrowOnly&& value) noexcept { out = value.payload; }));
    EXPECT_EQ(out, 7);
}
