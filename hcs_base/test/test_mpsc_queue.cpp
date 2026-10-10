// MpscQueue 是多条线程（含周期域）→ 一条线程的出口。要钉死的是三件事：
//   - 满了就丢，丢弃被数对，而且不在队列里留下半构造的对象；
//   - 多个生产者同时写，**每个生产者自己的顺序**不乱、不丢、不重；
//   - 游标回绕（发生在第 Capacity 个元素之后，每一圈一次）之后一切照旧。
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <hcs_base/channel/mpsc_queue.hpp>

namespace {

using hcs_sync::MpscQueue;

static_assert(MpscQueue<int, 4>::capacity == 4);

/// 记活着的实例数：构造 +1、析构 -1。队列里任何一次漏析构或重复析构都会让它对不上。
struct Counted {
    static inline std::atomic<int> alive{0};

    explicit Counted(int value) noexcept
        : value(value) {
        alive.fetch_add(1, std::memory_order_relaxed);
    }
    Counted(Counted&& other) noexcept
        : value(other.value) {
        alive.fetch_add(1, std::memory_order_relaxed);
    }
    Counted& operator=(Counted&& other) noexcept {
        value = other.value;
        return *this;
    }
    ~Counted() { alive.fetch_sub(1, std::memory_order_relaxed); }

    int value;
};

// ── 单线程：边界与计数 ──────────────────────────────────────────────────────

TEST(MpscQueue, EmptyOnConstruction) {
    MpscQueue<int, 4> queue;
    int value = -1;
    EXPECT_FALSE(queue.try_pop(value));
    EXPECT_EQ(value, -1); // 失败的 pop 不许动 out
    EXPECT_EQ(queue.dropped(), std::uint64_t{0});
}

TEST(MpscQueue, HoldsExactlyCapacityElementsInOrder) {
    MpscQueue<int, 4> queue;
    for (int i = 0; i < 4; ++i)
        EXPECT_TRUE(queue.try_push(i)) << "第 " << i << " 个元素就满了，容量没用满";

    EXPECT_FALSE(queue.try_push(4));
    EXPECT_FALSE(queue.try_push(5));
    EXPECT_EQ(queue.dropped(), std::uint64_t{2});

    int value = -1;
    for (int i = 0; i < 4; ++i) {
        ASSERT_TRUE(queue.try_pop(value));
        EXPECT_EQ(value, i); // FIFO，被丢的那两个没有挤掉任何已入队的元素
    }
    EXPECT_FALSE(queue.try_pop(value));
    EXPECT_EQ(queue.dropped(), std::uint64_t{2}); // pop 不该动丢弃计数
}

TEST(MpscQueue, PoppingFreesTheSlotForTheNextLap) {
    MpscQueue<int, 2> queue; // 最小容量：每两个元素回绕一次
    int value = -1;
    for (int i = 0; i < 1000; ++i) {
        ASSERT_TRUE(queue.try_push(i));
        ASSERT_TRUE(queue.try_pop(value));
        EXPECT_EQ(value, i);
    }
    EXPECT_EQ(queue.dropped(), std::uint64_t{0});
}

TEST(MpscQueue, FullThenDrainedThenFullAgain) {
    MpscQueue<int, 4> queue;
    int value = -1;
    for (int lap = 0; lap < 50; ++lap) {
        for (int i = 0; i < 4; ++i)
            ASSERT_TRUE(queue.try_push(lap * 4 + i));
        ASSERT_FALSE(queue.try_push(-1));
        for (int i = 0; i < 4; ++i) {
            ASSERT_TRUE(queue.try_pop(value));
            EXPECT_EQ(value, lap * 4 + i);
        }
    }
    EXPECT_EQ(queue.dropped(), std::uint64_t{50});
}

// ── 非平凡类型：对象生命周期 ────────────────────────────────────────────────

TEST(MpscQueue, RejectedPushLeavesNoObjectBehind) {
    Counted::alive = 0;
    {
        MpscQueue<Counted, 2> queue;
        EXPECT_TRUE(queue.try_emplace(1));
        EXPECT_TRUE(queue.try_emplace(2));
        EXPECT_FALSE(queue.try_emplace(3)); // 满：不许构造
        EXPECT_EQ(Counted::alive.load(), 2);

        Counted out{0};
        ASSERT_TRUE(queue.try_pop(out));
        EXPECT_EQ(out.value, 1);
        EXPECT_EQ(Counted::alive.load(), 2); // 队里剩一个 + out
    }
    EXPECT_EQ(Counted::alive.load(), 0) << "析构没把队里剩下的元素析构掉";
}

TEST(MpscQueue, MoveOnlyTypeRoundTrips) {
    MpscQueue<std::unique_ptr<int>, 4> queue;
    EXPECT_TRUE(queue.try_push(std::make_unique<int>(7)));

    std::unique_ptr<int> out;
    ASSERT_TRUE(queue.try_pop(out));
    ASSERT_NE(out, nullptr);
    EXPECT_EQ(*out, 7);
}

// ── 并发：多个生产者 ────────────────────────────────────────────────────────

struct Tagged {
    std::uint32_t producer;
    std::uint32_t sequence;
    std::uint64_t check; ///< 由前两个字段算出：撕裂（字段来自两次 push）会对不上
};

constexpr std::uint64_t check_of(std::uint32_t producer, std::uint32_t sequence) noexcept {
    return (std::uint64_t{producer} << 40) ^ (std::uint64_t{sequence} * 0x9E3779B97F4A7C15ull);
}

// 每个生产者推自己的 0,1,2,…；满了就丢（记下丢了哪些是做不到的，所以只要求：
// 收到的序号对每个生产者严格递增，收到的条数 + 丢弃数 == 推的总数）。
TEST(MpscQueue, ConcurrentProducersKeepPerProducerOrder) {
    constexpr std::uint32_t kProducers = 4;
    constexpr std::uint32_t kPerProducer = 200'000;

    MpscQueue<Tagged, 256> queue; // 故意小：让"满"和回绕都频繁发生
    std::atomic<std::uint32_t> producers_done{0};
    std::atomic<bool> go{false};

    std::vector<std::thread> producers;
    for (std::uint32_t id = 0; id < kProducers; ++id)
        producers.emplace_back([&, id] {
            while (!go.load(std::memory_order_acquire)) {
            }
            for (std::uint32_t sequence = 0; sequence < kPerProducer; ++sequence)
                queue.try_push(Tagged{id, sequence, check_of(id, sequence)});
            producers_done.fetch_add(1, std::memory_order_release);
        });

    std::vector<std::int64_t> last(kProducers, -1);
    std::uint64_t received = 0;
    std::uint64_t out_of_order = 0;
    std::uint64_t torn = 0;
    const auto drain = [&] {
        for (Tagged item{}; queue.try_pop(item);) {
            ++received;
            if (item.check != check_of(item.producer, item.sequence))
                ++torn;
            else if (static_cast<std::int64_t>(item.sequence) <= last[item.producer])
                ++out_of_order;
            last[item.producer] = item.sequence;
        }
    };

    go.store(true, std::memory_order_release);
    while (producers_done.load(std::memory_order_acquire) != kProducers)
        drain();
    for (auto& producer : producers)
        producer.join();
    drain(); // 生产者都停了之后队里剩下的

    EXPECT_EQ(torn, 0u);
    EXPECT_EQ(out_of_order, 0u);
    EXPECT_EQ(received + queue.dropped(), std::uint64_t{kProducers} * kPerProducer);
    EXPECT_GT(received, 0u);
}

// 消费者跟得上的时候一个都不许丢：生产者遇满就让出再试，于是每一条都必须到。
TEST(MpscQueue, NothingIsLostWhenProducersRetry) {
    constexpr std::uint32_t kProducers = 3;
    constexpr std::uint32_t kPerProducer = 50'000;

    MpscQueue<Tagged, 64> queue;
    std::vector<std::thread> producers;
    for (std::uint32_t id = 0; id < kProducers; ++id)
        producers.emplace_back([&, id] {
            for (std::uint32_t sequence = 0; sequence < kPerProducer; ++sequence)
                while (!queue.try_push(Tagged{id, sequence, check_of(id, sequence)}))
                    std::this_thread::yield();
        });

    std::vector<std::uint32_t> next(kProducers, 0);
    std::uint64_t received = 0;
    while (received < std::uint64_t{kProducers} * kPerProducer) {
        Tagged item{};
        if (!queue.try_pop(item)) {
            std::this_thread::yield();
            continue;
        }
        ASSERT_EQ(item.check, check_of(item.producer, item.sequence));
        ASSERT_EQ(item.sequence, next[item.producer]) << "producer " << item.producer;
        ++next[item.producer];
        ++received;
    }
    for (auto& producer : producers)
        producer.join();

    Tagged leftover{};
    EXPECT_FALSE(queue.try_pop(leftover));
}

} // namespace
