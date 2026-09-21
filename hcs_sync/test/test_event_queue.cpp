// EventQueue 是周期域 → 事件域的出口：满了就丢是**显式策略**，所以丢弃必须被数对，
// 而且丢弃不能在队列里留下半构造的对象。非平凡类型那一组测的就是后者。
#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <hcs_sync/event_queue.hpp>

namespace {

using hcs_sync::EventQueue;

static_assert(EventQueue<int, 4>::capacity == 4);
static_assert(EventQueue<int, 1024>::capacity == 1024);

// ── 平凡类型：边界与计数 ────────────────────────────────────────────────────

TEST(EventQueue, EmptyOnConstruction) {
    EventQueue<int, 4> queue;
    int value = -1;
    EXPECT_TRUE(queue.empty());
    EXPECT_EQ(queue.size(), std::size_t{0});
    EXPECT_EQ(queue.dropped(), std::uint64_t{0});
    EXPECT_FALSE(queue.try_pop(value));
    EXPECT_EQ(value, -1); // 失败的 pop 不许动 out
}

TEST(EventQueue, HoldsExactlyCapacityElements) {
    EventQueue<int, 4> queue;

    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(queue.try_push(i)) << "第 " << i << " 个元素就满了，容量没用满";
        EXPECT_EQ(queue.size(), static_cast<std::size_t>(i + 1));
    }
    EXPECT_FALSE(queue.empty());

    EXPECT_FALSE(queue.try_push(4));
    EXPECT_EQ(queue.dropped(), std::uint64_t{1});
    EXPECT_EQ(queue.size(), std::size_t{4}); // 丢弃不许影响已入队的元素

    int value = -1;
    for (int i = 0; i < 4; ++i) {
        ASSERT_TRUE(queue.try_pop(value));
        EXPECT_EQ(value, i); // FIFO
    }
    EXPECT_TRUE(queue.empty());
    EXPECT_FALSE(queue.try_pop(value));
    EXPECT_EQ(queue.dropped(), std::uint64_t{1}); // pop 不该动丢弃计数
}

TEST(EventQueue, DroppedCountsEveryRejectedPush) {
    EventQueue<int, 2> queue; // 最小容量
    EXPECT_TRUE(queue.try_push(1));
    EXPECT_TRUE(queue.try_push(2));

    for (int i = 0; i < 100; ++i)
        EXPECT_FALSE(queue.try_push(i));
    EXPECT_EQ(queue.dropped(), std::uint64_t{100});
    EXPECT_EQ(queue.size(), std::size_t{2});

    int value = 0;
    ASSERT_TRUE(queue.try_pop(value));
    EXPECT_EQ(value, 1);
    EXPECT_TRUE(queue.try_push(3)); // 腾出一格就又能写
    EXPECT_EQ(queue.dropped(), std::uint64_t{100});
}

TEST(EventQueue, IndexWrapsAroundManyTimes) {
    // 槽号是 index & (Capacity - 1)，绕圈几百次才能暴露掩码写错的实现。
    EventQueue<int, 4> queue;
    int value = 0;

    for (int i = 0; i < 1000; ++i) {
        ASSERT_TRUE(queue.try_push(i));
        ASSERT_TRUE(queue.try_push(i + 10000));
        ASSERT_TRUE(queue.try_pop(value));
        ASSERT_EQ(value, i);
        ASSERT_TRUE(queue.try_pop(value));
        ASSERT_EQ(value, i + 10000);
    }
    EXPECT_TRUE(queue.empty());
    EXPECT_EQ(queue.dropped(), std::uint64_t{0});
}

TEST(EventQueue, MoveAndCopyPushBothWork) {
    EventQueue<std::uint64_t, 4> queue;
    const std::uint64_t lvalue = 7;
    std::uint64_t rvalue = 9;

    EXPECT_TRUE(queue.try_push(lvalue));
    EXPECT_TRUE(queue.try_push(std::move(rvalue)));

    std::uint64_t out = 0;
    ASSERT_TRUE(queue.try_pop(out));
    EXPECT_EQ(out, std::uint64_t{7});
    ASSERT_TRUE(queue.try_pop(out));
    EXPECT_EQ(out, std::uint64_t{9});
}

// ── 非平凡类型：构造 / 析构守恒 ────────────────────────────────────────────

/// 带析构计数的非平凡类型。alive 归零 = 不泄漏，destructed == constructed = 不重复析构。
struct Counted {
    static inline int constructed = 0;
    static inline int destructed  = 0;
    static inline int alive       = 0;

    static void reset() noexcept {
        constructed = 0;
        destructed  = 0;
        alive       = 0;
    }

    int payload;

    explicit Counted(int value = 0) noexcept
        : payload(value) {
        ++constructed;
        ++alive;
    }
    Counted(const Counted& other) noexcept
        : payload(other.payload) {
        ++constructed;
        ++alive;
    }
    Counted(Counted&& other) noexcept
        : payload(other.payload) {
        other.payload = -1;
        ++constructed;
        ++alive;
    }
    Counted& operator=(const Counted& other) noexcept {
        payload = other.payload;
        return *this;
    }
    Counted& operator=(Counted&& other) noexcept {
        payload       = other.payload;
        other.payload = -1;
        return *this;
    }
    ~Counted() {
        ++destructed;
        --alive;
    }
};

static_assert(std::is_nothrow_destructible_v<Counted>);
static_assert(!std::is_trivially_destructible_v<Counted>);

TEST(EventQueue, NonTrivialTypeIsNeitherLeakedNorDestroyedTwice) {
    Counted::reset();
    {
        EventQueue<Counted, 4> queue;

        for (int i = 0; i < 4; ++i)
            ASSERT_TRUE(queue.try_push(Counted{i}));
        // 每次 push 净增一个活对象：临时量析构掉，队列里那个留下。
        EXPECT_EQ(Counted::alive, 4);

        // 队满时丢弃：只有调用方那个临时量的生死，队列里一个都不许多构造。
        const int alive_before_drop = Counted::alive;
        EXPECT_FALSE(queue.try_push(Counted{99}));
        EXPECT_EQ(Counted::alive, alive_before_drop);
        EXPECT_EQ(queue.dropped(), std::uint64_t{1});

        Counted out{-1};
        ASSERT_TRUE(queue.try_pop(out));
        EXPECT_EQ(out.payload, 0);
        // 取出一个：队列里的那格被析构，out 是调用方自己的对象。
        EXPECT_EQ(Counted::alive, 4); // 队列 3 + out 1

        const Counted lvalue{5};
        EXPECT_TRUE(queue.try_push(lvalue)); // 拷贝入队
        EXPECT_EQ(Counted::alive, 6);        // 队列 4 + out + lvalue

        // 这里带着 4 个未取出的元素析构队列 —— 它们必须被析构，且只析构一次。
    }
    EXPECT_EQ(Counted::alive, 0);
    EXPECT_EQ(Counted::destructed, Counted::constructed);
}

TEST(EventQueue, DrainedQueueLeavesNothingBehind) {
    Counted::reset();
    {
        EventQueue<Counted, 8> queue;
        for (int round = 0; round < 50; ++round) {
            for (int i = 0; i < 8; ++i)
                ASSERT_TRUE(queue.try_push(Counted{round * 8 + i}));
            Counted out{-1};
            for (int i = 0; i < 8; ++i) {
                ASSERT_TRUE(queue.try_pop(out));
                ASSERT_EQ(out.payload, round * 8 + i);
            }
            ASSERT_TRUE(queue.empty());
        }
        EXPECT_EQ(Counted::alive, 0); // 排空后队列里不该还留着活对象
    }
    EXPECT_EQ(Counted::alive, 0);
    EXPECT_EQ(Counted::destructed, Counted::constructed);
    EXPECT_GT(Counted::constructed, 0);
}

// ── SPSC 双线程 ────────────────────────────────────────────────────────────

TEST(EventQueue, SpscPreservesFifoOrderUnderContention) {
    constexpr std::uint32_t kCount = 1'000'000;
    EventQueue<std::uint32_t, 1024> queue;

    std::atomic<bool> producer_done{false};
    std::atomic<bool> producer_stuck{false};

    std::thread producer([&] {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
        for (std::uint32_t i = 0; i < kCount; ++i) {
            // 满了就重试（会计进 dropped，这里不关心），保证一个都不丢，好逐个核对顺序。
            while (!queue.try_push(i)) {
                if (std::chrono::steady_clock::now() > deadline) {
                    producer_stuck.store(true, std::memory_order_release);
                    producer_done.store(true, std::memory_order_release);
                    return;
                }
            }
        }
        producer_done.store(true, std::memory_order_release);
    });

    std::vector<std::uint32_t> received;
    received.reserve(kCount);
    std::uint32_t value = 0;
    for (;;) {
        const bool finished = producer_done.load(std::memory_order_acquire);
        if (queue.try_pop(value))
            received.push_back(value);
        else if (finished)
            break;
    }
    producer.join();

    ASSERT_FALSE(producer_stuck.load(std::memory_order_acquire)) << "生产者卡死：队列不再放行";
    ASSERT_EQ(received.size(), static_cast<std::size_t>(kCount));
    for (std::uint32_t i = 0; i < kCount; ++i)
        ASSERT_EQ(received[i], i) << "第 " << i << " 个元素乱序或丢失";
}

TEST(EventQueue, SpscConservesEveryElementPushedOrDropped) {
    constexpr std::uint32_t kAttempts = 200'000;
    EventQueue<std::uint32_t, 64> queue; // 故意做小，逼出真实的丢弃

    std::atomic<bool>          producer_done{false};
    std::atomic<std::uint32_t> pushed{0};

    std::thread producer([&] {
        std::uint32_t accepted = 0;
        for (std::uint32_t i = 0; i < kAttempts; ++i) {
            if (queue.try_push(i)) // 不重试：丢了就是丢了，由 dropped() 记账
                ++accepted;
        }
        pushed.store(accepted, std::memory_order_relaxed);
        producer_done.store(true, std::memory_order_release);
    });

    std::uint64_t popped   = 0;
    std::uint32_t previous = 0;
    bool          ordered  = true;
    bool          first    = true;
    std::uint32_t value    = 0;
    for (;;) {
        const bool finished = producer_done.load(std::memory_order_acquire);
        if (queue.try_pop(value)) {
            // 丢弃只会让序列出现空洞，绝不能让它逆序。
            if (!first && value <= previous)
                ordered = false;
            previous = value;
            first    = false;
            ++popped;
        } else if (finished)
            break;
    }
    producer.join();

    EXPECT_GT(popped, std::uint64_t{0}); // 消费者一个都没拿到的话下面两条是空转
    EXPECT_TRUE(ordered) << "取出的元素不是严格递增的，FIFO 被破坏";
    EXPECT_EQ(popped, static_cast<std::uint64_t>(pushed.load(std::memory_order_relaxed)));
    // 总数守恒：每次 try_push 要么进了队列，要么被记成一次丢弃。
    EXPECT_EQ(popped + queue.dropped(), static_cast<std::uint64_t>(kAttempts));
}

} // namespace
