// 门铃的单测。重点只有一个：**不许丢唤醒**。
//
// 丢唤醒是那种"平时跑一万遍都不出，比赛时出一次"的 bug，症状是发送线程偶尔晚一整个
// 超时周期。它出在 ring() 和 wait_for() 各自的"先写自己、再读对方"那一对上——
// 少一个全屏障，双方就可能都看不见对方。所以最后那个压力测试才是这个文件的重点，
// 前面几个只是把语义钉住。

#include <atomic>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include <rmcs_utility/doorbell.hpp>

namespace {

using namespace std::chrono_literals;
using rmcs_utility::Doorbell;

} // namespace

TEST(Doorbell, NoRingMeansTimeout) {
    Doorbell bell;
    EXPECT_FALSE(bell.wait_for(1ms));
}

TEST(Doorbell, RingBeforeWaitIsNotLost) {
    Doorbell bell;
    bell.ring();
    // 先响后等：等待者必须立刻拿到，而不是睡到超时。
    EXPECT_TRUE(bell.wait_for(0ms));
}

TEST(Doorbell, RingsCoalesce) {
    Doorbell bell;
    bell.ring();
    bell.ring();
    bell.ring();

    // 语义是"响过没有"，不是"响了几次"：三次并成一次。
    // 这正是配 Snapshot（latest-wins）该有的行为 —— 落后时要发的是最新那一帧，
    // 不是补齐三条旧指令。
    EXPECT_TRUE(bell.wait_for(0ms));
    EXPECT_FALSE(bell.wait_for(0ms));
    EXPECT_EQ(bell.rings(), 3u);
}

TEST(Doorbell, WaiterIsWokenByRing) {
    Doorbell bell;
    std::atomic<bool> woken{false};

    std::thread waiter{[&] {
        // 超时给得很长：如果唤醒机制不工作，这个测试会挂在 join 上而不是悄悄通过。
        if (bell.wait_for(10s))
            woken.store(true, std::memory_order_release);
    }};

    std::this_thread::sleep_for(20ms);
    bell.ring();
    waiter.join();

    EXPECT_TRUE(woken.load(std::memory_order_acquire));
}

/// 这个文件真正的测试。
///
/// 生产者尽可能快地"发布一个值再振铃"，消费者只靠门铃醒来读最新值。
/// 每次等待的超时给到 5 秒 —— 于是**任何一次丢唤醒都会变成一次 5 秒的超时**，
/// 而正常情况下 timeouts 恒为 0。用超时次数当判据，比用总耗时稳，也比用睡眠稳。
TEST(Doorbell, StressNoLostWakeups) {
    constexpr int kRings = 20000;

    Doorbell bell;
    std::atomic<int> published{0};

    std::thread producer{[&] {
        for (int i = 1; i <= kRings; ++i) {
            published.store(i, std::memory_order_release);
            bell.ring();
        }
    }};

    int seen = 0;
    int timeouts = 0;
    while (seen < kRings) {
        if (bell.wait_for(5s))
            seen = published.load(std::memory_order_acquire);
        else
            ++timeouts;
    }
    producer.join();

    EXPECT_EQ(timeouts, 0) << "超时就是丢唤醒：等待者已经登记，振铃者却没看见";
    EXPECT_EQ(seen, kRings);
}

/// 反向压力：等待者绝大多数时候**不在等**（在忙），振铃者应当走不进内核的那条路。
/// 这里测的是正确性 —— 不进内核也不许漏掉状态。
TEST(Doorbell, StressWithBusyWaiter) {
    constexpr int kRings = 20000;

    Doorbell bell;
    std::atomic<int> published{0};
    std::atomic<bool> done{false};
    std::atomic<int> busy{0};

    std::thread consumer{[&] {
        int seen = 0;
        while (seen < kRings) {
            if (bell.wait_for(5s))
                seen = published.load(std::memory_order_acquire);
            // 故意在两次等待之间干点活，制造"振铃时没人在等"的窗口。
            // 用原子而不是 volatile：C++20 起 volatile 的自增被废弃。
            for (int spin = 0; spin < 50; ++spin)
                busy.fetch_add(1, std::memory_order_relaxed);
        }
        done.store(true, std::memory_order_release);
    }};

    for (int i = 1; i <= kRings; ++i) {
        published.store(i, std::memory_order_release);
        bell.ring();
    }
    consumer.join();

    EXPECT_TRUE(done.load(std::memory_order_acquire));
}
