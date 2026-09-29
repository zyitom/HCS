// QuiescentCell 的单测：周期域拿到的指针在本拍内永远不会被销毁；周期域不跑拍时宁可泄漏也不销毁。
//
// "被销毁"不真的释放内存：对象来自一个静态池，删除器只把它标成已死。这样周期域线程
// 在拿着指针期间检查它是否已死，是有定义的行为，查得出释放早了，又不会真的踩到释放后的内存。

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "hcs_link/quiescent_cell.hpp"

namespace {

using namespace std::chrono_literals;

struct Canary {
    std::atomic<bool> alive{false};
};

constexpr std::size_t kPoolSize = 4096;
std::array<Canary, kPoolSize> pool;
std::atomic<std::size_t> next_canary{0};

struct MarkDead {
    void operator()(Canary* canary) const noexcept {
        if (canary != nullptr)
            canary->alive.store(false, std::memory_order_release);
    }
};

using Cell = hcs_link::QuiescentCell<Canary, MarkDead>;
using Handle = std::unique_ptr<Canary, MarkDead>;

Handle make_canary() {
    Canary& canary = pool[next_canary.fetch_add(1) % kPoolSize];
    canary.alive.store(true, std::memory_order_release);
    return Handle{&canary};
}

TEST(QuiescentCell, NeverDestroysWhatTheCycleIsUsing) {
    Cell cell;
    std::atomic<bool> stop{false};
    std::atomic<std::uint64_t> violations{0};
    std::atomic<std::uint64_t> ticks{0};

    // 模拟周期域：每拍拿一次指针，拍内反复检查它还活着，拍末走静止点。
    std::thread cycle{[&] {
        while (!stop.load(std::memory_order_relaxed)) {
            if (Canary* canary = cell.acquire()) {
                for (int i = 0; i < 64; ++i)
                    if (!canary->alive.load(std::memory_order_acquire))
                        violations.fetch_add(1);
            }
            cell.quiescent();
            ticks.fetch_add(1, std::memory_order_relaxed);
        }
    }};

    int replaced = 0;
    for (int i = 0; i < 2000; ++i)
        replaced += cell.replace(make_canary(), 1s) ? 1 : 0;

    stop.store(true);
    cycle.join();

    EXPECT_EQ(violations.load(), 0U);
    EXPECT_EQ(replaced, 2000);
    EXPECT_GT(ticks.load(), 0U);
}

TEST(QuiescentCell, LeaksInsteadOfDestroyingWhenTheCycleIsStopped) {
    Cell cell;
    ASSERT_TRUE(cell.replace(make_canary(), 10ms)); // 原来是空的：不用等
    Canary* first = cell.acquire();
    ASSERT_NE(first, nullptr);

    // 没有任何线程在走静止点：换下来的旧对象不能销毁。
    EXPECT_FALSE(cell.replace(make_canary(), 10ms));
    EXPECT_TRUE(first->alive.load());
}

TEST(QuiescentCell, ReplacingWithNullClearsTheCell) {
    Cell cell;
    ASSERT_TRUE(cell.replace(make_canary(), 10ms));
    Canary* first = cell.acquire();

    std::thread cycle{[&] {
        for (int i = 0; i < 1000; ++i) {
            (void)cell.acquire();
            cell.quiescent();
            std::this_thread::sleep_for(100us);
        }
    }};
    EXPECT_TRUE(cell.replace(nullptr, 1s));
    cycle.join();

    EXPECT_EQ(cell.acquire(), nullptr);
    EXPECT_FALSE(first->alive.load());
}

} // namespace
