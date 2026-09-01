// DoorbellWorker 的单测。它要保证的三条纪律，每条一个用例：
//   1. 被叫醒就干活；
//   2. 干活抛异常**不许带走线程** —— 这是发送线程最重要的一条：
//      线程一旦没了，之后每一拍的命令都静默发不出去，日志里什么都没有；
//   3. 析构先停再 join，不挂。

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>

#include <rmcs_utility/doorbell_worker.hpp>
#include <rmcs_utility/thread_config.hpp>

namespace {

using namespace std::chrono_literals;
using rmcs_utility::DoorbellWorker;
using rmcs_utility::ThreadConfig;

/// 轮询等到条件成立，或超时。用轮询而不是固定睡眠：前者在慢机器上不会假失败，
/// 在快机器上也不白等。
template <typename Predicate>
[[nodiscard]] bool wait_until(Predicate predicate, std::chrono::milliseconds timeout = 5s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate())
            return true;
        std::this_thread::sleep_for(1ms);
    }
    return predicate();
}

/// 不带任何特权要求的配置：只改线程名。CI 上没有 rtprio 也能过。
ThreadConfig plain_config() { return ThreadConfig{"", "test-worker"}; }

} // namespace

TEST(DoorbellWorker, RunsWhenRung) {
    std::atomic<int> runs{0};
    DoorbellWorker worker{plain_config(), [&] { runs.fetch_add(1, std::memory_order_relaxed); }};

    EXPECT_TRUE(worker.thread_config_error().empty()) << worker.thread_config_error();

    worker.doorbell().ring();
    EXPECT_TRUE(wait_until([&] { return runs.load(std::memory_order_relaxed) >= 1; }));

    // 再叫一次要再干一次。中间等一下，避免两次 ring 被合并成一次
    // （合并是门铃的正确语义，但这里要测的是"叫了就干"）。
    worker.doorbell().ring();
    EXPECT_TRUE(wait_until([&] { return runs.load(std::memory_order_relaxed) >= 2; }));
}

/// 发送路径是会抛的：librmcs 的 transmit() 在 submit 失败时抛，acquire 失败时也抛。
/// 抛了必须只丢这一帧，线程照旧活着接下一次。
TEST(DoorbellWorker, ExceptionDoesNotKillTheThread) {
    std::atomic<int> runs{0};
    std::atomic<bool> should_throw{true};

    DoorbellWorker worker{plain_config(), [&] {
        runs.fetch_add(1, std::memory_order_relaxed);
        if (should_throw.load(std::memory_order_acquire))
            throw std::runtime_error{"transmit failed"};
    }};

    worker.doorbell().ring();
    EXPECT_TRUE(wait_until([&] { return worker.exceptions() >= 1; }));

    // 线程还活着：关掉抛异常，再叫一次，还得干。
    should_throw.store(false, std::memory_order_release);
    const auto before = runs.load(std::memory_order_relaxed);
    worker.doorbell().ring();
    EXPECT_TRUE(wait_until([&] { return runs.load(std::memory_order_relaxed) > before; }))
        << "异常把工作线程带走了 —— 之后所有命令都会静默发不出去";

    EXPECT_EQ(worker.exceptions(), 1u);
}

TEST(DoorbellWorker, DestructorStopsPromptlyEvenWithoutRings) {
    const auto start = std::chrono::steady_clock::now();
    {
        // stop_latency 给 20ms：析构最坏等这么久，不该等满默认的 100ms。
        DoorbellWorker worker{plain_config(), [] {}, 20ms};
        (void)worker;
    }
    // 宽松上限，只为抓住"析构挂住"这种失败，不是在测调度精度。
    EXPECT_LT(std::chrono::steady_clock::now() - start, 3s);
}

TEST(DoorbellWorker, CountsWakeups) {
    std::atomic<int> runs{0};
    DoorbellWorker worker{plain_config(), [&] { runs.fetch_add(1, std::memory_order_relaxed); }};

    for (int i = 0; i < 5; ++i) {
        worker.doorbell().ring();
        EXPECT_TRUE(wait_until([&] { return runs.load(std::memory_order_relaxed) >= i + 1; }));
    }
    // 每次都等它跑完再叫下一次，所以不会发生合并，wakeups 应当正好是 5。
    EXPECT_EQ(worker.wakeups(), 5u);
    EXPECT_EQ(worker.exceptions(), 0u);
}
