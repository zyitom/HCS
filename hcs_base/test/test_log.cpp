// hcs_log：Logger（前端）→ Backend（队列 + 日志线程）→ Sink（输出端）。
// 要钉死的：
//   - 两套入口（当场格式化 / 延迟格式化）出来的文本一样对；
//   - 输出端堵死的时候，**打日志的线程不等它**——这是整个库存在的理由；
//   - 丢弃被报出来，而不是静默；
//   - flush / 换输出端 / 析构时，已经入队的日志一条不少；
//   - 输出端堵死时 flush_for() 按时放弃，而不是跟着一起堵死。
#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <hcs_base/logging/backend.hpp>
#include <hcs_base/logging/logger.hpp>
#include <hcs_base/thread/rt_attributes.hpp>

namespace {

using namespace std::chrono_literals;
using hcs_log::Backend;
using hcs_log::Level;
using hcs_log::Logger;

// ── 哪些实参可以延迟格式化（编译期）───────────────────────────────────────
static_assert(hcs_log::DeferredArgument<int>);
static_assert(hcs_log::DeferredArgument<double>);
static_assert(hcs_log::DeferredArgument<bool>);
static_assert(hcs_log::DeferredArgument<std::uint64_t>);
static_assert(!hcs_log::DeferredArgument<const char*>);
static_assert(!hcs_log::DeferredArgument<std::string>);
static_assert(!hcs_log::DeferredArgument<std::string_view>);
static_assert(!hcs_log::DeferredArgument<char[8]>);

struct Line {
    Level level;
    std::string logger;
    std::string text;
};

/// 把收到的日志记下来。write() 只在日志线程上跑；测试线程在 flush() 之后才读，
/// flush 内部的互斥量给了这两者 happens-before，但换输出端之后就没有了，所以自己加锁。
class Capture final : public hcs_log::Sink {
public:
    struct Lines {
        std::mutex mutex;
        std::vector<Line> lines;

        std::vector<Line> snapshot() {
            const std::scoped_lock lock{mutex};
            return lines;
        }
    };

    explicit Capture(std::shared_ptr<Lines> out)
        : out_(std::move(out)) {}

    void write(const hcs_log::Entry& entry) override {
        const std::scoped_lock lock{out_->mutex};
        out_->lines.push_back(Line{entry.level, std::string{entry.logger}, std::string{entry.text}});
    }

private:
    std::shared_ptr<Lines> out_;
};

/// 一打开闸门之前，write() 一直卡着——模拟读端不动的管道。
class Stuck final : public hcs_log::Sink {
public:
    struct Gate {
        std::promise<void> open;
        std::shared_future<void> opened{open.get_future()};
        std::atomic<std::uint64_t> written{0};
        std::atomic<std::uint64_t> drop_reports{0};
    };

    explicit Stuck(std::shared_ptr<Gate> gate)
        : gate_(std::move(gate)) {}

    void write(const hcs_log::Entry& entry) override {
        gate_->opened.wait();
        if (entry.logger == "hcs_log")
            gate_->drop_reports.fetch_add(1, std::memory_order_relaxed);
        else
            gate_->written.fetch_add(1, std::memory_order_relaxed);
    }

private:
    std::shared_ptr<Gate> gate_;
};

struct Bench {
    std::shared_ptr<Capture::Lines> lines = std::make_shared<Capture::Lines>();
    Backend backend{std::make_unique<Capture>(lines)};

    std::vector<Line> flushed() {
        backend.flush();
        return lines->snapshot();
    }
};

// 周期域入口必须真的是 nonblocking 的：clang 下这一段由 -Wfunction-effects 检查，
// 里面一旦混进格式化 / 分配 / 加锁，这个文件就编不过零警告。
void periodic_domain(const Logger& logger, int tick, double value) noexcept HCS_NONBLOCKING {
    logger.rt().warn("tick {} value {:.2f}", tick, value);
}

void periodic_domain_text(const Logger& logger, std::string_view text) noexcept HCS_NONBLOCKING {
    logger.rt().write(Level::kWarn, text);
}

} // namespace

TEST(Log, ImmediateFormatting) {
    Bench bench;
    const Logger logger{bench.backend, "imu"};

    logger.info("baudrate {} on {}", 921600, std::string{"uart0"});
    logger.error("plain");

    const auto lines = bench.flushed();
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_EQ(lines[0].level, Level::kInfo);
    EXPECT_EQ(lines[0].logger, "imu");
    EXPECT_EQ(lines[0].text, "baudrate 921600 on uart0");
    EXPECT_EQ(lines[1].level, Level::kError);
    EXPECT_EQ(lines[1].text, "plain");
}

TEST(Log, DeferredFormattingProducesTheSameText) {
    Bench bench;
    const Logger logger{bench.backend, "control"};

    periodic_domain(logger, 7, 1.005);
    logger.rt().error("flags {} {} id {:#x}", true, false, std::uint16_t{0x201});
    logger.rt().info("no arguments");

    const auto lines = bench.flushed();
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_EQ(lines[0].level, Level::kWarn);
    EXPECT_EQ(lines[0].logger, "control");
    EXPECT_EQ(lines[0].text, std::format("tick {} value {:.2f}", 7, 1.005));
    EXPECT_EQ(lines[1].text, "flags true false id 0x201");
    EXPECT_EQ(lines[2].text, "no arguments");
}

TEST(Log, PreformattedText) {
    Bench bench;
    const Logger logger{bench.backend, "guard"};
    logger.write(Level::kWarn, "braces {} are not a format string here");

    const auto lines = bench.flushed();
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0].text, "braces {} are not a format string here");
}

// 空文本也是一条日志。空视图的 data() 可以是空指针，两个入口都不许把它递给 memcpy
// （UBSan 下这个测试才有牙齿）。
TEST(Log, EmptyTextIsStillARecord) {
    Bench bench;
    const Logger logger{bench.backend, "empty"};
    logger.write(Level::kInfo, std::string_view{});
    logger.rt().write(Level::kInfo, std::string_view{});
    logger.info("");

    const auto lines = bench.flushed();
    ASSERT_EQ(lines.size(), 3u);
    for (const auto& line : lines)
        EXPECT_EQ(line.text, "");
}

// 周期域的文本入口：装得下原样到，装不下截断（那边不许上堆），而且看得出被截过。
TEST(Log, RealtimeTextIsCopiedAndTruncatedNotSpilled) {
    constexpr std::size_t kCapacity = hcs_log::Record::kPayloadCapacity;

    Bench bench;
    const Logger logger{bench.backend, "rt-text"};
    const std::string exact(kCapacity, 'e');
    const std::string one_over(kCapacity + 1, 'o');
    const std::string huge(5000, 'x');

    periodic_domain_text(logger, "braces {} are not a format string here");
    periodic_domain_text(logger, exact);
    periodic_domain_text(logger, one_over);
    periodic_domain_text(logger, huge);

    const auto lines = bench.flushed();
    ASSERT_EQ(lines.size(), 4u);
    EXPECT_EQ(lines[0].level, Level::kWarn);
    EXPECT_EQ(lines[0].logger, "rt-text");
    EXPECT_EQ(lines[0].text, "braces {} are not a format string here");
    EXPECT_EQ(lines[1].text, exact);
    EXPECT_EQ(lines[2].text, std::string(kCapacity - 3, 'o') + "...");
    EXPECT_EQ(lines[3].text, std::string(kCapacity - 3, 'x') + "...");
}

// 截断点落在一个多字节字符中间时，整个字符一起去掉：输出端拿到的永远是合法的 UTF-8。
TEST(Log, RealtimeTruncationDoesNotSplitAUtf8Character) {
    constexpr std::size_t kCapacity = hcs_log::Record::kPayloadCapacity;
    const std::string wide = "\xE6\x97\xA5"; // "日"，三个字节

    // 让这个字符分别以第 1、2、3 个字节压在截断点（kCapacity - 3）上。
    for (std::size_t lead = kCapacity - 5; lead <= kCapacity - 3; ++lead) {
        Bench bench;
        const Logger logger{bench.backend, "utf8"};
        logger.rt().write(Level::kInfo, std::string(lead, 'a') + wide + std::string(100, 'b'));

        const auto lines = bench.flushed();
        ASSERT_EQ(lines.size(), 1u);
        // 字符整个落在截断点之前才留得下；否则连同它的首字节一起去掉。
        const bool fits = lead + wide.size() <= kCapacity - 3;
        EXPECT_EQ(lines[0].text, std::string(lead, 'a') + (fits ? wide : "") + "...")
            << "lead = " << lead;
    }
}

TEST(Log, OneThreadsOrderIsPreservedAcrossBothEntrances) {
    Bench bench;
    const Logger logger{bench.backend, "order"};
    for (int i = 0; i < 100; ++i) {
        if (i % 2 == 0)
            logger.info("{}", i);
        else
            logger.rt().info("{}", i);
    }

    const auto lines = bench.flushed();
    ASSERT_EQ(lines.size(), 100u);
    for (int i = 0; i < 100; ++i)
        EXPECT_EQ(lines[static_cast<std::size_t>(i)].text, std::to_string(i));
}

TEST(Log, ThresholdStopsRecordsAtTheCallSite) {
    Bench bench;
    const Logger logger{bench.backend, "level"};

    logger.debug("hidden by default");
    logger.rt().debug("hidden too {}", 1);
    logger.info("shown");
    EXPECT_EQ(bench.flushed().size(), 1u);

    bench.backend.set_threshold(Level::kDebug);
    logger.debug("now shown");
    bench.backend.set_threshold(Level::kError);
    logger.warn("hidden again");
    logger.error("shown");

    const auto lines = bench.flushed();
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_EQ(lines[1].text, "now shown");
    EXPECT_EQ(lines[2].text, "shown");
}

// 尽力域的日志不截断：比记录的定长载荷长的文本放到堆上，原样到达。
TEST(Log, OverlongTextArrivesWhole) {
    Bench bench;
    const Logger logger{bench.backend, "long"};
    const std::string huge(5000, 'x');
    const std::string exact(hcs_log::Record::kPayloadCapacity, 'e');     // 恰好装满
    const std::string one_over(hcs_log::Record::kPayloadCapacity + 1, 'o'); // 恰好溢出

    logger.info("[{}] {}", 42, huge);
    logger.write(Level::kInfo, huge);
    logger.write(Level::kInfo, exact);
    logger.info("{}", one_over);
    logger.info("short");

    const auto lines = bench.flushed();
    ASSERT_EQ(lines.size(), 5u);
    EXPECT_EQ(lines[0].text, "[42] " + huge);
    EXPECT_EQ(lines[1].text, huge);
    EXPECT_EQ(lines[2].text, exact);
    EXPECT_EQ(lines[3].text, one_over);
    EXPECT_EQ(lines[4].text, "short");
}

// 溢出的文本在堆上。队列满时没入队的那几条、以及析构时还压在队列里的那几条，
// 都不许漏（这个测试在 ASan / LSan 下才真正有牙齿，平时至少保证不崩）。
TEST(Log, SpilledTextSurvivesAFullQueueAndShutdown) {
    auto gate = std::make_shared<Stuck::Gate>();
    const std::string huge(3000, 'x');
    {
        Backend backend{std::make_unique<Stuck>(gate)};
        const Logger logger{backend, "spill"};
        for (int i = 0; i < 3000; ++i)
            logger.info("{} {}", i, huge);
        EXPECT_GT(backend.dropped(), 0u);
        gate->open.set_value();
    }
    EXPECT_GT(gate->written.load(), 0u);
}

TEST(Log, OverlongLoggerNameIsTruncatedNotOverrun) {
    Bench bench;
    const std::string name(100, 'n');
    const Logger logger{bench.backend, name};
    logger.info("x");

    const auto lines = bench.flushed();
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0].logger, std::string(hcs_log::Record::kNameCapacity, 'n'));
}

// 这个库存在的理由：输出端卡死（管道另一头不读了）的时候，打日志的线程照常往下走。
TEST(Log, AStuckSinkNeverBlocksTheCaller) {
    auto gate = std::make_shared<Stuck::Gate>();
    constexpr std::uint64_t kSubmitted = 5000; // 远超队列容量

    {
        Backend backend{std::make_unique<Stuck>(gate)};
        const Logger logger{backend, "flood"};

        const auto start = std::chrono::steady_clock::now();
        for (std::uint64_t i = 0; i < kSubmitted; ++i)
            logger.rt().warn("event {}", i);
        const auto elapsed = std::chrono::steady_clock::now() - start;

        // 输出端一条都没放行，5000 次调用却全部返回了。给一个极宽的上限，只为区分
        // "没阻塞"和"阻塞到超时"——真阻塞的话这里根本走不到。
        EXPECT_LT(elapsed, 2s);
        EXPECT_EQ(gate->written.load(), 0u);
        EXPECT_GT(backend.dropped(), 0u);
        EXPECT_LE(backend.dropped(), kSubmitted);

        const std::uint64_t dropped = backend.dropped();
        gate->open.set_value();
        backend.flush();

        // 没丢的全部到了；丢了多少被报了出来。
        EXPECT_EQ(gate->written.load() + dropped, kSubmitted);
        EXPECT_GE(gate->drop_reports.load(), 1u);
    }
}

// terminate 处理函数走的路：输出端堵着的时候按时放弃，而不是把一个正要死的进程挂住。
TEST(Log, FlushForGivesUpOnAStuckSinkAndSucceedsOnceItMoves) {
    auto gate = std::make_shared<Stuck::Gate>();
    Backend backend{std::make_unique<Stuck>(gate)};
    const Logger logger{backend, "bounded"};
    logger.info("held behind the gate");

    const auto start = std::chrono::steady_clock::now();
    EXPECT_FALSE(backend.flush_for(50ms));
    const auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_GE(elapsed, 50ms);
    EXPECT_LT(elapsed, 5s); // 只为区分"按时放弃"和"一直等下去"
    EXPECT_EQ(gate->written.load(), 0u);

    // 放弃的那一次不许留下后遗症：闸门一开，下一次 flush 照常等到。
    gate->open.set_value();
    EXPECT_TRUE(backend.flush_for(30s));
    EXPECT_EQ(gate->written.load(), 1u);
}

TEST(Log, FlushForReturnsTrueWhenThereIsNothingToWaitFor) {
    Bench bench;
    EXPECT_TRUE(bench.backend.flush_for(30s));

    const Logger logger{bench.backend, "bounded"};
    logger.info("one");
    EXPECT_TRUE(bench.backend.flush_for(30s));
    EXPECT_EQ(bench.lines->snapshot().size(), 1u);
}

// 输出端在 write() 里回头调 flush：它等的是自己，必须立刻返回而不是死锁。
TEST(Log, FlushingFromTheLogThreadDoesNotDeadlock) {
    struct Reentrant final : hcs_log::Sink {
        Backend* backend = nullptr;
        std::atomic<int>* refused = nullptr;

        void write(const hcs_log::Entry&) override {
            backend->flush();
            if (!backend->flush_for(30s))
                refused->fetch_add(1, std::memory_order_relaxed);
        }
    };

    std::atomic<int> refused{0};
    auto sink = std::make_unique<Reentrant>();
    auto* reentrant = sink.get();
    reentrant->refused = &refused;

    Bench bench;
    reentrant->backend = &bench.backend;
    (void)bench.backend.exchange_sink(std::move(sink));

    const Logger logger{bench.backend, "reentrant"};
    logger.info("one");
    logger.info("two");
    bench.backend.flush();

    EXPECT_EQ(refused.load(), 2);
}

TEST(Log, ExchangeSinkRoutesEarlierRecordsToTheOldSink) {
    Bench bench;
    const Logger logger{bench.backend, "route"};
    auto second = std::make_shared<Capture::Lines>();

    logger.info("to the first");
    auto first_sink = bench.backend.exchange_sink(std::make_unique<Capture>(second));
    logger.info("to the second");
    bench.backend.flush();

    ASSERT_NE(first_sink, nullptr);
    const auto first = bench.lines->snapshot();
    const auto later = second->snapshot();
    ASSERT_EQ(first.size(), 1u);
    EXPECT_EQ(first[0].text, "to the first");
    ASSERT_EQ(later.size(), 1u);
    EXPECT_EQ(later[0].text, "to the second");
}

TEST(Log, DestructorWritesWhatIsStillQueued) {
    auto lines = std::make_shared<Capture::Lines>();
    {
        // 轮询周期拉到一小时：除了析构时那最后一趟，日志线程不会自己去取。
        Backend backend{std::make_unique<Capture>(lines), std::chrono::hours{1}};
        const Logger logger{backend, "exit"};
        for (int i = 0; i < 50; ++i)
            logger.rt().info("{}", i);
    }
    EXPECT_EQ(lines->snapshot().size(), 50u);
}

// 析构的那一趟排空必须发生在"看到停机标志"**之后**。反过来——排空完了才去看标志——
// 就有一条缝：一趟排空结束到看标志之间入队的记录，没有下一趟来写它了。
//
// 这条缝只有几十纳秒宽，硬撞撞不出来，所以由输出端自己站进去：日志线程每趟排空的最后
// 一件事是报丢弃数，那一次 write() 发生在"队列已经取空"之后、"看停机标志"之前。
// 输出端就在这次 write() 里再入队一条，然后等到析构已经开始才返回。
TEST(Log, DestructorWritesWhatWasQueuedAfterTheLastDrainBegan) {
    struct Shared {
        std::promise<void> open;
        std::shared_future<void> opened{open.get_future()};
        std::promise<void> late_record_queued;
        std::atomic<bool> destroying{false};
        std::atomic<bool> late_record_written{false};
        Backend* backend = nullptr;
    };

    struct InTheGap final : hcs_log::Sink {
        explicit InTheGap(std::shared_ptr<Shared> shared)
            : shared_(std::move(shared)) {}

        void write(const hcs_log::Entry& entry) override {
            shared_->opened.wait();
            if (entry.logger == "late") {
                shared_->late_record_written.store(true);
            } else if (entry.logger == "hcs_log" && !fired_) {
                fired_ = true;
                Logger{*shared_->backend, "late"}.info("queued after the drain emptied the queue");
                shared_->late_record_queued.set_value();
                while (!shared_->destroying.load())
                    std::this_thread::sleep_for(1ms);
                std::this_thread::sleep_for(50ms); // 让析构里的 request_stop 落下去
            }
        }

    private:
        std::shared_ptr<Shared> shared_;
        bool fired_ = false;
    };

    auto shared = std::make_shared<Shared>();
    {
        Backend backend{std::make_unique<InTheGap>(shared)};
        shared->backend = &backend;
        const Logger logger{backend, "flood"};

        // 输出端先堵着，把队列灌满到丢——这样排空的结尾才有一条丢弃报告。
        for (int i = 0; i < 3000; ++i)
            logger.rt().info("{}", i);
        EXPECT_GT(backend.dropped(), 0u);

        shared->open.set_value();
        const auto queued = shared->late_record_queued.get_future().wait_for(30s);
        EXPECT_EQ(queued, std::future_status::ready);

        // 不管上面成不成都要放行：输出端在等这个标志，不放行的话析构里的 join 等不到它。
        shared->destroying.store(true);
    }
    EXPECT_TRUE(shared->late_record_written.load());
}

TEST(Log, AThrowingSinkDoesNotTakeTheProcessDown) {
    struct Throwing final : hcs_log::Sink {
        void write(const hcs_log::Entry&) override { throw std::runtime_error{"sink"}; }
    };
    auto lines = std::make_shared<Capture::Lines>();

    Backend backend{std::make_unique<Throwing>()};
    const Logger logger{backend, "throw"};
    EXPECT_EQ(backend.sink_failures(), 0u);
    logger.info("lost");
    (void)backend.exchange_sink(std::make_unique<Capture>(lines));
    EXPECT_EQ(backend.sink_failures(), 1u); // 丢了，但留了数
    logger.info("kept");
    backend.flush();
    EXPECT_EQ(backend.sink_failures(), 1u);

    const auto kept = lines->snapshot();
    ASSERT_EQ(kept.size(), 1u);
    EXPECT_EQ(kept[0].text, "kept");
}

// 几条线程同时打：每条线程自己的顺序不乱，总数对得上。
TEST(Log, ConcurrentLoggersKeepPerThreadOrder) {
    constexpr int kThreads = 4;
    constexpr int kPerThread = 200; // 合计不超过队列容量，所以一条都不许丢

    Bench bench;
    std::vector<std::thread> threads;
    for (int id = 0; id < kThreads; ++id)
        threads.emplace_back([&, id] {
            const Logger logger{bench.backend, std::format("thread{}", id)};
            for (int i = 0; i < kPerThread; ++i) {
                if (i % 2 == 0)
                    logger.rt().info("{}", i);
                else
                    logger.info("{}", i);
            }
        });
    for (auto& thread : threads)
        thread.join();

    const auto lines = bench.flushed();
    ASSERT_EQ(lines.size(), static_cast<std::size_t>(kThreads * kPerThread));
    EXPECT_EQ(bench.backend.dropped(), 0u);

    std::vector<int> next(kThreads, 0);
    for (const auto& line : lines) {
        const int id = line.logger.back() - '0';
        ASSERT_GE(id, 0);
        ASSERT_LT(id, kThreads);
        EXPECT_EQ(line.text, std::to_string(next[static_cast<std::size_t>(id)]++));
    }
}

TEST(Throttle, FirstCallPassesThenWaitsOutThePeriod) {
    hcs_log::Throttle throttle{30ms};
    EXPECT_TRUE(throttle.ready());
    EXPECT_FALSE(throttle.ready());
    EXPECT_FALSE(throttle.ready());
    std::this_thread::sleep_for(40ms);
    EXPECT_TRUE(throttle.ready());
    EXPECT_FALSE(throttle.ready());
}

TEST(Backoff, FiresOnPowersOfTwo) {
    hcs_log::Backoff backoff;
    std::vector<std::uint64_t> fired;
    for (int i = 0; i < 20; ++i)
        if (const auto count = backoff.hit())
            fired.push_back(*count);

    EXPECT_EQ(fired, (std::vector<std::uint64_t>{1, 2, 4, 8, 16}));
    EXPECT_EQ(backoff.count(), 20u);
}
