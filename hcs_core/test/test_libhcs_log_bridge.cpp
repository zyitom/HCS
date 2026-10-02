// libhcs 的日志：它自己的默认输出，以及在 HCS 里被接到 hcs_log 上
// （src/hardware/libhcs_log_bridge.cpp）之后。要钉死的：
//
//   在 HCS 里
//   - 桥是加载时自动接上的，不靠谁记得去调；
//   - SDK 的一行日志原样到达：级别对、文本对，名字里带着是哪块板；
//   - 接上之后 SDK 不再自己碰 stderr；
//   - 过长的行被截断，而不是让打日志的线程去分配。
//
//   SDK 单独用（没有人接它的出口）
//   - 行的格式和以前一样，写 stderr；
//   - stderr 堵死的时候**不等它**：丢掉、计数，恢复之后先报丢了多少。
//     这一条是 SDK 自己的行为，放在这里测是因为 SDK 仓库里还没有启用测试目标。
#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <hcs_base/logging/backend.hpp>
#include <hcs_base/logging/sink.hpp>
#include <hcs_executor/component.hpp>
#include <libhcs/logging.hpp>

// SDK 的私有头：只有这里能替 SDK"打一行日志"而不必真的接一块板。
#include "host/src/logging/logging.hpp"

namespace {

using namespace std::chrono_literals;
using hcs_log::Level;
namespace sdk = libhcs::host::logging;

struct Line {
    Level level;
    std::string logger;
    std::string text;
};

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

/// 测试期间把进程的日志输出端换成 Capture，结束时换回去。
class CaptureProcessLog {
public:
    CaptureProcessLog()
        : previous_(backend().exchange_sink(std::make_unique<Capture>(lines_))) {}
    ~CaptureProcessLog() { (void)backend().exchange_sink(std::move(previous_)); }

    CaptureProcessLog(const CaptureProcessLog&) = delete;
    CaptureProcessLog& operator=(const CaptureProcessLog&) = delete;

    std::vector<Line> flushed() {
        backend().flush();
        return lines_->snapshot();
    }

    static hcs_log::Backend& backend() { return hcs_executor::process_log_backend(); }

private:
    std::shared_ptr<Capture::Lines> lines_ = std::make_shared<Capture::Lines>();
    std::unique_ptr<hcs_log::Sink> previous_;
};

/// 把 SDK 的出口摘掉一段时间（回到它单独使用时的行为），结束时把桥放回去。
class WithoutTheBridge {
public:
    WithoutTheBridge()
        : bridge_(sdk::set_sink(nullptr)) {}
    ~WithoutTheBridge() { (void)sdk::set_sink(bridge_); }

    WithoutTheBridge(const WithoutTheBridge&) = delete;
    WithoutTheBridge& operator=(const WithoutTheBridge&) = delete;

    [[nodiscard]] sdk::Sink* bridge() const { return bridge_; }

private:
    sdk::Sink* bridge_;
};

/// 把 stderr 接到一根管道上：看这段时间里有没有人往里写、写了什么，也可以先把它灌满。
class StderrPipe {
public:
    StderrPipe() {
        std::fflush(stderr);
        saved_ = ::dup(STDERR_FILENO);
        int ends[2];
        if (::pipe(ends) != 0)
            std::abort();
        read_end_ = ends[0];
        ::dup2(ends[1], STDERR_FILENO);
        ::close(ends[1]);
        // 读端不阻塞：读空了就返回，而不是等下一个字节。只影响读端这一个打开描述。
        ::fcntl(read_end_, F_SETFL, ::fcntl(read_end_, F_GETFL) | O_NONBLOCK);
    }

    ~StderrPipe() { restore(); }

    StderrPipe(const StderrPipe&) = delete;
    StderrPipe& operator=(const StderrPipe&) = delete;

    /// 把管道灌到一个字节都写不进去。
    ///
    /// 走 /proc 另开一个写端来灌：O_NONBLOCK 是打开描述上的标志，直接在 stderr 那个 fd 上设，
    /// 被测代码的 stderr 也就跟着不阻塞了，测试会在什么都没证明的情况下通过。
    void fill() {
        const int filler = ::open("/proc/self/fd/2", O_WRONLY | O_NONBLOCK);
        ASSERT_GE(filler, 0);
        const std::string chunk(4096, '.');
        while (::write(filler, chunk.data(), chunk.size()) > 0) {}
        for (char byte = '.'; ::write(filler, &byte, 1) > 0;) {}
        EXPECT_EQ(errno, EAGAIN);
        ::close(filler);

        // 灌的手段没有污染被测的那个 fd：它仍然是一根会阻塞的管道。
        ASSERT_EQ(::fcntl(STDERR_FILENO, F_GETFL) & O_NONBLOCK, 0);
    }

    /// 读走目前管道里的全部内容（相当于那头的读者追上来了）。
    std::string drain() {
        std::fflush(stderr);
        std::string content;
        char buffer[4096];
        for (ssize_t size; (size = ::read(read_end_, buffer, sizeof(buffer))) > 0;)
            content.append(buffer, static_cast<std::size_t>(size));
        return content;
    }

    /// 还原 stderr，返回这段时间里写进去、还没被读走的全部内容。幂等。
    std::string restore() {
        if (saved_ < 0)
            return {};
        std::string content = drain();
        ::dup2(saved_, STDERR_FILENO);
        ::close(saved_);
        ::close(read_end_);
        saved_ = -1;
        return content;
    }

private:
    int saved_ = -1;
    int read_end_ = -1;
};

} // namespace

// ── 在 HCS 里 ─────────────────────────────────────────────────────────────

TEST(LibhcsLogBridge, IsInstalledAtLoadTime) {
    // 没有人调过任何"安装"函数。把出口摘下来，摘到的应当正是桥；析构时原样放回去。
    const WithoutTheBridge detached;
    EXPECT_NE(detached.bridge(), nullptr);
}

TEST(LibhcsLogBridge, SdkLinesArriveThroughHcsLogAndStayOffStderr) {
    CaptureProcessLog capture;
    StderrPipe stderr_pipe;

    const sdk::Logger board{"AF-90A7"};
    board.error("USB link faulted: {} ({})", -4, "LIBUSB_ERROR_NO_DEVICE");
    board.warn("Reconnect: the board is not back yet (x{})", 8);
    sdk::get_logger().info("libusb dev_mem (zero-copy) available");
    sdk::get_logger().critical(std::string_view{"raw text with {} braces"});

    const auto lines = capture.flushed();
    const auto written_to_stderr = stderr_pipe.restore();

    ASSERT_EQ(lines.size(), 4u);

    // 针对某块板的行，名字里带着它的序列号；不针对哪块板的就叫 "libhcs"。
    EXPECT_EQ(lines[0].logger, "libhcs.AF-90A7");
    EXPECT_EQ(lines[0].level, Level::kError);
    EXPECT_EQ(lines[0].text, "USB link faulted: -4 (LIBUSB_ERROR_NO_DEVICE)");
    EXPECT_EQ(lines[1].logger, "libhcs.AF-90A7");
    EXPECT_EQ(lines[1].level, Level::kWarn);
    EXPECT_EQ(lines[1].text, "Reconnect: the board is not back yet (x8)");
    EXPECT_EQ(lines[2].logger, "libhcs");
    EXPECT_EQ(lines[2].level, Level::kInfo);
    EXPECT_EQ(lines[2].text, "libusb dev_mem (zero-copy) available");
    EXPECT_EQ(lines[3].logger, "libhcs");
    EXPECT_EQ(lines[3].level, Level::kFatal);
    EXPECT_EQ(lines[3].text, "raw text with {} braces");

    // 目的所在：SDK 的线程不再自己往 stderr 写。
    EXPECT_EQ(written_to_stderr, "");
}

// 两块板的日志分得开，各自的顺序不乱。
TEST(LibhcsLogBridge, LinesOfDifferentBoardsCarryDifferentNames) {
    CaptureProcessLog capture;
    const sdk::Logger gimbal{"AF-90A7"};
    const sdk::Logger chassis{"AF-958F"};
    for (int i = 0; i < 3; ++i) {
        gimbal.warn("gimbal {}", i);
        chassis.warn("chassis {}", i);
    }

    const auto lines = capture.flushed();
    ASSERT_EQ(lines.size(), 6u);
    for (int i = 0; i < 3; ++i) {
        const auto& from_gimbal = lines[static_cast<std::size_t>(2 * i)];
        const auto& from_chassis = lines[static_cast<std::size_t>(2 * i + 1)];
        EXPECT_EQ(from_gimbal.logger, "libhcs.AF-90A7");
        EXPECT_EQ(from_gimbal.text, "gimbal " + std::to_string(i));
        EXPECT_EQ(from_chassis.logger, "libhcs.AF-958F");
        EXPECT_EQ(from_chassis.text, "chassis " + std::to_string(i));
    }
}

// 出口的契约没有给序列号定上限。来一个比日志口名字还长的，名字截断，不许越界。
TEST(LibhcsLogBridge, AnOverlongSourceIsTruncatedNotOverrun) {
    CaptureProcessLog capture;
    const WithoutTheBridge detached; // 借出桥本身，直接喂它一条
    ASSERT_NE(detached.bridge(), nullptr);

    const std::string serial(200, 'S');
    detached.bridge()->write(
        sdk::Record{.level = sdk::Level::kWarn, .source = serial, .message = "text"});

    const auto lines = capture.flushed();
    ASSERT_EQ(lines.size(), 1u);
    const std::string expected =
        "libhcs." + std::string(hcs_log::Record::kNameCapacity - std::string_view{"libhcs."}.size(), 'S');
    EXPECT_EQ(lines[0].logger, expected);
    EXPECT_EQ(lines[0].text, "text");
}

// 低于 SDK 编译期级别（默认 kInfo）的那些在 SDK 里就被拦下了，到不了桥。
TEST(LibhcsLogBridge, LinesBelowTheSdkLevelNeverReachTheBridge) {
    CaptureProcessLog capture;
    CaptureProcessLog::backend().set_threshold(Level::kDebug);
    sdk::get_logger().debug("hidden {}", 1);
    sdk::get_logger().trace("hidden {}", 2);
    sdk::get_logger().info("shown");
    const auto lines = capture.flushed();
    CaptureProcessLog::backend().set_threshold(Level::kInfo);

    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0].text, "shown");
}

// SDK 的一行最长约 1000 字节，hcs_log 一条记录的定长载荷更短。打日志的是 IO 线程，
// 不许为了一行长日志去分配：截断，并且看得出截过。
TEST(LibhcsLogBridge, OverlongLinesAreTruncatedNotSpilled) {
    CaptureProcessLog capture;
    const std::string reason(800, 'x');
    sdk::get_logger().error("fault: {}", reason);

    const auto lines = capture.flushed();
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0].text.size(), hcs_log::Record::kPayloadCapacity);
    EXPECT_TRUE(lines[0].text.starts_with("fault: xxxx"));
    EXPECT_TRUE(lines[0].text.ends_with("x..."));
}

// ScopedSink：活着的时候接管，析构时把原来的放回去——包括原来就有人接着的情况。
TEST(LibhcsLogBridge, ScopedSinkPutsThePreviousSinkBack) {
    struct Counting final : sdk::Sink {
        void write(const sdk::Record& record) noexcept override {
            ++count;
            last_source = std::string{record.source};
        }
        int count = 0;
        std::string last_source;
    };

    CaptureProcessLog capture;
    Counting counting;
    {
        const sdk::ScopedSink scope{counting};
        const sdk::Logger board{"AF-90A7"};
        board.warn("to the scoped sink");
        EXPECT_EQ(counting.count, 1);
        EXPECT_EQ(counting.last_source, "AF-90A7"); // 出口拿得到是哪块板，不必自己去猜
        EXPECT_TRUE(capture.flushed().empty());
    }
    sdk::get_logger().warn("back to the bridge");

    EXPECT_EQ(counting.count, 1);
    const auto lines = capture.flushed();
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0].text, "back to the bridge");
}

// 出口可以在任何时刻、任何线程上换，哪怕别的线程正在打日志：每一行要么整行走了旧的去处，
// 要么整行走了新的，一行不多、一行不少、没有半行（TSan 下这个测试才真正有牙齿）。
TEST(LibhcsLogBridge, SwappingTheSinkWhileOtherThreadsLogLosesNothing) {
    constexpr int kThreads = 4;
    constexpr int kPerThread = 100; // 合计远小于 hcs_log 的队列容量和管道容量：一行都不许丢

    CaptureProcessLog capture;
    StderrPipe stderr_pipe;
    const std::uint64_t dropped_before = sdk::stderr_lines_dropped();

    std::atomic<bool> go{false};
    std::atomic<int> finished{0};
    std::vector<std::thread> threads;
    for (int id = 0; id < kThreads; ++id)
        threads.emplace_back([&, id] {
            const sdk::Logger board{std::string{"BOARD-"} + std::to_string(id)};
            while (!go.load())
                std::this_thread::yield();
            for (int i = 0; i < kPerThread; ++i)
                board.warn("line {}", i);
            finished.fetch_add(1);
        });

    // 主线程在它们打日志的同时反复把出口摘下来再放回去。
    sdk::Sink* const bridge = sdk::set_sink(nullptr);
    ASSERT_NE(bridge, nullptr);
    go.store(true);
    for (bool attached = false; finished.load() < kThreads; attached = !attached) {
        (void)sdk::set_sink(attached ? nullptr : bridge);
        std::this_thread::yield();
    }
    for (auto& thread : threads)
        thread.join();
    (void)sdk::set_sink(bridge);

    const auto through_hcs_log = capture.flushed();
    const std::string on_stderr = stderr_pipe.restore();

    std::size_t stderr_lines = 0;
    for (std::size_t at = 0; (at = on_stderr.find('\n', at)) != std::string::npos; ++at)
        ++stderr_lines;

    EXPECT_EQ(sdk::stderr_lines_dropped(), dropped_before);
    EXPECT_EQ(through_hcs_log.size() + stderr_lines, static_cast<std::size_t>(kThreads * kPerThread));

    // 走 stderr 的每一行都是完整的一行：前缀、哪块板、文本。
    for (std::size_t begin = 0; begin < on_stderr.size();) {
        const std::size_t end = on_stderr.find('\n', begin);
        const std::string_view line{on_stderr.data() + begin, end - begin};
        EXPECT_TRUE(line.starts_with("[libhcs] [warn] [BOARD-")) << line;
        EXPECT_NE(line.find("] line "), std::string_view::npos) << line;
        begin = end + 1;
    }
    for (const auto& line : through_hcs_log) {
        EXPECT_TRUE(line.logger.starts_with("libhcs.BOARD-")) << line.logger;
        EXPECT_TRUE(line.text.starts_with("line ")) << line.text;
    }
}

// ── SDK 单独用：没有人接它的出口 ──────────────────────────────────────────

// 行的格式：带前缀、带换行、直接写 stderr；针对某块板的行多一段序列号。
// （这是 SDK 自带的示例、别的使用者看到的行为。）
TEST(LibhcsStderr, WithoutASinkTheSdkWritesItsOwnLinesToStderr) {
    CaptureProcessLog capture;
    const WithoutTheBridge detached;
    StderrPipe stderr_pipe;

    sdk::get_logger().warn("no board in particular {}", 1);
    sdk::Logger{"AF-90A7"}.error("about one board {}", 2);

    EXPECT_EQ(
        stderr_pipe.restore(), "[libhcs] [warn] no board in particular 1\n"
                               "[libhcs] [error] [AF-90A7] about one board 2\n");
    EXPECT_TRUE(capture.flushed().empty());
}

// 整件事的起因：stderr 那头的读者不动了。SDK 不许等它——等的那条线程是 USB 事件线程。
// 写不进去的行丢掉、计数；读者追上来之后，第一行之前先报丢了多少。
TEST(LibhcsStderr, AFullStderrPipeDropsLinesInsteadOfBlockingTheCaller) {
    constexpr std::uint64_t kLines = 100;

    const WithoutTheBridge detached;
    StderrPipe stderr_pipe;
    stderr_pipe.fill();
    if (::testing::Test::HasFatalFailure())
        return;

    const std::uint64_t dropped_before = sdk::stderr_lines_dropped();
    const auto start = std::chrono::steady_clock::now();
    for (std::uint64_t i = 0; i < kLines; ++i)
        sdk::get_logger().error("USB link faulted (x{})", i);
    const auto elapsed = std::chrono::steady_clock::now() - start;

    // 真阻塞的话根本走不到这里（管道没人读）。上限给得极宽，只为把话说完整。
    EXPECT_LT(elapsed, 1s);
    EXPECT_EQ(sdk::stderr_lines_dropped() - dropped_before, kLines);

    // 读者追上来了。灌进去的填充读走，管道空出来。
    const std::string filler = stderr_pipe.drain();
    EXPECT_EQ(filler.find("libhcs"), std::string::npos); // 丢掉的那些一个字节都没进去

    sdk::get_logger().info("stderr is back");
    EXPECT_EQ(
        stderr_pipe.restore(), "[libhcs] [warn] 100 log line(s) dropped: stderr was not writable\n"
                               "[libhcs] [info] stderr is back\n");
    EXPECT_EQ(sdk::stderr_lines_dropped() - dropped_before, kLines); // 恢复之后不再丢
}
