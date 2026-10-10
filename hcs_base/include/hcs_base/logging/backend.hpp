#pragma once

#include <pthread.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <format>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

#include "hcs_base/channel/mpsc_queue.hpp"
#include "hcs_base/logging/record.hpp"
#include "hcs_base/logging/sink.hpp"
#include "hcs_base/thread/rt_attributes.hpp"

namespace hcs_log {

/// 日志的后半段：一条队列、一条线程、一个输出端。
///
///     任何线程 ──submit()──▶ MpscQueue<Record> ──日志线程──▶ Sink
///
/// 存在的理由是把"输出会阻塞"这件事关进一条线程里。stderr 接的是管道（launch、
/// journald），读的一方跟不上时 write 就会卡住；ROS 自己的日志是在**调用线程**上
/// 同步做这件事的，还拿着一把全局锁。走这里的话，卡住的只有日志线程：别的线程
/// 入队失败就丢一条、记个数，谁也不等。
///
/// 它是一个普通的类，不是单例：单测各建各的。进程里"那一个"由 hcs_executor 提供
/// （hcs_executor::process_log_backend()），这个包自己不放任何全局状态。
///
/// 日志线程定时去看队列，而不是被生产者叫醒：叫醒要进内核（futex），而生产者里有
/// 周期域的线程。代价是一条日志最多晚 poll_period 才出现，换来的是 submit() 对任何
/// 线程都只是一次无锁入队。
class Backend {
public:
    static constexpr std::size_t kQueueCapacity = 1024;

    explicit Backend(
        std::unique_ptr<Sink> sink = std::make_unique<StderrSink>(),
        std::chrono::milliseconds poll_period = std::chrono::milliseconds{5})
        : sink_(std::move(sink))
        , poll_period_(poll_period)
        , worker_([this](std::stop_token stop) { run(std::move(stop)); }) {}

    /// 停线程之前把队列里剩下的全部写完（worker_ 最后声明、最先析构）。
    /// 这也是溢出文本不泄漏的保证：队列里每一条记录最终都会经过 drain()。
    ~Backend() = default;

    Backend(const Backend&) = delete;
    Backend& operator=(const Backend&) = delete;

    /// 任何线程、任何域。不阻塞、不分配、不进内核；队列满了返回 false 并计数。
    bool submit(const Record& record) noexcept HCS_NONBLOCKING { return queue_.try_push(record); }

    /// 低于这个级别的日志在调用点就被拦下，连入队都不发生。
    [[nodiscard]] Level threshold() const noexcept HCS_NONBLOCKING {
        return threshold_.load(std::memory_order_relaxed);
    }
    void set_threshold(Level level) noexcept { threshold_.store(level, std::memory_order_relaxed); }

    /// 尽力域。等到调用之前已经入队的日志全部交给了输出端才返回。
    ///
    /// 输出端卡死的话它也跟着卡死，所以只用在"宁可等也不能少"的地方（卸组件库之前）。
    /// 别的地方用 flush_for()。在日志线程自己身上调是空操作，理由见 flush_for()。
    void flush() {
        std::unique_lock lock{state_mutex_};
        if (on_log_thread())
            return;
        const std::uint64_t target = ++flush_requested_;
        wake_.notify_all();
        flushed_.wait(lock, [&] { return flush_completed_ >= target; });
    }

    /// 同 flush()，但最多等 timeout。返回 false：到点了还没写完（输出端堵着）。
    ///
    /// 给"进程快没了，顺手把日志带出去"的路径用（terminate 处理函数）：那里宁可少几行，
    /// 也不能被一根堵住的管道挂着不死——进程不死，板子就等不到断连，电机停不下来。
    ///
    /// 在日志线程自己身上调（也就是从输出端的 write() 里）立刻返回 false：它要等的正是自己。
    [[nodiscard]] bool flush_for(std::chrono::milliseconds timeout) {
        std::unique_lock lock{state_mutex_};
        if (on_log_thread())
            return false;
        const std::uint64_t target = ++flush_requested_;
        wake_.notify_all();
        return flushed_.wait_for(lock, timeout, [&] { return flush_completed_ >= target; });
    }

    /// 尽力域。换输出端，返回原来那个。先 flush，所以换之前入队的日志都走旧的输出端。
    std::unique_ptr<Sink> exchange_sink(std::unique_ptr<Sink> sink) {
        flush();
        const std::scoped_lock lock{sink_mutex_};
        return std::exchange(sink_, std::move(sink));
    }

    /// 因为队列满而丢掉的条数。
    [[nodiscard]] std::uint64_t dropped() const noexcept { return queue_.dropped(); }

    /// 输出端抛了异常、因而没写出去的条数。日志自己出的毛病没法再走日志报，只能留个数。
    [[nodiscard]] std::uint64_t sink_failures() const noexcept {
        return sink_failures_.load(std::memory_order_relaxed);
    }

private:
    void run(std::stop_token stop) {
        ::pthread_setname_np(::pthread_self(), "hcs-log");

        std::unique_lock state{state_mutex_};
        log_thread_ = std::this_thread::get_id();
        for (;;) {
            // 两样都在排空**之前**读：排空之后才来的 flush 请求、才置上的停机标志，
            // 都留给下一趟——那一趟才看得全在它们之前入队的记录。
            const std::uint64_t request = flush_requested_;
            const bool stopping = stop.stop_requested();

            // 输出端可能堵很久，这期间不拿 state_mutex_：flush_for() 要能按时放弃。
            state.unlock();
            {
                const std::scoped_lock sink_lock{sink_mutex_};
                drain();
            }
            state.lock();

            flush_completed_ = request;
            flushed_.notify_all();

            // 析构时的最后一趟：停机标志是在这趟排空之前看到的，所以析构之前入队的都写完了。
            if (stopping)
                return;
            wake_.wait_for(
                state, stop, poll_period_, [&] { return flush_requested_ != flush_completed_; });
        }
    }

    /// 持有 state_mutex_ 时调。
    [[nodiscard]] bool on_log_thread() const noexcept {
        return log_thread_ == std::this_thread::get_id();
    }

    /// 日志线程，持有 sink_mutex_。输出端抛异常只计数：日志不许把进程带走。
    void drain() {
        for (Record record; queue_.try_pop(record);) {
            // 溢出文本的所有权在入队那一刻就到了这里：先接住，后面怎么抛都不漏。
            const std::unique_ptr<char[]> spilled{record.spilled ? record.spill().data : nullptr};
            try {
                if (record.spilled)
                    write(record, std::string_view{spilled.get(), record.spill().size});
                else if (record.render == nullptr)
                    write(record, record.text());
                else
                    write(record, record.render(record.format, record.payload.data()));
            } catch (...) {
                sink_failures_.fetch_add(1, std::memory_order_relaxed);
            }
        }
        report_dropped();
    }

    void write(const Record& record, std::string_view text) {
        sink_->write(Entry{record.level, record.logger_name(), text});
    }

    /// 丢弃是显式策略，但不许是静默的。最多每秒报一次，免得"报丢弃"本身变成洪水。
    void report_dropped() {
        const std::uint64_t dropped = queue_.dropped();
        const auto now = std::chrono::steady_clock::now();
        if (dropped == reported_dropped_ || now - last_drop_report_ < std::chrono::seconds{1})
            return;

        try {
            scratch_ = std::format(
                "{} log records dropped: the queue was full", dropped - reported_dropped_);
            sink_->write(Entry{Level::kWarn, "hcs_log", scratch_});
        } catch (...) {
            sink_failures_.fetch_add(1, std::memory_order_relaxed);
        }
        reported_dropped_ = dropped;
        last_drop_report_ = now;
    }

    hcs_sync::MpscQueue<Record, kQueueCapacity> queue_;
    std::atomic<Level> threshold_{Level::kInfo};

    std::atomic<std::uint64_t> sink_failures_{0};

    // 两把锁，从不同时持有。生产者哪一把都不碰。
    //
    // ── state_mutex_：flush 的记账。只包几条赋值，谁也不会在它上面等很久。─────────
    std::mutex state_mutex_;
    std::condition_variable_any wake_;    ///< 叫日志线程：有 flush 请求，或者该停了
    std::condition_variable flushed_;     ///< 叫 flush() 的等待者
    std::uint64_t flush_requested_ = 0;
    std::uint64_t flush_completed_ = 0;
    std::thread::id log_thread_{};        ///< 日志线程起来后写一次

    // ── sink_mutex_：输出端。日志线程整趟排空都拿着它，输出端堵多久它就被拿多久。───
    std::mutex sink_mutex_;
    std::unique_ptr<Sink> sink_;
    std::string scratch_;
    std::uint64_t reported_dropped_ = 0;
    std::chrono::steady_clock::time_point last_drop_report_{};

    std::chrono::milliseconds poll_period_;

    /// 最后声明：构造时其余成员都已就绪，析构时先停先 join。
    std::jthread worker_;
};

} // namespace hcs_log
