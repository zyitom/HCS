#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <format>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "hcs_utility/rt_sampler.hpp"
#include "hcs_utility/thread_config.hpp"

#include "tdigest.hpp"

// 与 executor.hpp 同一个开关，取值而不是取"是否定义"：CMake 关掉时也可能定义成 0。
#ifndef HCS_EXECUTOR_COMPONENT_TIMING
#    define HCS_EXECUTOR_COMPONENT_TIMING 0
#endif

namespace hcs_executor {

/// 采样报告线程。SCHED_OTHER，绑到"除 RT 核以外的全部核"。
///
/// 为什么整段统计要从控制线程搬到这里：改造前 executor 每 5 秒在 RT 线程里跑
/// TDigest::merge()（256 点排序归并）＋ RCLCPP_INFO（格式化 ＋ 分配 ＋ write 系统调用），
/// 而报告周期正好 5000 拍——README 里那个 max 583us 就出现在 5000 拍的样本里。
/// 日志是被测对象的一部分，测不准的不是抖动而是这条打印。搬走它既是 RT 修复，
/// 也是后面所有测量可信的前提：RT 线程只往环里塞裸样本，merge / 格式化 / 打印
/// 全发生在这条尽力域（D3）线程上——它没有截止期，允许阻塞、分配、打日志。
///
/// 统计口径与格式与改造前逐字段等价（README 里那两行输出必须继续成立）。
class RtReporter {
public:
    RtReporter(
        rclcpp::Logger logger, hcs_utility::RtSampler& sampler,
        std::vector<std::string> component_names, std::chrono::seconds report_period,
        hcs_utility::ThreadConfig thread_config)
        : logger_(std::move(logger))
        , sampler_(sampler)
        , component_names_(std::move(component_names))
        // 非正周期会让"跳到下一个报告点"的补齐循环空转，钳一下
        , report_period_(std::max(report_period, std::chrono::seconds{1}))
        , thread_config_(std::move(thread_config))
        , window_(component_names_.size()) {}

    ~RtReporter() { stop(); }

    RtReporter(const RtReporter&) = delete;
    RtReporter& operator=(const RtReporter&) = delete;
    RtReporter(RtReporter&&) = delete;
    RtReporter& operator=(RtReporter&&) = delete;

    void start() {
        if (thread_.joinable())
            return;
        {
            auto lock = std::lock_guard{mutex_};
            stop_requested_ = false;
        }
        thread_ = std::thread{[this]() { main(); }};
    }

    void stop() noexcept {
        {
            auto lock = std::lock_guard{mutex_};
            stop_requested_ = true;
        }
        condition_.notify_all();
        if (thread_.joinable())
            thread_.join();
    }

private:
    using SteadyClock = std::chrono::steady_clock;

    static constexpr size_t digest_size_ = 256;

    /// 掏环的节奏。比报告周期密得多，否则 RT 侧的环会被写满。
    static constexpr auto drain_period_ = std::chrono::milliseconds{100};

    struct CumulativeStats {
        TDigest<double> start_lateness_ms{digest_size_};
        TDigest<double> update_duration_ms{digest_size_};
        uint64_t update_count = 0;
        uint64_t skipped_count = 0;
        uint64_t start_lateness_sample_count = 0;
        uint64_t update_duration_sample_count = 0;
        double start_lateness_max_ms = 0.0;
        double update_duration_max_ms = 0.0;
    };

    struct WindowStats {
        explicit WindowStats(size_t component_count)
            : component_update_duration_sums(component_count) {}

        uint64_t executed_count = 0;
        /// 其中带逐组件采样的拍数。逐组件是按 stride 抽样的，占比只能用这一批当分母，
        /// 拿总拍数去除会把百分比稀释成 1/stride。
        uint64_t component_sampled_ticks = 0;
        SteadyClock::duration total_update_duration{};
        std::vector<SteadyClock::duration> component_update_duration_sums;
    };

    struct ComponentWindowStat {
        size_t index;
        SteadyClock::duration duration_sum;
    };

    void main() {
        // 尽力域：绑核 / nice 设不上不该拉整机，警告一句继续跑
        if (const auto result = thread_config_.apply_to_current_thread(); !result)
            RCLCPP_WARN(logger_, "%s", result.error().c_str());

        auto next_report_time = SteadyClock::now() + report_period_;

        while (true) {
            {
                auto lock = std::unique_lock{mutex_};
                condition_.wait_for(lock, drain_period_, [this]() { return stop_requested_; });
                if (stop_requested_)
                    break;
            }

            drain_samples();

            const auto current_time = SteadyClock::now();
            if (current_time < next_report_time)
                continue;

            log_due_report();
            // 被饿住时不补打历史窗口（窗口已经被合并了），直接跳到下一个报告点
            do {
                next_report_time += report_period_;
            } while (next_report_time <= current_time);
        }
    }

    // drain 回调必须 noexcept：它在环的消费循环里跑，抛出去游标就推到一半了。
    void drain_samples() {
        static_cast<void>(sampler_.drain_ticks(
            [this](const hcs_utility::TickSample& sample) noexcept { accumulate_tick(sample); }));
        static_cast<void>(
            sampler_.drain_components([this](const hcs_utility::ComponentSample& sample) noexcept {
                accumulate_component(sample);
            }));
    }

    void accumulate_tick(const hcs_utility::TickSample& sample) noexcept {
        // 口径与改造前 calculate_next_iteration_time 里的 update_count += skipped_cycles 一致：
        // 被跳过的拍也算进 update_count，否则 README 里的 Update/Skipped 比例会变。
        cumulative_.skipped_count += sample.skipped;
        cumulative_.update_count += 1 + static_cast<uint64_t>(sample.skipped);
        window_.executed_count++;
        failed_component_count_ = sample.failed_count;

        const auto start_lateness_ms =
            static_cast<double>(sample.start_ns - sample.scheduled_ns) / 1e6;
        if (start_lateness_ms < 0.0) {
            // 早醒进 digest 会把分位数拉偏，只计数（改造前是一条 WARN_THROTTLE）
            woke_early_count_++;
        } else {
            cumulative_.start_lateness_ms.insert(start_lateness_ms);
            cumulative_.start_lateness_max_ms =
                std::max(cumulative_.start_lateness_max_ms, start_lateness_ms);
            cumulative_.start_lateness_sample_count++;
        }

        const auto update_duration_ms = static_cast<double>(sample.end_ns - sample.start_ns) / 1e6;
        cumulative_.update_duration_ms.insert(update_duration_ms);
        cumulative_.update_duration_max_ms =
            std::max(cumulative_.update_duration_max_ms, update_duration_ms);
        cumulative_.update_duration_sample_count++;
    }

    void accumulate_component(const hcs_utility::ComponentSample& sample) noexcept {
        if (sample.component_index >= window_.component_update_duration_sums.size())
            return;

        if (sample.sequence != last_component_sequence_) {
            last_component_sequence_ = sample.sequence;
            window_.component_sampled_ticks++;
        }

        const auto duration = std::chrono::duration_cast<SteadyClock::duration>(
            std::chrono::nanoseconds{sample.duration_ns});
        window_.component_update_duration_sums[sample.component_index] += duration;
        // 逐组件计时首尾相接，其和就是该拍的 update 耗时；拿它当占比分母，
        // 抽样与否都自洽（用全部拍的 update 耗时会被稀释成 1/stride）。
        window_.total_update_duration += duration;
    }

    void log_due_report() {
        log_cumulative_stats(cumulative_);
#if HCS_EXECUTOR_COMPONENT_TIMING
        log_window_stats(window_);
#endif
        log_drop_stats();
        reset_window_stats(window_);
    }

    void log_cumulative_stats(CumulativeStats& stats) {
        stats.start_lateness_ms.merge();
        stats.update_duration_ms.merge();

        const double skipped_ratio = stats.update_count == 0
                                       ? 0.0
                                       : static_cast<double>(stats.skipped_count)
                                             / static_cast<double>(stats.update_count) * 100.0;

        RCLCPP_INFO(
            logger_,
            "Update/Skipped: %llu/%llu (%s%%), "
            "Stat ms p50/p99/max: start_late %s/%s/%s, update %s/%s/%s",
            static_cast<unsigned long long>(stats.update_count),
            static_cast<unsigned long long>(stats.skipped_count),
            format_percent(skipped_ratio).c_str(),
            format_stat(
                maybe_quantile(stats.start_lateness_ms, stats.start_lateness_sample_count, 50.0))
                .c_str(),
            format_stat(
                maybe_quantile(stats.start_lateness_ms, stats.start_lateness_sample_count, 99.0))
                .c_str(),
            format_stat(
                stats.start_lateness_sample_count == 0
                    ? std::nullopt
                    : std::optional<double>{stats.start_lateness_max_ms})
                .c_str(),
            format_stat(
                maybe_quantile(stats.update_duration_ms, stats.update_duration_sample_count, 50.0))
                .c_str(),
            format_stat(
                maybe_quantile(stats.update_duration_ms, stats.update_duration_sample_count, 99.0))
                .c_str(),
            format_stat(
                stats.update_duration_sample_count == 0
                    ? std::nullopt
                    : std::optional<double>{stats.update_duration_max_ms})
                .c_str());
    }

    void log_window_stats(const WindowStats& stats) {
        // total_update_duration 非零就意味着至少来过一条逐组件样本，
        // 下面拿 component_sampled_ticks 当除数是安全的。
        if (stats.executed_count == 0
            || stats.total_update_duration == SteadyClock::duration::zero())
            return;

        auto top_component_stats = collect_top_component_stats(stats);
        if (top_component_stats.empty())
            return;

        RCLCPP_INFO(
            logger_, "Component %llds: %s",
            static_cast<long long>(report_period_.count()),
            format_top_component_stats(top_component_stats, stats).c_str());
    }

    /// 丢样本 / 早醒 / 失效组件都是异常态，没有内容就不打这一行。
    /// 三个计数都是自启动以来的累计值，和上面那行的 Update/Skipped 同口径。
    void log_drop_stats() const {
        const auto dropped_ticks = sampler_.dropped_ticks();
        const auto dropped_components = sampler_.dropped_components();
        if (dropped_ticks == 0 && dropped_components == 0 && woke_early_count_ == 0
            && failed_component_count_ == 0)
            return;

        RCLCPP_WARN(
            logger_,
            "RT drops: ticks %llu, components %llu; woke early %llu; failed components: %u",
            static_cast<unsigned long long>(dropped_ticks),
            static_cast<unsigned long long>(dropped_components),
            static_cast<unsigned long long>(woke_early_count_),
            static_cast<unsigned>(failed_component_count_));
    }

    static std::vector<ComponentWindowStat> collect_top_component_stats(const WindowStats& stats) {
        auto top_component_stats = std::vector<ComponentWindowStat>{};
        top_component_stats.reserve(stats.component_update_duration_sums.size());
        for (size_t component_index = 0;
             component_index < stats.component_update_duration_sums.size(); ++component_index) {
            const auto duration_sum = stats.component_update_duration_sums[component_index];
            if (duration_sum == SteadyClock::duration::zero())
                continue;
            top_component_stats.emplace_back(component_index, duration_sum);
        }

        const size_t top_count = std::min<size_t>(3, top_component_stats.size());
        std::partial_sort(
            top_component_stats.begin(),
            top_component_stats.begin() + static_cast<std::ptrdiff_t>(top_count),
            top_component_stats.end(), [](const auto& lhs, const auto& rhs) {
                if (lhs.duration_sum != rhs.duration_sum)
                    return lhs.duration_sum > rhs.duration_sum;
                return lhs.index < rhs.index;
            });
        top_component_stats.resize(top_count);
        return top_component_stats;
    }

    std::string format_top_component_stats(
        const std::vector<ComponentWindowStat>& top_component_stats,
        const WindowStats& stats) const {
        auto top_components = std::string{};
        for (size_t rank = 0; rank < top_component_stats.size(); ++rank) {
            if (rank != 0)
                top_components += ", ";

            const auto& stat = top_component_stats[rank];
            top_components += std::format(
                "{} {}ms/{}%", component_names_[stat.index],
                format_stat(
                    std::optional<double>{
                        duration_to_ms(stat.duration_sum)
                        / static_cast<double>(stats.component_sampled_ticks)}),
                format_percent(
                    duration_to_ms(stat.duration_sum) / duration_to_ms(stats.total_update_duration)
                    * 100.0));
        }
        return top_components;
    }

    static void reset_window_stats(WindowStats& stats) {
        stats.executed_count = 0;
        stats.component_sampled_ticks = 0;
        stats.total_update_duration = SteadyClock::duration::zero();
        std::fill(
            stats.component_update_duration_sums.begin(),
            stats.component_update_duration_sums.end(), SteadyClock::duration::zero());
    }

    static double duration_to_ms(SteadyClock::duration duration) {
        return std::chrono::duration<double, std::milli>(duration).count();
    }

    static std::string format_stat(const std::optional<double>& value) {
        if (!value.has_value())
            return "n/a";
        return std::format("{:.3f}", *value);
    }

    static std::string format_percent(double value) { return std::format("{:.1f}", value); }

    static std::optional<double> maybe_quantile(
        const TDigest<double>& digest, uint64_t sample_count, double quantile) {
        if (sample_count == 0)
            return std::nullopt;
        return digest.quantile(quantile);
    }

    rclcpp::Logger logger_;
    hcs_utility::RtSampler& sampler_;
    std::vector<std::string> component_names_;
    std::chrono::seconds report_period_;
    hcs_utility::ThreadConfig thread_config_;

    CumulativeStats cumulative_;
    WindowStats window_;

    /// 早醒的拍不进 digest，只在这里计数。
    uint64_t woke_early_count_ = 0;
    /// 最近一拍看到的失效组件数。哪些组件失效由 executor 自己在主线程打。
    uint32_t failed_component_count_ = 0;
    /// 逐组件样本按拍成组到达，用它数"这一批抽了几拍"。
    uint64_t last_component_sequence_ = static_cast<uint64_t>(-1);

    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable condition_;
    bool stop_requested_ = false;
};

} // namespace hcs_executor
