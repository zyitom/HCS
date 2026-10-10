#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include <hcs_base/channel/event_queue.hpp>
#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/doorbell.hpp>
#include <hcs_base/thread/doorbell_worker.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_base/thread/thread_config.hpp>
#include <hcs_executor/component.hpp>

#include "controller/shooting/shooting_statistics.hpp"

namespace hcs_core::controller::shooting {

// 弹速记录：每打一发（或者定时）把裁判系统报的初速记下来，连同最近若干发的统计写进文件、
// 打进日志。调摩擦轮一致性用的。移植自 RMCS 的 shooting_recorder.cpp。
//
// 参数：
//   log_mode        1 = 每打一发记一次（裁判系统的发射时间戳变了）；2 = 每 100 ms 记一次
//   log_directory   记录文件放哪，文件名是启动时刻的毫秒数。默认 /robot_shoot（与 RMCS 相同）
//   velocity_file   只记初速、一行一个的那份文件。默认是当前目录下的 shoot_recorder
//   thread_config   写文件的线程的核与优先级（缺省 SCHED_OTHER，线程名 hcs-shoot-rec）
//
// 这个组件横跨两个域，两个域之间只走一条队列：
//
//   周期域（控制线程，update()）    只判断"这一拍要不要记"，要记就把两个数塞进队列
//   尽力域（记录线程）              拍尾被门铃叫醒，取出样本，算统计、写文件、打日志
//
// RMCS 里这些事全在 update() 里做：排序、格式化、每一发开一次文件再关上。那在 HCS 的周期域里
// 是不允许的（会分配、会进内核、会等磁盘），所以挪到了记录线程上。
//
// 和 RMCS 不一样的地方：
//   - 统计窗口（最近 1000 发）按到达顺序滑动。RMCS 是在原数组上排序之后再删第一个，
//     超过 1000 发以后删掉的其实是最小的那个，而不是最早的那个。
//   - 不管有几个摩擦轮都写记录。RMCS 只在 friction_wheel_count == 6 时才有内容，
//     别的车上写出来的是空行。
class ShootingRecorder
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    ShootingRecorder()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , log_mode_(static_cast<LogMode>(get_parameter("log_mode").as_int()))
        , recorder_thread_{
              hcs_utility::ThreadConfig{param<std::string>("thread_config", ""), "hcs-shoot-rec"},
              [this] { record_pending(); }} {
        if (const auto& error = recorder_thread_.thread_config_error(); !error.empty())
            logger().warn("Failed to apply thread_config: {}", error);

        register_input("/referee/shooter/initial_speed", initial_speed_);
        register_input("/referee/shooter/shoot_timestamp", shoot_timestamp_);

        using namespace std::chrono;
        const auto started_ms =
            duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        const auto log_file =
            std::filesystem::path{param<std::string>("log_directory", "/robot_shoot")}
            / std::format("{}.log", started_ms);
        log_stream_.open(log_file);
        if (log_stream_)
            logger().info("ShootingRecorder initialized, log file: {}", log_file.string());
        else
            logger().warn(
                "ShootingRecorder cannot open {}; statistics go to the log only",
                log_file.string());

        velocity_file_ = param<std::string>("velocity_file", "shoot_recorder");
        std::ofstream{velocity_file_}; // 每次启动清空
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        switch (log_mode_) {
        case LogMode::kTrigger:
            // 裁判系统每报一次发射，时间戳变一次。
            if (*shoot_timestamp_ == last_shoot_timestamp_)
                return;
            break;
        case LogMode::kTiming:
            // 10 Hz。第一拍就记一次。
            if (since_last_record_ < kTimingPeriod) {
                since_last_record_ += tick.dt;
                return;
            }
            since_last_record_ = tick.dt;
            break;
        }

        // 队列满了就丢这一条（EventQueue 自己计数，由记录线程报），绝不等。
        samples_.try_push(Sample{.initial_speed = *initial_speed_});
        last_shoot_timestamp_ = *shoot_timestamp_;
    }

    [[nodiscard]] hcs_utility::Doorbell* tick_end_doorbell() override {
        return &recorder_thread_.doorbell();
    }

private:
    enum class LogMode { kTrigger = 1, kTiming = 2 };

    struct Sample {
        float initial_speed;
    };

    /// 最多拿最近这么多发来算统计。
    static constexpr std::size_t kWindowSize = 1000;
    static constexpr std::size_t kSampleQueueCapacity = 256;
    static constexpr hcs_sync::Duration kTimingPeriod = std::chrono::milliseconds{100};

    template <class T>
    T param(const std::string& name, T fallback) const {
        get_parameter_or(name, fallback, fallback);
        return fallback;
    }

    /// 尽力域（记录线程），每拍拍尾被叫醒一次：大多数时候队列是空的。
    void record_pending() {
        for (Sample sample; samples_.try_pop(sample);) {
            velocities_.push_back(sample.initial_speed);
            if (velocities_.size() > kWindowSize)
                velocities_.pop_front();

            sorted_.assign(velocities_.begin(), velocities_.end());
            const auto statistics = analyse(sorted_);

            const auto line = std::format(
                "{},{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}", sample.initial_speed,
                velocities_.size(), statistics.mean, statistics.excellence_rate,
                statistics.pass_rate, statistics.range, statistics.trimmed_range,
                statistics.max, statistics.min);
            if (log_stream_)
                log_stream_ << line << std::endl;
            logger().write(hcs_log::Level::kInfo, line);

            if (std::ofstream out{velocity_file_, std::ios::app})
                out << std::format("{:.3f}", sample.initial_speed) << std::endl;
        }

        if (const auto dropped = samples_.dropped(); dropped != reported_dropped_) {
            logger().warn(
                "Recording fell behind the control loop, {} shots not recorded.",
                dropped - reported_dropped_);
            reported_dropped_ = dropped;
        }
    }

    // ── 周期域：只有控制线程碰 ────────────────────────────────────────────
    InputInterface<float> initial_speed_;
    InputInterface<double> shoot_timestamp_;

    const LogMode log_mode_;
    double last_shoot_timestamp_ = 0;
    hcs_sync::Duration since_last_record_ = kTimingPeriod;

    // ── 跨域通道 ──────────────────────────────────────────────────────────
    hcs_sync::EventQueue<Sample, kSampleQueueCapacity> samples_; ///< 周期域 → 记录线程

    // ── 尽力域：只有记录线程碰（构造函数里的初始化发生在线程能看到样本之前）──
    std::ofstream log_stream_;
    std::string velocity_file_;
    std::deque<double> velocities_; ///< 最近 kWindowSize 发，按到达顺序
    std::vector<double> sorted_;    ///< 算统计用的那份拷贝，留着复用
    std::uint64_t reported_dropped_ = 0;

    /// 最后声明、最先析构：先停线程，它用到的东西才开始析构。
    hcs_utility::DoorbellWorker<std::move_only_function<void()>> recorder_thread_;
};

} // namespace hcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::controller::shooting::ShootingRecorder, hcs_executor::Component)
