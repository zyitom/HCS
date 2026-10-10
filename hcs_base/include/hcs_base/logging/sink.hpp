#pragma once

#include <chrono>
#include <cstdio>
#include <format>
#include <iterator>
#include <string>
#include <string_view>

#include "hcs_base/logging/record.hpp"

namespace hcs_log {

/// 交到输出端手上的一条日志：文本已经生成好。视图只在 write() 调用期间有效。
struct Entry {
    Level level;
    std::string_view logger;
    std::string_view text;
};

/// 日志最终去哪。只在日志线程上被调用，所以实现**可以阻塞**（写管道、写文件、进 DDS）
/// ——这正是把输出收到一条线程上的意义：堵住的只会是它。
///
/// 实现不需要自己加锁：Backend 保证同一时刻只有一次 write()。
class Sink {
public:
    Sink() = default;
    Sink(const Sink&) = delete;
    Sink& operator=(const Sink&) = delete;
    virtual ~Sink() = default;

    virtual void write(const Entry& entry) = 0;
};

/// 写 stderr，一条日志一次 fwrite。格式照着 ROS 控制台的来，换输出端时肉眼看不出差别：
///
///     [WARN] [1790844890.711878019] [value_broadcaster]: text
///
/// 不依赖 ROS：单测、不带 executor 的小程序、以及进程收尾（rclcpp 已经关了）时用它。
class StderrSink final : public Sink {
public:
    void write(const Entry& entry) override {
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
        const auto nanoseconds =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now - seconds);

        line_.clear();
        std::format_to(
            std::back_inserter(line_), "[{}] [{}.{:09}] [{}]: {}\n", to_string(entry.level),
            seconds.count(), nanoseconds.count(), entry.logger, entry.text);
        std::fwrite(line_.data(), 1, line_.size(), stderr);
    }

private:
    std::string line_; ///< 复用，稳态下不再分配
};

} // namespace hcs_log
