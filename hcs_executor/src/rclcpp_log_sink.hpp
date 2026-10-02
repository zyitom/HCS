#pragma once

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <hcs_base/logging/backend.hpp>
#include <hcs_base/logging/sink.hpp>

namespace hcs_executor {

/// 把日志交给 rclcpp：控制台、~/.ros/log 下的文件、/rosout，以及按 logger 名字调级别，
/// 这些 ROS 工具照常能用。
///
/// rclcpp 的日志是同步的、全进程共用一把锁——这正是不让别的线程直接调它的原因。
/// 这里只有日志线程一条线程在调，所以它再怎么堵，堵住的也只是日志线程。
///
/// **/rosout 认的是名字。** rcl 只替两种 logger 往 /rosout 上发：名字和某个节点的 logger
/// 相同的，以及从节点 logger 上 get_child() 出来的。一个凭空的 rclcpp::get_logger("x")
/// 控制台和文件里都有，/rosout 上没有，而且不报错（实测，jazzy）。所以名字分三种处理：
///
///   - 本身就是节点的组件：用那个节点自己的 logger，名字原样（`[value_broadcaster]`）。
///   - 其余的（不是节点的组件、"libhcs" 这样的库）：挂到 root 下面，
///     名字变成 `[hcs_executor.libhcs]`，这样才上得了 /rosout。
///   - 还没有 root 的时候（节点建起来之前）：凭空的 logger。那会儿本来也没有 /rosout。
class RclcppSink final : public hcs_log::Sink {
public:
    /// 日志口的名字 → 这个名字对应的节点的 logger。
    using NodeLoggers = std::map<std::string, rclcpp::Logger, std::less<>>;

    RclcppSink() = default;

    RclcppSink(rclcpp::Logger root, NodeLoggers node_loggers)
        : root_(std::move(root))
        , loggers_(std::move(node_loggers)) {}

    void write(const hcs_log::Entry& entry) override {
        const rclcpp::Logger& logger = logger_for(entry.logger);
        const auto size = static_cast<int>(entry.text.size());
        const char* text = entry.text.data();

        using hcs_log::Level;
        switch (entry.level) {
        case Level::kDebug: RCLCPP_DEBUG(logger, "%.*s", size, text); break;
        case Level::kInfo: RCLCPP_INFO(logger, "%.*s", size, text); break;
        case Level::kWarn: RCLCPP_WARN(logger, "%.*s", size, text); break;
        case Level::kError: RCLCPP_ERROR(logger, "%.*s", size, text); break;
        case Level::kFatal: RCLCPP_FATAL(logger, "%.*s", size, text); break;
        }
    }

private:
    /// rclcpp::get_logger / get_child 每次都要拼名字、查表、拿锁；logger 的名字就那么几十个，
    /// 记下来。节点的那几个在构造时就已经放进来了，这里只会为"其余的"新建。
    const rclcpp::Logger& logger_for(std::string_view name) {
        auto found = loggers_.find(name);
        if (found == loggers_.end())
            found = loggers_.emplace(std::string{name}, make_logger(std::string{name})).first;
        return found->second;
    }

    [[nodiscard]] rclcpp::Logger make_logger(const std::string& name) {
        if (!root_)
            return rclcpp::get_logger(name);
        try {
            return root_->get_child(name);
        } catch (...) {
            // 往 /rosout 上登记失败（rcl 报错）。控制台和文件不依赖它，退回凭空的 logger。
            return rclcpp::get_logger(name);
        }
    }

    std::optional<rclcpp::Logger> root_;
    NodeLoggers loggers_;
};

/// 在它活着的这段时间里，日志走 rclcpp；之前和之后走原来的输出端（stderr）。
///
/// 三个时刻要紧：
///   - 节点和组件都建好**之后**调 attach()：不是节点的名字要挂到一个节点下面才上得了 /rosout。
///   - rclcpp::shutdown() **之前**调 restore()：关掉之后再走 rclcpp 的日志是没人保证的。
///   - 组件库被卸载**之前**析构：周期域发的日志记录里存着组件库里的函数地址
///     （见 hcs_log::Record），库没了再去排空队列就是调野指针。所以它要声明在
///     pluginlib::ClassLoader 之后——后声明的先析构。
class RclcppLogScope {
public:
    explicit RclcppLogScope(hcs_log::Backend& backend)
        : backend_(backend)
        , previous_(backend.exchange_sink(std::make_unique<RclcppSink>())) {
        // rclcpp 自己会按 logger 的级别再筛一遍，这里不拦 DEBUG，免得 ros2 的调级别工具失效。
        backend_.set_threshold(hcs_log::Level::kDebug);
    }

    ~RclcppLogScope() {
        restore();
        backend_.flush();
    }

    RclcppLogScope(const RclcppLogScope&) = delete;
    RclcppLogScope& operator=(const RclcppLogScope&) = delete;

    /// 节点和组件都建好之后调一次：从这里起，不是节点的那些名字挂到 root 下面，
    /// 它们的日志才上得了 /rosout（见 RclcppSink）。之前入队的日志仍然按原来的名字走。
    void attach(rclcpp::Logger root, RclcppSink::NodeLoggers node_loggers) {
        if (previous_ == nullptr)
            return; // 已经 restore 过了，输出端不再是 rclcpp 的
        backend_.exchange_sink(
            std::make_unique<RclcppSink>(std::move(root), std::move(node_loggers)));
    }

    /// 幂等。换回原来的输出端；换之前已经入队的日志仍然走 rclcpp。
    void restore() {
        if (previous_ == nullptr)
            return;
        backend_.exchange_sink(std::move(previous_));
        backend_.set_threshold(hcs_log::Level::kInfo);
    }

private:
    hcs_log::Backend& backend_;
    std::unique_ptr<hcs_log::Sink> previous_;
};

} // namespace hcs_executor
