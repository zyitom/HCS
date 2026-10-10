#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/float64.hpp>

#include <hcs_base/channel/event_queue.hpp>
#include <hcs_base/channel/snapshot.hpp>
#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/doorbell.hpp>
#include <hcs_base/thread/doorbell_worker.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_base/thread/thread_config.hpp>
#include <hcs_executor/component.hpp>

namespace hcs_core::broadcaster {

// 把图里的 double 输出按名字转发成 ROS 话题（std_msgs/Float64），给画图工具看。
//
// 参数：
//   forward_list   要转发的输出名列表，运行中可改（ros2 param set）
//   thread_config  发布线程的核与优先级（缺省 SCHED_OTHER，线程名 hcs-values）
//
// 这个组件横跨两个域，两个域之间**只走通道**，没有任何一个字段被两边同时直接碰：
//
//   周期域（控制线程，update()）
//     只做两件事：取最新的"要采哪些"，把这些输入的当前值各拷一份塞进队列。
//     不碰 publisher、不分配、不加锁。
//
//   尽力域（发布线程 + spin 线程）
//     发布线程每拍被拍尾门铃叫醒一次，把队列里的样本发出去；
//     spin 线程在 forward_list 变化时建 / 拆 publisher。这两条都不是实时线程，
//     它们之间用一把普通互斥量就够。
//
//        spin 线程 ──Snapshot<Selection>──▶ 控制线程 ──EventQueue<Sample>──▶ 发布线程
//
// 为什么每拍叫醒发布线程，而不是攒一批定时发：Float64 不带时间戳，画图工具用的是
// **收到的时刻**。攒到 10 ms 再发，曲线上就是每 10 ms 挤在一起的十个点。
class ValueBroadcaster
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    ValueBroadcaster()
        : Node{get_component_name()}
        , publisher_thread_{declare_thread_config(), [this] { publish_pending(); }} {
        if (const auto& error = publisher_thread_.thread_config_error(); !error.empty())
            logger().warn("Failed to apply thread_config: {}", error);

        declare_parameter<std::vector<std::string>>("forward_list", std::vector<std::string>{});
        parameter_subscriber_ = std::make_unique<rclcpp::ParameterEventHandler>(this);
        parameter_callback_ = parameter_subscriber_->add_parameter_callback(
            "forward_list", [this](const rclcpp::Parameter& parameter) {
                apply_forward_list(parameter.as_string_array());
            });
    }

    /// 给每个 double 输出开一条通道、注册一个输入。在接线之前、主线程上跑，
    /// 此时控制线程和发布线程都还没有东西可碰。
    void before_pairing(const OutputInfoMap& output_map) override {
        const auto is_double = [](const auto& entry) {
            return entry.second.get() == typeid(double);
        };
        // output_map 是 std::map，所以 channels_ 天然按名字有序，下面查名字用二分。
        // （不写成 `| std::ranges::to<std::vector>()`：clang 20 配 libstdc++ 14 编不过管道形式。）
        for (const auto& name : output_map | std::views::filter(is_double) | std::views::keys)
            channels_.push_back(Channel{.name = name});
        if (channels_.size() > std::numeric_limits<SlotIndex>::max())
            throw std::length_error{"ValueBroadcaster: too many double outputs to index"};

        // InputInterface 不可移动，所以一次构造到位，之后不再增删。
        inputs_ = std::vector<InputInterface<double>>(channels_.size());
        for (std::size_t slot = 0; slot < channels_.size(); ++slot)
            register_input(channels_[slot].name, inputs_[slot]);

        apply_forward_list(get_parameter("forward_list").as_string_array());
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        if (const auto requested = requested_selection_.read(tick.scheduled); requested.fresh)
            selection_ = requested.value;

        // 队列满了就丢样本（EventQueue 自己计数，由发布线程报），绝不等。
        for (const SlotIndex slot : selection_.active())
            samples_.try_push(Sample{.slot = slot, .value = *inputs_[slot]});
    }

    [[nodiscard]] hcs_utility::Doorbell* tick_end_doorbell() override {
        return &publisher_thread_.doorbell();
    }

private:
    using SlotIndex = std::uint16_t;
    using Message = std_msgs::msg::Float64;

    /// 同时转发的上限。Selection 要过 Snapshot，所以必须是定长的平凡类型。
    static constexpr std::size_t kMaxForwarded = 64;
    /// 1 kHz × kMaxForwarded 路全开时，够发布线程落后 64 拍。
    static constexpr std::size_t kSampleQueueCapacity = 4096;

    /// 一个 double 输出。下标（SlotIndex）同时是 inputs_ 与 channels_ 的下标。
    struct Channel {
        std::string name;
        rclcpp::Publisher<Message>::SharedPtr publisher = nullptr; ///< 非空 = 正在转发
    };

    /// 控制线程每拍该采哪些。
    struct Selection {
        std::uint16_t count = 0;
        std::array<SlotIndex, kMaxForwarded> slots{};

        [[nodiscard]] std::span<const SlotIndex> active() const noexcept {
            return std::span{slots}.first(count);
        }
    };

    struct Sample {
        SlotIndex slot;
        double value;
    };

    hcs_utility::ThreadConfig declare_thread_config() {
        return hcs_utility::ThreadConfig{
            declare_parameter<std::string>("thread_config", ""), "hcs-values"};
    }

    /// 尽力域（主线程接线时一次，之后是 spin 线程上的参数回调）。
    void apply_forward_list(const std::vector<std::string>& forward_list) {
        const std::scoped_lock lock{channels_mutex_};

        auto selection = Selection{};
        auto wanted = std::vector<bool>(channels_.size(), false);
        for (const auto& name : forward_list) {
            const auto channel = std::ranges::lower_bound(channels_, name, {}, &Channel::name);
            if (channel == channels_.end() || channel->name != name) {
                logger().error(
                    "Unable to find corresponding output of '{}', maybe the type of output is "
                    "unsupported or the output does not exist.",
                    name);
                continue;
            }

            const auto slot = static_cast<SlotIndex>(std::distance(channels_.begin(), channel));
            if (wanted[slot])
                continue; // 列表里重复写了同一个名字
            if (selection.count == kMaxForwarded) {
                logger().error(
                    "forward_list is longer than {} entries, '{}' is not forwarded.", kMaxForwarded,
                    name);
                continue;
            }
            wanted[slot] = true;
            selection.slots[selection.count++] = slot;
        }

        for (std::size_t slot = 0; slot < channels_.size(); ++slot) {
            auto& channel = channels_[slot];
            if (!wanted[slot])
                channel.publisher.reset();
            else if (!channel.publisher)
                channel.publisher =
                    create_publisher<Message>(channel.name, rclcpp::QoS{5}.reliable());
        }

        // publisher 先就位，再让控制线程开始采：反过来的话头几个样本没有地方发。
        // 被摘掉的那几路，队列里残留的样本在 publish_pending() 里因为 publisher 为空被丢掉。
        requested_selection_.publish(selection, hcs_sync::Clock::now());
    }

    /// 尽力域（发布线程），每拍拍尾被叫醒一次。
    void publish_pending() {
        const std::scoped_lock lock{channels_mutex_};

        auto message = Message{};
        for (Sample sample; samples_.try_pop(sample);) {
            if (const auto& publisher = channels_[sample.slot].publisher) {
                message.data = sample.value;
                publisher->publish(message);
            }
        }

        if (const auto dropped = samples_.dropped(); dropped != reported_dropped_) {
            logger().warn(
                "Publishing fell behind the control loop, {} samples dropped.",
                dropped - reported_dropped_);
            reported_dropped_ = dropped;
        }
    }

    // ── 周期域：只有控制线程碰 ────────────────────────────────────────────
    std::vector<InputInterface<double>> inputs_; ///< before_pairing 定型，之后只读
    Selection selection_;

    // ── 跨域通道 ──────────────────────────────────────────────────────────
    hcs_sync::Snapshot<Selection> requested_selection_;         ///< 尽力域 → 周期域
    hcs_sync::EventQueue<Sample, kSampleQueueCapacity> samples_; ///< 周期域 → 发布线程

    // ── 尽力域：spin 线程与发布线程共用，channels_mutex_ 保护 ─────────────
    std::mutex channels_mutex_;
    std::vector<Channel> channels_; ///< 按 name 有序；元素个数 before_pairing 之后不变
    std::uint64_t reported_dropped_ = 0;

    std::unique_ptr<rclcpp::ParameterEventHandler> parameter_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_;

    /// 最后声明、最先析构：先停线程，它用到的东西才开始析构。
    hcs_utility::DoorbellWorker<std::move_only_function<void()>> publisher_thread_;
};

} // namespace hcs_core::broadcaster

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::broadcaster::ValueBroadcaster, hcs_executor::Component)
