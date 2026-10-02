#include <chrono>

#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>

#include <fast_tf/fast_tf.hpp>
#include <fast_tf/rcl.hpp>
#include <hcs_base/channel/snapshot.hpp>
#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_description/tf_description.hpp>
#include <hcs_executor/component.hpp>

namespace hcs_core::broadcaster {

// 把 /tf（fast_tf 的关节集合）以 20 Hz 发成 ROS 的 TF，给 rviz 之类的工具看。
//
// 位姿怎么抓、消息怎么拼都是 fast_tf 的事（fast_tf::capture / fast_tf::rcl::Broadcaster），
// 这里只负责把两半放到各自的域里，中间走一条 Snapshot：
//
//   周期域（update()）    每 50 ms capture 一次，publish 进 Snapshot。只有定长的算术，不碰 ROS。
//   尽力域（spin 线程）   定时器看 Snapshot 有没有新样本，有就一次发出整棵树。
//
// 这里用定时器轮询而不是拍尾门铃：TF 只有 20 Hz，为它每拍叫醒一条线程不值；
// 而且 TF 消息自带时间戳，发得晚几毫秒不影响接收方——时间戳填的是**采样**那一刻
// （由 Snapshot 的 age 倒推），不是发出去的那一刻。
class TfBroadcaster
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    TfBroadcaster()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , broadcaster_{this} {
        register_input("/tf", tf_);
        poll_timer_ = create_wall_timer(kPollPeriod, [this] { broadcast_if_fresh(); });
    }

    /// 起拍之前先放一份初始位姿进去，这样 spin 一开始转就有 TF 可发。
    /// 跑在主线程上，此时控制线程还没创建，所以它在这里当一次 Snapshot 的写者是安全的。
    void before_updating() override {
        poses_.publish(fast_tf::capture(*tf_), hcs_sync::Clock::now());
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        if (tick.scheduled < next_capture_at_)
            return;
        next_capture_at_ = tick.scheduled + kBroadcastPeriod;
        poses_.publish(fast_tf::capture(*tf_), tick.scheduled);
    }

private:
    using Tf = hcs_description::Tf;

    static constexpr std::chrono::milliseconds kBroadcastPeriod{50};
    /// 从采到样到发出去最多晚这么久。
    static constexpr std::chrono::milliseconds kPollPeriod{10};

    /// 尽力域（spin 线程）。
    void broadcast_if_fresh() {
        const auto reading = poses_.read(hcs_sync::Clock::now());
        if (reading.fresh)
            broadcaster_.broadcast(reading.value, now() - rclcpp::Duration{reading.age});
    }

    // ── 周期域 ────────────────────────────────────────────────────────────
    InputInterface<Tf> tf_;
    hcs_sync::Timestamp next_capture_at_{}; ///< 纪元 = 首拍立即采

    // ── 跨域通道：周期域 → 尽力域 ─────────────────────────────────────────
    hcs_sync::Snapshot<fast_tf::JointPoses<Tf>> poses_;

    // ── 尽力域：只有 spin 线程碰 ──────────────────────────────────────────
    fast_tf::rcl::Broadcaster<Tf> broadcaster_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
};

} // namespace hcs_core::broadcaster

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::broadcaster::TfBroadcaster, hcs_executor::Component)
