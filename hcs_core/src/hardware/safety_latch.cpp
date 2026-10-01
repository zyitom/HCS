#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/timer.hpp>
#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_msgs/switch.hpp>

#include "hardware/util/safety_latch.hpp"

namespace hcs_core::hardware {

// 关键设备失效 → 整车锁存全失能（逻辑见 util::SafetyLatch）。跨板的事，所以是图里单独的
// 组件：读各关键设备的 <名字>/health 与复位拨杆，输出 /hardware/safe，各板组件的指令侧
// 读它决定这一拍发不发失能帧。
//
// 失效即安全：/hardware/safe 的初值（也是本组件被隔离时复位到的值）是 true；
// 板组件被隔离时它的 health 输出复位成"上过线、现在不在线"，本组件立刻锁存。
//
// 参数：
//   critical_devices  关键设备名列表（它们坏了车站不住），顺序即日志里报的那台
//   rearm_switch      复位手势的拨杆输入名：先拨到 DOWN、再拨离 DOWN 解除锁存
class SafetyLatch
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    SafetyLatch()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , names_(get_parameter("critical_devices").as_string_array())
        , health_inputs_(names_.size())
        , health_(names_.size()) {
        for (std::size_t i = 0; i < names_.size(); ++i)
            register_input(names_[i] + "/health", health_inputs_[i]);
        register_input(get_parameter("rearm_switch").as_string(), rearm_switch_);
        register_output("/hardware/safe", safe_, true);

        report_timer_ = create_wall_timer(std::chrono::seconds{1}, [this] { report(); });
    }

    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        for (std::size_t i = 0; i < health_.size(); ++i)
            health_[i] = *health_inputs_[i];
        *safe_ = latch_.update(health_, *rearm_switch_);
    }

private:
    // 锁存日志：周期域不许打日志，这里 1 Hz 读一次计数。读的是周期域在写的普通成员，
    // 是良性的撕读：最坏把同一次锁存晚一秒报、或把原因报成上一次的。
    void report() {
        using Reason = util::SafetyLatch::Reason;
        const auto trips = latch_.trip_count();
        if (trips == reported_trips_)
            return;
        reported_trips_ = trips;
        const auto index = latch_.tripped_device();
        RCLCPP_ERROR(
            get_logger(),
            "SAFETY LATCHED: %s %s; all motors disabled. "
            "Fix it, then flip the rearm switch DOWN and back to re-arm",
            index < names_.size() ? names_[index].c_str() : "?",
            latch_.reason() == Reason::kFaulted ? "reported a fault" : "went offline");
    }

    std::vector<std::string> names_;
    std::vector<InputInterface<util::DeviceHealth>> health_inputs_;
    std::vector<util::DeviceHealth> health_; ///< 周期域每拍重写，构造期定长
    InputInterface<hcs_msgs::Switch> rearm_switch_;
    OutputInterface<bool> safe_;

    util::SafetyLatch latch_;
    std::uint32_t reported_trips_ = 0; ///< 尽力域
    rclcpp::TimerBase::SharedPtr report_timer_;
};

} // namespace hcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::hardware::SafetyLatch, hcs_executor::Component)
