#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_base/thread/thread_config.hpp>
#include <hcs_executor/component.hpp>

#include "hardware/util/board_transmitter.hpp"

namespace hcs_core::hardware {

// 几块板共用的发送线程，作为图里的一个组件出现：输出 /hardware/transmitter，
// 各板组件经输入拿到它、打开板卡后各登记一条 Lane。拍尾门铃由它提供，executor 每拍叫醒
// 它一次，它依次替各板发（见 util::SharedTransmitter）。
//
// 参数：
//   thread_config   发送线程的核与优先级（缺省线程名 "tx-<组件名>"）
//   stale_after_ms  批次陈旧阈值：某块板被隔离后，超过它就给那块板补发一次全失能（默认 10）
class TransmitThread
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    TransmitThread()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        auto thread_name = "tx-" + get_component_name();
        thread_name.resize(std::min<std::size_t>(thread_name.size(), 15)); // Linux 线程名上限
        transmitter_ = std::make_shared<util::SharedTransmitter>(
            hcs_utility::ThreadConfig{
                get_parameter_or<std::string>("thread_config", ""), thread_name},
            std::chrono::milliseconds{get_parameter_or<std::int64_t>("stale_after_ms", 10)});
        register_output("/hardware/transmitter", output_, transmitter_);
    }

    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {}

    hcs_utility::Doorbell* tick_end_doorbell() override { return &transmitter_->doorbell(); }

private:
    std::shared_ptr<util::SharedTransmitter> transmitter_;
    OutputInterface<std::shared_ptr<util::SharedTransmitter>> output_;
};

} // namespace hcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::hardware::TransmitThread, hcs_executor::Component)
