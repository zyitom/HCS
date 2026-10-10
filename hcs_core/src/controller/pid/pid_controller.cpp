#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace hcs_core::controller::pid {

// 三个接口名全部来自参数，所以同一个类可以在 yaml 里实例化任意多份；
// 串级 PID 只是把上一级的 control 名字填进下一级的 setpoint，
// 拓扑排序会自动把顺序排对。
//
// 真实工程里这三个 input 用的是 SmartInput，额外支持「直接填常数」和
// 「名字前加 - 表示取反」；这里为了最小化用了裸的 register_input。
class PidController
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    PidController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , pid_calculator_(
              get_parameter("kp").as_double(), get_parameter("ki").as_double(),
              get_parameter("kd").as_double()) {

        register_input(get_parameter("measurement").as_string(), measurement_);
        register_input(get_parameter("setpoint").as_string(), setpoint_);
        register_output(get_parameter("control").as_string(), control_, 0.0);

        get_parameter("integral_min", pid_calculator_.integral_min);
        get_parameter("integral_max", pid_calculator_.integral_max);

        get_parameter("integral_split_min", pid_calculator_.integral_split_min);
        get_parameter("integral_split_max", pid_calculator_.integral_split_max);

        get_parameter("output_min", pid_calculator_.output_min);
        get_parameter("output_max", pid_calculator_.output_max);
    }

    // 这里刻意不把 Tick 的 dt 喂进 PidCalculator：它的积分是纯累加、微分是纯差分，
    // dt 早就折进了 ki / kd 里。改成按时间积分会让所有已经调好的增益一夜之间失效。
    //
    // 代价是已知且被接受的：跳拍时积分少一个样本、微分少一次差分 ——
    // 幅度就是一拍的量，比「悄悄改掉用户的增益」小得多。
    // 真要按时间积分是 C 线的事，届时要连带给出增益迁移规则。
    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        *control_ = pid_calculator_.update(*setpoint_ - *measurement_);
    }

private:
    InputInterface<double> measurement_, setpoint_;

    PidCalculator pid_calculator_;

    OutputInterface<double> control_;
};

} // namespace hcs_core::controller::pid

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::controller::pid::PidController, hcs_executor::Component)
