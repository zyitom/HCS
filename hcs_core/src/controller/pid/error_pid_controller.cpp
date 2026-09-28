#include <limits>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_sync/tick.hpp>
#include <hcs_utility/rt_attributes.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "controller/pid/smart_input.hpp"

namespace hcs_core::controller::pid {

// 移植自 RMCS 的 error_pid_controller.cpp：输入 measurement 的语义是
// 「误差已经算好」（例如云台的 /gimbal/yaw/control_angle_error），
// 输出 control = feedforward + PID(measurement)。
// 三个接口名全部来自参数，可在 yaml 里实例化任意多份。
class ErrorPidController
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    ErrorPidController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , measurement_(*this, "measurement")
        , feedforward_(*this, "feedforward", 0.0)
        , pid_calculator_(
              get_parameter("kp").as_double(), get_parameter("ki").as_double(),
              get_parameter("kd").as_double()) {

        register_output(get_parameter("control").as_string(), control_, nan_);

        get_parameter("integral_min", pid_calculator_.integral_min);
        get_parameter("integral_max", pid_calculator_.integral_max);

        get_parameter("integral_split_min", pid_calculator_.integral_split_min);
        get_parameter("integral_split_max", pid_calculator_.integral_split_max);

        get_parameter("output_min", pid_calculator_.output_min);
        get_parameter("output_max", pid_calculator_.output_max);
    }

    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        auto err = *measurement_;
        *control_ = *feedforward_ + pid_calculator_.update(err);
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    SmartInput measurement_, feedforward_;

    PidCalculator pid_calculator_;

    OutputInterface<double> control_;
};

} // namespace hcs_core::controller::pid

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::controller::pid::ErrorPidController, hcs_executor::Component)
