#include <cmath>
#include <limits>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_msgs/mouse.hpp>
#include <hcs_msgs/switch.hpp>
#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>

#include "controller/gimbal/two_axis_gimbal_solver.hpp"

namespace hcs_core::controller::gimbal {

// 移植自 RMCS 的 simple_gimbal_controller.cpp：/remote/* →
// TwoAxisGimbalSolver → /gimbal/{yaw,pitch}/control_angle_error。
// 自瞄输入（/auto_aim/*）保持可选且不接线：视觉桥这次不做（§4.5）。
// 安全态（双 UNKNOWN 或双 DOWN）输出 NaN。
class SimpleGimbalController
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    SimpleGimbalController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , two_axis_gimbal_solver{
              *this, get_parameter("upper_limit").as_double(),
              get_parameter("lower_limit").as_double()} {

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/auto_aim/should_control", should_control_, false);
        register_input("/auto_aim/control_direction", auto_aim_control_direction_, false);


        register_output("/gimbal/yaw/control_angle_error", yaw_control_angle_error_, nan_);
        register_output("/gimbal/pitch/control_angle_error", pitch_control_angle_error_, nan_);
    }

    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        calculate_angle_error();
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    void calculate_angle_error() {
        using SetDisabled = TwoAxisGimbalSolver::SetDisabled;
        using SetToLevel = TwoAxisGimbalSolver::SetToLevel;
        using SetControlDirection = TwoAxisGimbalSolver::SetControlDirection;
        using SetControlShift = TwoAxisGimbalSolver::SetControlShift;

        if ((*switch_right_ == hcs_msgs::Switch::UNKNOWN
                && *switch_left_ == hcs_msgs::Switch::UNKNOWN)
            || (*switch_right_ == hcs_msgs::Switch::DOWN
                && *switch_left_ == hcs_msgs::Switch::DOWN)) {
            const auto [yaw_angle_error, pitch_angle_error] = two_axis_gimbal_solver.update(SetDisabled{});
            *yaw_control_angle_error_ = yaw_angle_error;
            *pitch_control_angle_error_ = pitch_angle_error;
            return;
        }

        const bool mouse_control = mouse_->right;
        if (mouse_control && should_control_.has_provider() && *should_control_
            && (*auto_aim_control_direction_).vector.allFinite()) {
            const auto [yaw_angle_error, pitch_angle_error] = two_axis_gimbal_solver.update(
                SetControlDirection{OdomImu::DirectionVector{*auto_aim_control_direction_}});
            *yaw_control_angle_error_ = yaw_angle_error;
            *pitch_control_angle_error_ = pitch_angle_error;
            return;
        }

        if (!two_axis_gimbal_solver.enabled()) {
            const auto [yaw_angle_error, pitch_angle_error] = two_axis_gimbal_solver.update(SetToLevel{});
            *yaw_control_angle_error_ = yaw_angle_error;
            *pitch_control_angle_error_ = pitch_angle_error;
            return;
        }

        constexpr double joystick_sensitivity = 0.006;
        constexpr double mouse_sensitivity = 0.5;
        double yaw_shift = joystick_sensitivity * joystick_left_->y()
            + mouse_sensitivity * mouse_velocity_->y();
        double pitch_shift = -joystick_sensitivity * joystick_left_->x()
            + mouse_sensitivity * mouse_velocity_->x();
        const auto [yaw_angle_error, pitch_angle_error] =
            two_axis_gimbal_solver.update(SetControlShift{yaw_shift, pitch_shift});
        *yaw_control_angle_error_ = yaw_angle_error;
        *pitch_control_angle_error_ = pitch_angle_error;
    }

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<hcs_msgs::Switch> switch_right_;
    InputInterface<hcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<hcs_msgs::Mouse> mouse_;
    InputInterface<bool> should_control_;
    InputInterface<gimbal::OdomImu::DirectionVector> auto_aim_control_direction_;

    TwoAxisGimbalSolver two_axis_gimbal_solver;

    OutputInterface<double> yaw_control_angle_error_;
    OutputInterface<double> pitch_control_angle_error_;
};

} // namespace hcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    hcs_core::controller::gimbal::SimpleGimbalController, hcs_executor::Component)
