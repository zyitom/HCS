#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_sync/tick.hpp>
#include <hcs_utility/rt_attributes.hpp>

#include "controller/chassis/balance/leg_kinematics.hpp"

namespace hcs_core::controller::chassis::balance {

// ============================================================================
// 腿关节控制器：VMC 力映射（对应 Helios Motor_Send 的关节部分 +
// leg_VMC 调用）。在 yaml 里实例化两次（left / right）。
//
// 输入本腿的支撑力（LegForceController 的 control_force，已含弹簧补偿）、
// 髋力矩（LQR 控制器 + 起跳补偿之和）与四个关节角（估计器），调 leg_vmc
// 得到两个关节力矩。Helios 在 Motor_Send 里对右腿取负（-final_Tl0/Tl1），
// 这里由 side_sign 参数表达；输出限幅 ±40 N·m（T_MAX_LEG 的发送端限幅）。
// ============================================================================

class LegJointController
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    LegJointController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        const std::string leg_prefix = get_parameter("leg_prefix").as_string();
        register_input(leg_prefix + "/control_force", control_force_);
        register_input(leg_prefix + "/control_hip_torque", control_hip_torque_);
        register_input(leg_prefix + "/hip_torque_compensation", hip_torque_compensation_);
        register_input(leg_prefix + "/length", length_);
        register_input(leg_prefix + "/angle", leg_angle_);
        register_input(leg_prefix + "/hip_front_angle", hip_front_angle_);
        register_input(leg_prefix + "/hip_back_angle", hip_back_angle_);
        register_input(leg_prefix + "/knee_front_angle", knee_front_angle_);
        register_input(leg_prefix + "/knee_back_angle", knee_back_angle_);

        const std::string front_prefix = get_parameter("front_joint_prefix").as_string();
        const std::string back_prefix = get_parameter("back_joint_prefix").as_string();
        register_output(front_prefix + "/control_torque", front_torque_, nan_);
        register_output(back_prefix + "/control_torque", back_torque_, nan_);

        l1_ = get_parameter("leg_link1").as_double();
        side_sign_ = get_parameter("side_sign").as_double();
        max_torque_ = get_parameter("max_torque").as_double();
    }

    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        double front_torque = 0.0;
        double back_torque = 0.0;
        leg_vmc(
            *leg_angle_, *hip_front_angle_, *knee_front_angle_, *knee_back_angle_,
            *hip_back_angle_, l1_, *length_, *control_force_,
            *control_hip_torque_ + *hip_torque_compensation_, front_torque, back_torque);

        *front_torque_ = side_sign_ * std::clamp(front_torque, -max_torque_, max_torque_);
        *back_torque_ = side_sign_ * std::clamp(back_torque, -max_torque_, max_torque_);
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<double> control_force_;
    InputInterface<double> control_hip_torque_;
    InputInterface<double> hip_torque_compensation_;
    InputInterface<double> length_;
    InputInterface<double> leg_angle_;
    InputInterface<double> hip_front_angle_;
    InputInterface<double> hip_back_angle_;
    InputInterface<double> knee_front_angle_;
    InputInterface<double> knee_back_angle_;

    OutputInterface<double> front_torque_;
    OutputInterface<double> back_torque_;

    double l1_ = 0.21;
    double side_sign_ = 1.0;  ///< 左 +1 / 右 −1（Helios Motor_Send 的取负）
    double max_torque_ = 40.0;
};

} // namespace hcs_core::controller::chassis::balance

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    hcs_core::controller::chassis::balance::LegJointController, hcs_executor::Component)
