#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <eigen3/Eigen/Dense>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_msgs/keyboard.hpp>
#include <hcs_msgs/switch.hpp>
#include <hcs_sync/tick.hpp>
#include <hcs_utility/rt_attributes.hpp>

#include "controller/chassis/balance/balance_types.hpp"
#include "controller/chassis/balance/control_math.hpp"

namespace hcs_core::controller::chassis::balance {

// ============================================================================
// 平衡底盘操作意图（对应 Helios fsm.cpp 里底盘相关的键位 + 遥控拨杆）。
//
// /remote/* → 模式请求 + 速度指令 + 边沿计数器。键位映射（对照 Helios
// Keyboard_Controller 的活分支，R 组合键一律不搬——里面有整机复位，太危险）：
//
//   W/S            前后速度（乘 max_velocity，TD 平滑后输出）
//   C（边沿）      小陀螺开/关（spin）
//   X（边沿）      掉头（turn_count + 1，跟随目标加 π）
//   Q（边沿）      跳跃请求（jump_count + 1）
//   A（边沿）      自救请求（recover_count + 1）
//   E（边沿）      腿长三档循环（LOW → MID → HIGH）
//   Ctrl（按住）   飞坡请求（离地检测默认禁用时不可达，同 Helios）
//
// 拨杆：switch_left DOWN/UNKNOWN = 失能；MIDDLE/UP = 使能。
// 边沿请求用计数器表达（照 RMCS /chassis/deformable/reset_count 的做法），
// 不传单拍脉冲。速度指令做 TD 平滑（Helios td_input，r=25、h0=10h）。
// ============================================================================

class BalanceChassisController
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    BalanceChassisController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        register_input("/remote/joystick/right", joystick_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/switch/left", switch_left_);

        register_output("/chassis/control_mode", control_mode_,
                        static_cast<std::uint8_t>(Mode::kDisabled));
        register_output("/chassis/control_velocity", control_velocity_);
        register_output("/chassis/balance/leg_length_level", leg_length_level_,
                        static_cast<std::uint8_t>(LegLengthLevel::kLow));
        register_output("/chassis/balance/jump_count", jump_count_, std::size_t{0});
        register_output("/chassis/balance/recover_count", recover_count_, std::size_t{0});
        register_output("/chassis/balance/turn_count", turn_count_, std::size_t{0});
        register_output("/chassis/balance/fly_request", fly_request_, false);

        max_velocity_ = get_parameter("max_velocity").as_double();
        spin_rate_ = get_parameter("spin_rate").as_double();
        velocity_td_.parameters.r = get_parameter("velocity_td_r").as_double();
        velocity_td_.parameters.h0 = get_parameter("velocity_td_h0").as_double();
        spin_td_.parameters.r = get_parameter("spin_td_r").as_double();
        spin_td_.parameters.h0 = get_parameter("spin_td_h0").as_double();

    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        const auto& keyboard = *keyboard_;

        update_key_edges(keyboard);

        // 拨杆使能：DOWN/UNKNOWN 失能。
        const bool enabled =
            *switch_left_ == hcs_msgs::Switch::MIDDLE || *switch_left_ == hcs_msgs::Switch::UP;
        if (!enabled) {
            *control_mode_ = static_cast<std::uint8_t>(Mode::kDisabled);
            velocity_td_.reset();
            spin_td_.reset();
            *control_velocity_ = {0.0, 0.0, 0.0};
            return;
        }

        if (spin_toggle_edge_) {
            spinning_ = !spinning_;
            if (!spinning_)
                spin_td_.reset();
        }

        *control_mode_ = static_cast<std::uint8_t>(spinning_ ? Mode::kSpin : Mode::kNormal);

        // 速度指令：摇杆前后（y）+ 键盘 W/S，TD 平滑；小陀螺转速单独一路 TD。
        const double raw_velocity =
            joystick_->y() + static_cast<double>(keyboard.w) - static_cast<double>(keyboard.s);
        const double velocity =
            std::clamp(raw_velocity, -1.0, 1.0) * max_velocity_;
        const double smoothed_velocity = velocity_td_.update(velocity, tick.dt_seconds());
        const double smoothed_spin =
            spinning_ ? spin_td_.update(spin_rate_, tick.dt_seconds()) : 0.0;

        *control_velocity_ = {smoothed_velocity, 0.0, smoothed_spin};
    }

private:
    void update_key_edges(const hcs_msgs::Keyboard& keyboard) {
        spin_toggle_edge_ = edge(keyboard.c, last_c_);
        if (edge(keyboard.x, last_x_))
            *turn_count_ += 1;
        if (edge(keyboard.q, last_q_))
            *jump_count_ += 1;
        if (edge(keyboard.a, last_a_))
            *recover_count_ += 1;
        if (edge(keyboard.e, last_e_)) {
            const auto level = static_cast<LegLengthLevel>((*leg_length_level_ + 1) % 3);
            *leg_length_level_ = static_cast<std::uint8_t>(level);
        }
        *fly_request_ = keyboard.ctrl;
    }

    static bool edge(bool current, bool& last) {
        const bool pressed = current && !last;
        last = current;
        return pressed;
    }

    InputInterface<Eigen::Vector2d> joystick_;
    InputInterface<hcs_msgs::Keyboard> keyboard_;
    InputInterface<hcs_msgs::Switch> switch_left_;

    OutputInterface<std::uint8_t> control_mode_;
    OutputInterface<Eigen::Vector3d> control_velocity_;
    OutputInterface<std::uint8_t> leg_length_level_;
    OutputInterface<std::size_t> jump_count_;
    OutputInterface<std::size_t> recover_count_;
    OutputInterface<std::size_t> turn_count_;
    OutputInterface<bool> fly_request_;

    double max_velocity_ = 2.0;
    double spin_rate_ = 8.0;
    TrackingDifferentiator velocity_td_;
    TrackingDifferentiator spin_td_;

    bool spinning_ = false;
    bool spin_toggle_edge_ = false;
    bool last_c_ = false;
    bool last_x_ = false;
    bool last_q_ = false;
    bool last_a_ = false;
    bool last_e_ = false;
};

} // namespace hcs_core::controller::chassis::balance

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    hcs_core::controller::chassis::balance::BalanceChassisController, hcs_executor::Component)
