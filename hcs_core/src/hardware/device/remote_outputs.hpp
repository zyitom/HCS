#pragma once

#include <eigen3/Eigen/Dense>

#include <hcs_executor/component.hpp>
#include <hcs_msgs/keyboard.hpp>
#include <hcs_msgs/mouse.hpp>
#include <hcs_msgs/switch.hpp>

namespace hcs_core::hardware::device {

/// 遥控器对外的那一组输出 /remote/*。
///
/// 一台车上只有一个接收机驱动持有它（Vt13Remote 或 Dr16Remote）：每拍由那个驱动决定这一拍
/// 遥控的状态，交给 publish()。两种接收机都接上的车会在接线时因为输出重名被拒绝，而不是
/// 悄悄地让两个驱动抢着写——真要两路并用，仲裁规则得先定下来，那时再写。
class RemoteOutputs {
public:
    /// 这一拍遥控器的状态。默认值就是"空安全态"：摇杆、鼠标、键盘全零，拨杆 UNKNOWN。
    /// 没有可用的遥控（失联、挡位不允许）时发布的就是它，而不是失联前的最后一帧。
    struct State {
        Eigen::Vector2d joystick_right = Eigen::Vector2d::Zero();
        Eigen::Vector2d joystick_left = Eigen::Vector2d::Zero();
        hcs_msgs::Switch switch_right = hcs_msgs::Switch::UNKNOWN;
        hcs_msgs::Switch switch_left = hcs_msgs::Switch::UNKNOWN;
        Eigen::Vector2d mouse_velocity = Eigen::Vector2d::Zero();
        double mouse_wheel = 0.0;
        hcs_msgs::Mouse mouse = hcs_msgs::Mouse::zero();
        hcs_msgs::Keyboard keyboard = hcs_msgs::Keyboard::zero();
    };

    explicit RemoteOutputs(hcs_executor::Component& component) {
        const State safe;
        component.register_output("/remote/joystick/right", joystick_right_, safe.joystick_right);
        component.register_output("/remote/joystick/left", joystick_left_, safe.joystick_left);
        component.register_output("/remote/switch/right", switch_right_, safe.switch_right);
        component.register_output("/remote/switch/left", switch_left_, safe.switch_left);
        component.register_output("/remote/mouse/velocity", mouse_velocity_, safe.mouse_velocity);
        component.register_output("/remote/mouse/mouse_wheel", mouse_wheel_, safe.mouse_wheel);
        component.register_output("/remote/mouse", mouse_, safe.mouse);
        component.register_output("/remote/keyboard", keyboard_, safe.keyboard);
    }

    /// 周期域，每拍一次。
    void publish(const State& state) noexcept {
        *joystick_right_ = state.joystick_right;
        *joystick_left_ = state.joystick_left;
        *switch_right_ = state.switch_right;
        *switch_left_ = state.switch_left;
        *mouse_velocity_ = state.mouse_velocity;
        *mouse_wheel_ = state.mouse_wheel;
        *mouse_ = state.mouse;
        *keyboard_ = state.keyboard;
    }

    /// 这一拍 /remote/switch/left 上的值。
    [[nodiscard]] hcs_msgs::Switch switch_left() const noexcept { return *switch_left_; }

private:
    hcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_right_;
    hcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_left_;
    hcs_executor::Component::OutputInterface<hcs_msgs::Switch> switch_right_;
    hcs_executor::Component::OutputInterface<hcs_msgs::Switch> switch_left_;
    hcs_executor::Component::OutputInterface<Eigen::Vector2d> mouse_velocity_;
    hcs_executor::Component::OutputInterface<double> mouse_wheel_;
    hcs_executor::Component::OutputInterface<hcs_msgs::Mouse> mouse_;
    hcs_executor::Component::OutputInterface<hcs_msgs::Keyboard> keyboard_;
};

} // namespace hcs_core::hardware::device
