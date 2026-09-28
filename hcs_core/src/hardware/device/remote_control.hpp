#pragma once

#include <hcs_executor/component.hpp>
#include <hcs_msgs/keyboard.hpp>
#include <hcs_msgs/mouse.hpp>
#include <hcs_msgs/switch.hpp>

#include "hardware/device/vt13.hpp"

namespace hcs_core::hardware::device {

/*
遥控仲裁（移植自 RMCS 的 device/remote_control.hpp）。

HCS 的驱动清单里只有 VT13（挂 aux 板 UART0），RMCS 版里的 DR16 分支
与两路接收机之间的超时互锁没有对应物，不移植。仲裁规则相应简化为：

- vt13 valid 且 S 挡：vt13 主控（比赛用），拨杆强制 MIDDLE/MIDDLE；
- vt13 valid 且 C 挡：安全态，拨杆强制 DOWN/DOWN、其余全空（救车用）；
- 其余（invalid / N 挡 / 挡位未知）：空安全态，全部归零。

输出接口与 RMCS 保持同名：/remote/joystick、/remote/switch、/remote/mouse、
/remote/keyboard 各族。旋钮接口属于 DR16，不注册。
*/

class RemoteControl {
public:
    explicit RemoteControl(hcs_executor::Component& component) {
        component.register_output(
            "/remote/joystick/right", joystick_right_output_, Eigen::Vector2d::Zero());
        component.register_output(
            "/remote/joystick/left", joystick_left_output_, Eigen::Vector2d::Zero());

        component.register_output(
            "/remote/switch/right", switch_right_output_, hcs_msgs::Switch::UNKNOWN);
        component.register_output(
            "/remote/switch/left", switch_left_output_, hcs_msgs::Switch::UNKNOWN);

        component.register_output(
            "/remote/mouse/velocity", mouse_velocity_output_, Eigen::Vector2d::Zero());
        component.register_output("/remote/mouse/mouse_wheel", mouse_wheel_output_, 0.0);

        component.register_output("/remote/mouse", mouse_output_, hcs_msgs::Mouse::zero());
        component.register_output(
            "/remote/keyboard", keyboard_output_, hcs_msgs::Keyboard::zero());
    }

    void register_vt13(Vt13* vt13) { vt13_ = vt13; }

    void update() {
        if (vt13_ && vt13_->valid()) {
            switch (vt13_->mode_switch()) {
            case Vt13::ModeSwitch::kSport:
                snapshot_remote_control(Vt13Source{});
                *switch_right_output_ = hcs_msgs::Switch::MIDDLE;
                *switch_left_output_  = hcs_msgs::Switch::MIDDLE;
                return;
            case Vt13::ModeSwitch::kCine:
                clear_remote_control();
                *switch_right_output_ = hcs_msgs::Switch::DOWN;
                *switch_left_output_  = hcs_msgs::Switch::DOWN;
                return;
            case Vt13::ModeSwitch::kNormal:
            case Vt13::ModeSwitch::kUnknown:
                break;
            }
        }

        clear_remote_control();
    }

private:
    struct Vt13Source {};

    void snapshot_remote_control(Vt13Source) {
        *joystick_right_output_ = vt13_->joystick_right();
        *joystick_left_output_  = vt13_->joystick_left();
        *mouse_velocity_output_ = vt13_->mouse_velocity();
        *mouse_wheel_output_    = vt13_->mouse_wheel();
        *mouse_output_          = vt13_->mouse();
        *keyboard_output_       = vt13_->keyboard();
    }

    void clear_remote_control() {
        *joystick_right_output_ = Eigen::Vector2d::Zero();
        *joystick_left_output_  = Eigen::Vector2d::Zero();
        *mouse_velocity_output_ = Eigen::Vector2d::Zero();
        *mouse_wheel_output_    = 0.0;
        *mouse_output_          = hcs_msgs::Mouse::zero();
        *keyboard_output_       = hcs_msgs::Keyboard::zero();
        *switch_right_output_   = hcs_msgs::Switch::UNKNOWN;
        *switch_left_output_    = hcs_msgs::Switch::UNKNOWN;
    }

    Vt13* vt13_{nullptr};

    hcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_right_output_;
    hcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_left_output_;

    hcs_executor::Component::OutputInterface<hcs_msgs::Switch> switch_right_output_;
    hcs_executor::Component::OutputInterface<hcs_msgs::Switch> switch_left_output_;

    hcs_executor::Component::OutputInterface<Eigen::Vector2d> mouse_velocity_output_;
    hcs_executor::Component::OutputInterface<double> mouse_wheel_output_;

    hcs_executor::Component::OutputInterface<hcs_msgs::Mouse> mouse_output_;
    hcs_executor::Component::OutputInterface<hcs_msgs::Keyboard> keyboard_output_;
};

} // namespace hcs_core::hardware::device
