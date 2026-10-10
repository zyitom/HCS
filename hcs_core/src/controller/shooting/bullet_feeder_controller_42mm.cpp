#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_msgs/keyboard.hpp>
#include <hcs_msgs/mouse.hpp>
#include <hcs_msgs/shoot_mode.hpp>
#include <hcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace hcs_core::controller::shooting {

// 42 mm 拨弹盘（英雄，六格拨盘）：角度环套速度环，一发一发地拨。
// 移植自 RMCS 的 bullet_feeder_controller_42mm.cpp。
//
// 一发的过程：
//
//   PRELOADED ──开火──▶ SHOOTING ──到位──▶ PRELOADING ──到位──▶ PRELOADED
//       │                                                          ▲
//       └─低延迟模式─▶ COMPRESSING ──到位──▶ COMPRESSED ──开火──▶ SHOOTING
//
//   PRELOADING / PRELOADED   弹丸推到半格的位置等着
//   COMPRESSING / COMPRESSED 低延迟模式：提前把弹丸压到整格的位置，开火时少走半格
//   SHOOTING                 推过整格再多 0.2 格，把弹丸送进摩擦轮
//
// 操作：
//   鼠标左键按下，或左拨杆 中 → 下     打一发（要摩擦轮就绪、热量够）
//   Ctrl + R                           开 / 关低延迟模式
//
// 卡弹：输出力矩连续 1 s 顶在 300 以上判卡弹，反转 500 ms、再松开 500 ms，然后重新对位。
// 安全态（任一拨杆 UNKNOWN，或双下）：给定是 NaN。
//
// 所有计时都按 Tick::dt 累计，不数拍。没有参数：拨盘的几何尺寸和 PID 都写在这里。
class BulletFeederController42mm
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    BulletFeederController42mm()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/friction_ready", friction_ready_);

        register_input("/gimbal/bullet_feeder/angle", bullet_feeder_angle_);
        register_input("/gimbal/bullet_feeder/velocity", bullet_feeder_velocity_);

        register_input(
            "/gimbal/control_bullet_allowance/limited_by_heat",
            control_bullet_allowance_limited_by_heat_);
        register_input("/gimbal/bullet_fired", bullet_fired_);

        bullet_feeder_velocity_pid_.kp = 50.0;
        bullet_feeder_velocity_pid_.ki = 10.0;
        bullet_feeder_velocity_pid_.kd = 0.0;
        bullet_feeder_velocity_pid_.integral_max = 60.0;
        bullet_feeder_velocity_pid_.integral_min = 0.0;

        bullet_feeder_angle_pid_.kp = 60.0;
        bullet_feeder_angle_pid_.ki = 0.0;
        bullet_feeder_angle_pid_.kd = 2.0;

        register_output(
            "/gimbal/bullet_feeder/control_torque", bullet_feeder_control_torque_, kNan);

        // 只为和 17 mm 的接口保持一致：英雄恒为单发。
        register_output("/gimbal/shooter/mode", shoot_mode_, hcs_msgs::ShootMode::SINGLE);
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto mouse = *mouse_;
        const auto keyboard = *keyboard_;

        using hcs_msgs::Switch;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            return;
        }
        overdrive_mode_ = keyboard.f;
        if (keyboard.ctrl && !last_keyboard_.r && keyboard.r)
            low_latency_mode_ = !low_latency_mode_;

        const auto zero = hcs_sync::Duration::zero();
        if (bullet_feeder_cool_down_ > zero) {
            bullet_feeder_cool_down_ = std::max(zero, bullet_feeder_cool_down_ - tick.dt);

            // 前半段反转退弹，后半段松开。
            if (bullet_feeder_cool_down_ > kJamReleaseTime)
                *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(
                    -5.0 * kAnglePerBullet - *bullet_feeder_velocity_);
            else {
                bullet_feeder_velocity_pid_.reset();
                *bullet_feeder_control_torque_ = 0.0;
            }

            bullet_feeder_angle_pid_.reset();

            if (bullet_feeder_cool_down_ == zero)
                logger().rt().info("Jamming Solved!");
        } else {
            // 摩擦轮没就绪、或者刚退完弹：以拨盘现在的位置为准重新对位，
            // 算出现在是第几格。
            if (!*friction_ready_ || std::isnan(bullet_feeder_control_angle_)) {
                bullet_feeder_control_angle_ = *bullet_feeder_angle_;
                shoot_stage_ = ShootStage::kPreloaded;
                bullet_fed_count_ = static_cast<int>(
                    (*bullet_feeder_angle_ - kCompressedZeroPoint - 0.1) / kAnglePerBullet);
            }

            if (*friction_ready_) {
                if (switch_right != Switch::DOWN) {
                    if ((!last_mouse_.left && mouse.left)
                        || (last_switch_left_ == Switch::MIDDLE && switch_left == Switch::DOWN)) {
                        if (*control_bullet_allowance_limited_by_heat_ > 0)
                            set_shooting();
                    }
                }

                const double angle_error_abs =
                    std::abs(bullet_feeder_control_angle_ - *bullet_feeder_angle_);
                if (shoot_stage_ == ShootStage::kPreloading) {
                    if (angle_error_abs < 0.1)
                        set_preloaded();
                }
                if (shoot_stage_ == ShootStage::kPreloaded) {
                    if (low_latency_mode_)
                        set_compressing();
                }
                if (shoot_stage_ == ShootStage::kCompressing) {
                    if (angle_error_abs < 0.1)
                        set_compressed();
                }
                if (shoot_stage_ == ShootStage::kShooting) {
                    if (angle_error_abs < 0.1)
                        set_preloading();
                }
            }

            const double velocity_error = bullet_feeder_angle_pid_.update(
                                              bullet_feeder_control_angle_ - *bullet_feeder_angle_)
                                        - *bullet_feeder_velocity_;
            *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(velocity_error);

            update_jam_detection(tick);
        }

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_mouse_ = mouse;
        last_keyboard_ = keyboard;
    }

private:
    void reset_all_controls() {
        last_switch_right_ = hcs_msgs::Switch::UNKNOWN;
        last_switch_left_ = hcs_msgs::Switch::UNKNOWN;
        last_mouse_ = hcs_msgs::Mouse::zero();
        last_keyboard_ = hcs_msgs::Keyboard::zero();

        overdrive_mode_ = low_latency_mode_ = false;

        shoot_stage_ = ShootStage::kPreloaded;
        bullet_fed_count_ = std::numeric_limits<int>::min();

        bullet_feeder_control_angle_ = kNan;
        bullet_feeder_angle_pid_.output_max = kInf;

        bullet_feeder_velocity_pid_.reset();
        bullet_feeder_angle_pid_.reset();
        *bullet_feeder_control_torque_ = kNan;

        bullet_feeder_faulty_time_ = hcs_sync::Duration::zero();
        bullet_feeder_cool_down_ = hcs_sync::Duration::zero();
    }

    void set_preloading() {
        logger().rt().info("PRELOADING");
        bullet_fed_count_++;
        shoot_stage_ = ShootStage::kPreloading;
        bullet_feeder_control_angle_ =
            kCompressedZeroPoint + (bullet_fed_count_ + 0.5) * kAnglePerBullet;
        bullet_feeder_angle_pid_.output_max = 1.0;
    }

    void set_preloaded() {
        logger().rt().info("PRELOADED");
        shoot_stage_ = ShootStage::kPreloaded;
    }

    void set_compressing() {
        logger().rt().info("COMPRESSING");
        shoot_stage_ = ShootStage::kCompressing;
        bullet_feeder_control_angle_ =
            kCompressedZeroPoint + (bullet_fed_count_ + 1) * kAnglePerBullet;
        bullet_feeder_angle_pid_.output_max = 0.8;
    }

    void set_compressed() {
        logger().rt().info("COMPRESSED");
        shoot_stage_ = ShootStage::kCompressed;
    }

    void set_shooting() {
        logger().rt().info("SHOOTING");
        shoot_stage_ = ShootStage::kShooting;
        bullet_feeder_control_angle_ =
            kCompressedZeroPoint + (bullet_fed_count_ + 1.2) * kAnglePerBullet;
        bullet_feeder_angle_pid_.output_max = 1.0;
    }

    void update_jam_detection(const hcs_sync::Tick& tick) {
        if (*bullet_feeder_control_torque_ < kJamTorque) {
            bullet_feeder_faulty_time_ = hcs_sync::Duration::zero();
            return;
        }

        if (bullet_feeder_faulty_time_ < kJamTimeout)
            bullet_feeder_faulty_time_ += tick.dt;
        else {
            bullet_feeder_faulty_time_ = hcs_sync::Duration::zero();
            enter_jam_protection();
        }
    }

    void enter_jam_protection() {
        bullet_feeder_control_angle_ = kNan;
        bullet_feeder_cool_down_ = kJamCoolDown;
        bullet_feeder_angle_pid_.reset();
        bullet_feeder_velocity_pid_.reset();
        logger().rt().info("Jammed!");
    }

    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kInf = std::numeric_limits<double>::infinity();

    /// 拨盘把弹丸压到整格位置时的角度（第 0 格）。
    static constexpr double kCompressedZeroPoint = 0.58;
    static constexpr double kAnglePerBullet = 2 * std::numbers::pi / 6;

    static constexpr double kJamTorque = 300.0;
    static constexpr hcs_sync::Duration kJamTimeout = std::chrono::milliseconds{1000};
    static constexpr hcs_sync::Duration kJamCoolDown = std::chrono::milliseconds{1000};
    /// 冷却还剩这么多的时候不再反转，松开。
    static constexpr hcs_sync::Duration kJamReleaseTime = std::chrono::milliseconds{500};

    InputInterface<bool> friction_ready_;
    InputInterface<bool> bullet_fired_;

    InputInterface<hcs_msgs::Switch> switch_right_;
    InputInterface<hcs_msgs::Switch> switch_left_;
    InputInterface<hcs_msgs::Mouse> mouse_;
    InputInterface<hcs_msgs::Keyboard> keyboard_;

    hcs_msgs::Switch last_switch_right_ = hcs_msgs::Switch::UNKNOWN;
    hcs_msgs::Switch last_switch_left_ = hcs_msgs::Switch::UNKNOWN;
    hcs_msgs::Mouse last_mouse_ = hcs_msgs::Mouse::zero();
    hcs_msgs::Keyboard last_keyboard_ = hcs_msgs::Keyboard::zero();

    bool overdrive_mode_ = false;
    bool low_latency_mode_ = false;

    InputInterface<double> bullet_feeder_angle_;
    InputInterface<double> bullet_feeder_velocity_;

    InputInterface<std::int64_t> control_bullet_allowance_limited_by_heat_;

    enum class ShootStage { kPreloading, kPreloaded, kCompressing, kCompressed, kShooting };
    ShootStage shoot_stage_ = ShootStage::kPreloaded;
    int bullet_fed_count_ = std::numeric_limits<int>::min();
    double bullet_feeder_control_angle_ = kNan;

    pid::PidCalculator bullet_feeder_velocity_pid_;
    pid::PidCalculator bullet_feeder_angle_pid_;
    OutputInterface<double> bullet_feeder_control_torque_;

    hcs_sync::Duration bullet_feeder_faulty_time_{};
    hcs_sync::Duration bullet_feeder_cool_down_{};

    OutputInterface<hcs_msgs::ShootMode> shoot_mode_;
};

} // namespace hcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    hcs_core::controller::shooting::BulletFeederController42mm, hcs_executor::Component)
