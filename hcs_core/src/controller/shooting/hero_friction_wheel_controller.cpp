#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_msgs/keyboard.hpp>
#include <hcs_msgs/switch.hpp>

namespace hcs_core::controller::shooting {

// 英雄的摩擦轮：两套转速档（profile 0 / 1）之间切换，其余同 FrictionWheelController。
// 移植自 RMCS 的 hero_friction_wheel_controller.cpp。
//
// 参数：
//   friction_wheels                 各摩擦轮的话题前缀，第一个是"主轮"
//   friction_velocities_profile_0   第 0 档各轮的转速
//   friction_velocities_profile_1   第 1 档各轮的转速
//   friction_soft_start_stop_time   从 0 升到工作转速（或反过来）用多少秒
//
// 操作：
//   键盘 V，或左拨杆 中 → 上        开 / 关摩擦轮
//   键盘 F                          切换转速档
//
// 安全态（任一拨杆 UNKNOWN，或双下）与摩擦轮关着的时候：给定是 NaN，状态输出都是 false。
//
// 从 RMCS 原样带过来、没有改的三处（上车前要对着实物确认是不是本意）：
//   - 按 F 会切一次档，Ctrl + F 的上升沿又切一次——所以按住 Ctrl 再按 F 等于切两次、回到原档。
//   - detect_friction_faulty() 两个分支都返回 false：卡住检测实际上是关着的。
//   - 出弹判断里拿来比的是第 2 个轮（下标 2）的目标转速，而不是主轮的：少于三个摩擦轮时
//     这个下标不存在，所以构造时要求至少三个。
class HeroFrictionWheelController
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    HeroFrictionWheelController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);

        const auto friction_wheels = get_parameter("friction_wheels").as_string_array();
        const auto profile_0 = get_parameter("friction_velocities_profile_0").as_double_array();
        const auto profile_1 = get_parameter("friction_velocities_profile_1").as_double_array();
        if (friction_wheels.size() != profile_0.size()
            || friction_wheels.size() != profile_1.size())
            throw std::runtime_error(
                "'friction_wheels' and both friction velocity profiles must have the same length!");
        if (friction_wheels.size() <= kFireReferenceWheel)
            throw std::runtime_error(
                "'friction_wheels' needs at least three wheels: bullet detection compares against "
                "the target velocity of the third one");

        friction_profile_0_.assign(profile_0.begin(), profile_0.end());
        friction_profile_1_.assign(profile_1.begin(), profile_1.end());
        friction_count_ = friction_wheels.size();
        friction_velocities_ = std::make_unique<InputInterface<double>[]>(friction_count_);
        friction_control_velocities_ = std::make_unique<OutputInterface<double>[]>(friction_count_);
        for (std::size_t i = 0; i < friction_count_; i++) {
            register_input(friction_wheels[i] + "/velocity", friction_velocities_[i]);
            register_output(
                friction_wheels[i] + "/control_velocity", friction_control_velocities_[i], kNan);
        }

        friction_soft_start_stop_time_ =
            get_parameter("friction_soft_start_stop_time").as_double();

        register_output("/gimbal/friction_ready", friction_ready_, false);
        register_output("/gimbal/friction_jammed", friction_jammed_, false);
        register_output("/gimbal/bullet_fired", bullet_fired_, false);
        register_output("/gimbal/friction_profile_1_active", friction_profile_1_active_, false);
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto keyboard = *keyboard_;

        if (!last_keyboard_.f && keyboard.f)
            switch_profile();

        const bool profile_switch_now = keyboard.ctrl && keyboard.f;
        const bool profile_switch_last = last_keyboard_.ctrl && last_keyboard_.f;
        if (!profile_switch_last && profile_switch_now)
            switch_profile();

        *friction_profile_1_active_ = active_profile_ == 1;

        using hcs_msgs::Switch;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            return;
        }

        if (switch_right != Switch::DOWN) {
            if ((!last_keyboard_.v && keyboard.v)
                || (last_switch_left_ == Switch::MIDDLE && switch_left == Switch::UP)) {
                friction_enabled_ = !friction_enabled_;
            }

            update_friction_velocities(tick);
            update_friction_status(tick);
            if (*friction_jammed_)
                logger().rt().info("Friction Jammed!");
            if (*bullet_fired_)
                logger().rt().info("Bullet Fired!");

            last_switch_right_ = switch_right;
            last_switch_left_ = switch_left;
            last_keyboard_ = keyboard;
        }
        if (!friction_enabled_)
            reset_all_controls();
    }

private:
    void reset_all_controls() {
        friction_enabled_ = false;

        last_primary_friction_velocity_ = kNan;
        primary_friction_velocity_decrease_integral_ = 0;

        friction_soft_start_stop_percentage_ = kNan;

        for (std::size_t i = 0; i < friction_count_; i++)
            *friction_control_velocities_[i] = kNan;

        *friction_ready_ = *friction_jammed_ = *bullet_fired_ = false;
    }

    /// 缓启停的进度按"当前转速占目标转速的几成"重新估一遍。
    void resync_soft_start_stop_percentage() {
        double sum = 0.0;
        for (std::size_t i = 0; i < friction_count_; i++)
            sum += *friction_velocities_[i] / target_friction_velocity(i);
        friction_soft_start_stop_percentage_ = sum / static_cast<double>(friction_count_);
    }

    void switch_profile() {
        active_profile_ ^= 1;

        if (!std::isnan(friction_soft_start_stop_percentage_)) {
            resync_soft_start_stop_percentage();
            friction_soft_start_stop_percentage_ =
                std::clamp(friction_soft_start_stop_percentage_, 0.0, 1.0);
        }
    }

    void update_friction_velocities(const hcs_sync::Tick& tick) {
        // 刚离开安全态：从摩擦轮现在的实际转速接着走，而不是从 0 开始。
        if (std::isnan(friction_soft_start_stop_percentage_))
            resync_soft_start_stop_percentage();

        const double step = tick.dt_seconds() / friction_soft_start_stop_time_;
        friction_soft_start_stop_percentage_ += friction_enabled_ ? step : -step;
        friction_soft_start_stop_percentage_ =
            std::clamp(friction_soft_start_stop_percentage_, 0.0, 1.0);

        for (std::size_t i = 0; i < friction_count_; i++)
            *friction_control_velocities_[i] =
                friction_soft_start_stop_percentage_ * target_friction_velocity(i);
    }

    void update_friction_status(const hcs_sync::Tick& tick) {
        *friction_ready_ = *friction_jammed_ = *bullet_fired_ = false;

        if (!friction_enabled_)
            return;
        if (friction_soft_start_stop_percentage_ < 1.0)
            return;

        if (detect_friction_faulty()) {
            if (friction_faulty_time_ >= kFaultyTimeout) {
                friction_enabled_ = false;
                *friction_jammed_ = true;
            } else {
                friction_faulty_time_ += tick.dt;
                *friction_ready_ = true;
            }
            return;
        }

        *friction_ready_ = true;
        *bullet_fired_ = detect_bullet_fire();
    }

    /// RMCS 原样：循环里和循环后都返回 false，所以它从不报卡住。见文件头的说明。
    [[nodiscard]] bool detect_friction_faulty() const {
        for (std::size_t i = 0; i < friction_count_; i++) {
            if (std::abs(*friction_velocities_[i])
                < std::abs(*friction_control_velocities_[i] * 0.5))
                return false;
        }
        return false;
    }

    /// 只盯主轮（列表里的第一个）的转速：打出一发时它会掉速再回升。把连续下降的量攒起来，
    /// 等它开始回升时看攒了多少。
    bool detect_bullet_fire() {
        bool fired = false;
        if (!std::isnan(last_primary_friction_velocity_)) {
            const double differential = *friction_velocities_[0] - last_primary_friction_velocity_;
            if (differential < 0.1)
                primary_friction_velocity_decrease_integral_ += differential;
            else {
                if (primary_friction_velocity_decrease_integral_ < -14.0
                    && last_primary_friction_velocity_
                           < target_friction_velocity(kFireReferenceWheel) - 25.0)
                    fired = true;

                primary_friction_velocity_decrease_integral_ = 0;
            }
        }
        last_primary_friction_velocity_ = *friction_velocities_[0];

        return fired;
    }

    [[nodiscard]] double target_friction_velocity(std::size_t i) const {
        return active_profile_ == 0 ? friction_profile_0_[i] : friction_profile_1_[i];
    }

    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    static constexpr hcs_sync::Duration kFaultyTimeout = std::chrono::milliseconds{200};
    /// 出弹判断拿哪个轮的目标转速来比。RMCS 原样，见文件头的说明。
    static constexpr std::size_t kFireReferenceWheel = 2;

    InputInterface<hcs_msgs::Switch> switch_right_;
    InputInterface<hcs_msgs::Switch> switch_left_;
    InputInterface<hcs_msgs::Keyboard> keyboard_;

    hcs_msgs::Switch last_switch_right_ = hcs_msgs::Switch::UNKNOWN;
    hcs_msgs::Switch last_switch_left_ = hcs_msgs::Switch::UNKNOWN;
    hcs_msgs::Keyboard last_keyboard_ = hcs_msgs::Keyboard::zero();

    std::size_t friction_count_;

    std::vector<double> friction_profile_0_;
    std::vector<double> friction_profile_1_;
    std::size_t active_profile_ = 0;

    std::unique_ptr<InputInterface<double>[]> friction_velocities_;

    bool friction_enabled_ = false;

    double friction_soft_start_stop_time_; ///< 秒
    double friction_soft_start_stop_percentage_ = kNan;
    std::unique_ptr<OutputInterface<double>[]> friction_control_velocities_;

    OutputInterface<bool> friction_ready_;

    hcs_sync::Duration friction_faulty_time_{};
    OutputInterface<bool> friction_jammed_;

    double last_primary_friction_velocity_ = kNan;
    double primary_friction_velocity_decrease_integral_ = 0;
    OutputInterface<bool> bullet_fired_;
    OutputInterface<bool> friction_profile_1_active_;
};

} // namespace hcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    hcs_core::controller::shooting::HeroFrictionWheelController, hcs_executor::Component)
