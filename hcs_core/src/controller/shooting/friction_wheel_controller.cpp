#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_msgs/keyboard.hpp>
#include <hcs_msgs/switch.hpp>

namespace hcs_core::controller::shooting {

// 摩擦轮：开关、缓启停、转速给定，以及从转速上看出来的三件事——就绪、卡住、打出了一发。
// 移植自 RMCS 的 friction_wheel_controller.cpp。
//
// 参数：
//   friction_wheels                 各摩擦轮的话题前缀，第一个是"主轮"（只看它的掉速判断出弹）
//   friction_velocities             各轮的工作转速，与上面一一对应
//   friction_velocities_low_mode    可选：低速档各轮的转速。给了且长度对得上才有低速档
//   friction_soft_start_stop_time   从 0 升到工作转速（或反过来）用多少秒
//
// 操作：
//   键盘 V，或左拨杆 中 → 上                     开 / 关摩擦轮
//   Ctrl + F 的同时滚鼠标滚轮                    每格 ±5 调工作转速（不超过配置值，不低于低速档）
//   左拨杆 下、右拨杆 中 时把拨轮拨到顶          切换低速档（要接了 DR16：VT13 没有拨轮）
//
// 安全态（任一拨杆 UNKNOWN，或双下）：摩擦轮关掉，给定是 NaN，三个状态输出都是 false。
//
// 所有计时都按 Tick::dt 累计，不数拍。
class FrictionWheelController
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    FrictionWheelController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/mouse/mouse_wheel", mouse_wheel_);
        register_input("/remote/rotary_knob", rotary_knob_, false);

        const auto friction_wheels = get_parameter("friction_wheels").as_string_array();
        const auto working_velocities = get_parameter("friction_velocities").as_double_array();
        if (friction_wheels.size() != working_velocities.size())
            throw std::runtime_error(
                "Mismatch in array sizes: "
                "'friction_wheels' and 'friction_velocities' must have the same length!");
        if (friction_wheels.empty())
            throw std::runtime_error(
                "Empty array error: 'friction_wheels' and 'friction_velocities' cannot be empty!");

        friction_count_ = friction_wheels.size();
        friction_working_velocities_ = std::make_unique<double[]>(friction_count_);
        friction_velocities_ = std::make_unique<InputInterface<double>[]>(friction_count_);
        friction_working_velocity_outputs_ =
            std::make_unique<OutputInterface<double>[]>(friction_count_);
        friction_control_velocities_ = std::make_unique<OutputInterface<double>[]>(friction_count_);
        if (has_parameter("friction_velocities_low_mode")) {
            const auto low_velocities =
                get_parameter("friction_velocities_low_mode").as_double_array();
            if (low_velocities.size() == friction_count_) {
                friction_working_velocities_low_ = std::make_unique<double[]>(friction_count_);
                for (std::size_t i = 0; i < friction_count_; i++)
                    friction_working_velocities_low_[i] = low_velocities[i];
                low_mode_enabled_ = true;
            }
        }
        for (std::size_t i = 0; i < friction_count_; i++) {
            friction_working_velocities_[i] = working_velocities[i];
            register_input(friction_wheels[i] + "/velocity", friction_velocities_[i]);
            register_output(
                friction_wheels[i] + "/working_velocity", friction_working_velocity_outputs_[i],
                friction_working_velocities_[i]);
            register_output(
                friction_wheels[i] + "/control_velocity", friction_control_velocities_[i], kNan);
        }

        friction_velocity_max_ = std::make_unique<double[]>(friction_count_);
        friction_velocity_min_ = std::make_unique<double[]>(friction_count_);
        for (std::size_t i = 0; i < friction_count_; i++) {
            friction_velocity_max_[i] = friction_working_velocities_[i];
            friction_velocity_min_[i] =
                low_mode_enabled_ ? friction_working_velocities_low_[i]
                                  : friction_working_velocities_[i];
        }

        friction_soft_start_stop_time_ =
            get_parameter("friction_soft_start_stop_time").as_double();

        register_output("/gimbal/friction_ready", friction_ready_, false);
        register_output("/gimbal/friction_jammed", friction_jammed_, false);
        register_output("/gimbal/bullet_fired", bullet_fired_, false);
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto keyboard = *keyboard_;

        using hcs_msgs::Switch;
        // 没接拨轮（VT13）时这个输入没有上游，读出来是静止的 0，永远到不了阈值。
        const bool knob_up = *rotary_knob_ <= -kKnobEdgeThreshold;
        if (low_mode_enabled_ && switch_left == Switch::DOWN && switch_right == Switch::MIDDLE
            && knob_up && !last_knob_up_)
            toggle_low_mode();

        last_knob_up_ = knob_up;

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            return;
        }

        if (switch_right != Switch::DOWN) {
            if (keyboard.ctrl && keyboard.f)
                update_friction_speed_by_mouse_wheel();
            else {
                wheel_accumulator_ = 0.0;
                wheel_tick_pending_ = false;
            }

            update_friction_working_velocity_outputs();

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
    }

private:
    void reset_all_controls() {
        friction_enabled_ = false;
        low_mode_active_ = false;

        last_primary_friction_velocity_ = kNan;
        primary_friction_velocity_decrease_integral_ = 0;

        friction_soft_start_stop_percentage_ = kNan;

        for (std::size_t i = 0; i < friction_count_; i++)
            *friction_control_velocities_[i] = kNan;

        *friction_ready_ = *friction_jammed_ = *bullet_fired_ = false;
    }

    /// 缓启停的进度按"当前转速占目标转速的几成"重新估一遍。换了目标转速（切档）之后要做这件事，
    /// 否则给定会从旧进度乘新目标的地方跳过去。
    void resync_soft_start_stop_percentage() {
        double sum = 0.0;
        for (std::size_t i = 0; i < friction_count_; i++)
            sum += *friction_velocities_[i] / target_friction_velocity(i);
        friction_soft_start_stop_percentage_ = sum / static_cast<double>(friction_count_);
    }

    void toggle_low_mode() {
        low_mode_active_ = !low_mode_active_;

        if (!std::isnan(friction_soft_start_stop_percentage_)) {
            resync_soft_start_stop_percentage();
            friction_soft_start_stop_percentage_ =
                std::clamp(friction_soft_start_stop_percentage_, 0.0, 1.0);
        }
    }

    /// 滚轮的读数是速度：累加起来过了阈值算"滚了一格"，滚轮停下来之前不再算第二格。
    void update_friction_speed_by_mouse_wheel() {
        wheel_accumulator_ += *mouse_wheel_;
        if (std::abs(*mouse_wheel_) < kWheelRestThreshold) {
            wheel_accumulator_ = 0.0;
            wheel_tick_pending_ = false;
        }
        if (wheel_tick_pending_)
            return;
        if (wheel_accumulator_ > kWheelTickThreshold) {
            wheel_tick_pending_ = true;
            adjust_friction_speed(kFrictionSpeedAdjustStep);
        } else if (wheel_accumulator_ < -kWheelTickThreshold) {
            wheel_tick_pending_ = true;
            adjust_friction_speed(-kFrictionSpeedAdjustStep);
        }
    }

    void adjust_friction_speed(double delta) {
        for (std::size_t i = 0; i < friction_count_; i++)
            friction_working_velocities_[i] = std::clamp(
                friction_working_velocities_[i] + delta, friction_velocity_min_[i],
                friction_velocity_max_[i]);
    }

    [[nodiscard]] double target_friction_velocity(std::size_t i) const {
        return low_mode_active_ ? friction_working_velocities_low_[i]
                                : friction_working_velocities_[i];
    }

    void update_friction_working_velocity_outputs() {
        for (std::size_t i = 0; i < friction_count_; i++)
            *friction_working_velocity_outputs_[i] = target_friction_velocity(i);
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
            // 转速不到给定的一半，累计满 kFaultyTimeout 就判卡住、关掉摩擦轮。
            // 累计的时间不清零（RMCS 原样）：是开机以来的总和，不是"连续"这么久。
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

    [[nodiscard]] bool detect_friction_faulty() const {
        for (std::size_t i = 0; i < friction_count_; i++) {
            if (*friction_velocities_[i] < *friction_control_velocities_[i] * 0.5)
                return true;
        }
        return false;
    }

    /// 只盯主轮（列表里的第一个）的转速：打出一发时它会掉速再回升。把连续下降的量攒起来，
    /// 等它开始回升时看攒了多少——降得够多、而且确实掉到了工作转速以下一截，才算打出了一发。
    bool detect_bullet_fire() {
        bool fired = false;

        if (!std::isnan(last_primary_friction_velocity_)) {
            const double differential = *friction_velocities_[0] - last_primary_friction_velocity_;
            if (differential < 0.1)
                primary_friction_velocity_decrease_integral_ += differential;
            else {
                if (primary_friction_velocity_decrease_integral_ < -14.0
                    && last_primary_friction_velocity_ < target_friction_velocity(0) - 20.0)
                    fired = true;

                primary_friction_velocity_decrease_integral_ = 0;
            }
        }
        last_primary_friction_velocity_ = *friction_velocities_[0];

        return fired;
    }

    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kKnobEdgeThreshold = 0.7;
    static constexpr double kFrictionSpeedAdjustStep = 5.0;
    static constexpr double kWheelTickThreshold = 0.005;
    static constexpr double kWheelRestThreshold = 1e-6;
    static constexpr hcs_sync::Duration kFaultyTimeout = std::chrono::milliseconds{200};

    InputInterface<hcs_msgs::Switch> switch_right_;
    InputInterface<hcs_msgs::Switch> switch_left_;
    InputInterface<hcs_msgs::Keyboard> keyboard_;
    InputInterface<double> mouse_wheel_;
    InputInterface<double> rotary_knob_;

    hcs_msgs::Switch last_switch_right_ = hcs_msgs::Switch::UNKNOWN;
    hcs_msgs::Switch last_switch_left_ = hcs_msgs::Switch::UNKNOWN;
    hcs_msgs::Keyboard last_keyboard_ = hcs_msgs::Keyboard::zero();
    bool last_knob_up_ = false;

    std::size_t friction_count_;

    std::unique_ptr<double[]> friction_working_velocities_;
    std::unique_ptr<double[]> friction_working_velocities_low_;
    std::unique_ptr<double[]> friction_velocity_min_;
    std::unique_ptr<double[]> friction_velocity_max_;
    bool low_mode_enabled_ = false;
    bool low_mode_active_ = false;

    double wheel_accumulator_ = 0.0;
    bool wheel_tick_pending_ = false;

    std::unique_ptr<InputInterface<double>[]> friction_velocities_;
    std::unique_ptr<OutputInterface<double>[]> friction_working_velocity_outputs_;

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
};

} // namespace hcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    hcs_core::controller::shooting::FrictionWheelController, hcs_executor::Component)
