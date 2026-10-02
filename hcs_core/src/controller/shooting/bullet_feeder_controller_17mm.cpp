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

namespace hcs_core::controller::shooting {

// 17 mm 拨弹盘：按"还能打几发"给拨弹盘转速，卡弹时反转退弹。
// 移植自 RMCS 的 bullet_feeder_controller_17mm.cpp。
//
// 参数（时间单位都是秒，频率是每秒几发）：
//   bullets_per_feeder_turn      拨弹盘转一圈拨几发
//   shot_frequency               连发时的射频
//   safe_shot_frequency          只剩一发余量时的射频（慢一点，免得多打一发超热量）
//   eject_frequency / eject_time            卡弹后反转退弹的速度与时长（头两次）
//   deep_eject_frequency / deep_eject_time  连着卡第三次起，退得更狠的速度与时长
//   single_shot_max_stop_delay   单发模式下，按一下之后最多转多久（等不到出弹信号也停）
//
// 什么时候打：
//   连发（默认）  自瞄开着（鼠标右键 / 右拨杆 上）：听 /auto_aim/should_shoot
//                 否则：鼠标左键按着 / 左拨杆 下
//   单发          鼠标左键按下的沿、左拨杆拨到 下 的沿、或自瞄给出的单发请求，各打一发
//                 （左拨杆拨到 下 之后的 500 ms 内，以及自瞄要求单发时，处在单发模式）
//
// 摩擦轮没就绪时不拨弹。安全态（任一拨杆 UNKNOWN，或双下）：给定是 NaN。
//
// 所有计时都按 Tick::dt 累计，不数拍。
class BulletFeederController17mm
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    BulletFeederController17mm()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        const double bullets_per_feeder_turn = get_parameter("bullets_per_feeder_turn").as_double();
        const double angle_per_bullet = 2 * std::numbers::pi / bullets_per_feeder_turn;

        bullet_feeder_working_velocity_ =
            angle_per_bullet * get_parameter("shot_frequency").as_double();
        bullet_feeder_safe_shot_velocity_ =
            angle_per_bullet * get_parameter("safe_shot_frequency").as_double();

        bullet_feeder_eject_velocity_ =
            -angle_per_bullet * get_parameter("eject_frequency").as_double();
        bullet_feeder_eject_time_ = seconds(get_parameter("eject_time").as_double());

        bullet_feeder_deep_eject_velocity_ =
            -angle_per_bullet * get_parameter("deep_eject_frequency").as_double();
        bullet_feeder_deep_eject_time_ = seconds(get_parameter("deep_eject_time").as_double());

        single_shot_max_stop_delay_ =
            seconds(get_parameter("single_shot_max_stop_delay").as_double());

        register_input("/gimbal/friction_ready", friction_ready_);
        register_input("/gimbal/bullet_fired", bullet_fired_);
        register_input(
            "/gimbal/control_bullet_allowance/limited_by_heat",
            control_bullet_allowance_limited_by_heat_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        // 自瞄没接的时候这两个输入没有上游，读出来是静止的 false。
        register_input("/auto_aim/should_shoot", should_shoot_, false);
        register_input("/auto_aim/single_shoot", single_shoot_, false);

        register_input("/gimbal/bullet_feeder/velocity", bullet_feeder_velocity_);
        register_output(
            "/gimbal/bullet_feeder/control_velocity", bullet_feeder_control_velocity_, kNan);

        register_output("/gimbal/shooter/mode", shoot_mode_, hcs_msgs::ShootMode::AUTOMATIC);
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        auto& shoot_mode = *shoot_mode_;

        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto mouse = *mouse_;
        const auto keyboard = *keyboard_;

        using hcs_msgs::ShootMode;
        using hcs_msgs::Switch;
        const bool current_should_shoot = *should_shoot_;

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
        } else {
            std::int64_t bullet_allowance = 0;

            if (switch_right != Switch::DOWN) {
                // 单发只给打符用，所以不再让操作手用按键切：默认就是连发。
                shoot_mode = ShootMode::AUTOMATIC;

                single_shot_stop_remaining_ = count_down(single_shot_stop_remaining_, tick);
                temporary_single_shot_remaining_ =
                    count_down(temporary_single_shot_remaining_, tick);

                const auto current_single_shoot = *single_shoot_;

                if (!last_mouse_.left && mouse.left) {
                    single_shot_stop_remaining_ = single_shot_max_stop_delay_;
                } else if (last_switch_left_ != Switch::DOWN && switch_left == Switch::DOWN) {
                    single_shot_stop_remaining_ = single_shot_max_stop_delay_;
                    temporary_single_shot_remaining_ = kTemporarySingleShotTime;
                } else if (current_single_shoot && current_should_shoot && !last_should_shoot_) {
                    single_shot_stop_remaining_ = single_shot_max_stop_delay_;
                }

                if (temporary_single_shot_remaining_ > hcs_sync::Duration::zero())
                    shoot_mode = ShootMode::SINGLE;
                if (current_single_shoot)
                    shoot_mode = ShootMode::SINGLE;

                // 打出去了：这一发的任务完成，不用等超时。
                if (*bullet_fired_)
                    single_shot_stop_remaining_ = hcs_sync::Duration::zero();

                if (*friction_ready_) {
                    if (shoot_mode == ShootMode::AUTOMATIC) {
                        const bool aiming_enable = mouse.right || (switch_right == Switch::UP);
                        const bool attack_intent = mouse.left || (switch_left == Switch::DOWN);
                        const bool triggered = aiming_enable ? current_should_shoot : attack_intent;
                        bullet_allowance =
                            triggered ? *control_bullet_allowance_limited_by_heat_ : 0;
                    } else {
                        const bool triggered =
                            single_shot_stop_remaining_ > hcs_sync::Duration::zero();
                        bullet_allowance =
                            triggered && (*control_bullet_allowance_limited_by_heat_ > 0);
                    }
                }
            }

            update_bullet_feeder_velocity(bullet_allowance, tick);
        }

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_mouse_ = mouse;
        last_keyboard_ = keyboard;
        last_should_shoot_ = current_should_shoot;
    }

private:
    [[nodiscard]] static hcs_sync::Duration seconds(double value) {
        return std::chrono::duration_cast<hcs_sync::Duration>(
            std::chrono::duration<double>{value});
    }

    /// 倒计时走一拍，到 0 为止。
    [[nodiscard]] static hcs_sync::Duration count_down(
        hcs_sync::Duration remaining, const hcs_sync::Tick& tick) noexcept {
        return std::max(hcs_sync::Duration::zero(), remaining - tick.dt);
    }

    void reset_all_controls() {
        *shoot_mode_ = hcs_msgs::ShootMode::AUTOMATIC;

        *bullet_feeder_control_velocity_ = kNan;
    }

    void update_bullet_feeder_velocity(std::int64_t bullet_allowance, const hcs_sync::Tick& tick) {
        if (bullet_allowance <= 0) {
            bullet_feeder_working_status_ = hcs_sync::Duration::zero();
            *bullet_feeder_control_velocity_ = 0.0;
            return;
        }

        update_jam_detection(tick);

        // 退弹中：保持退弹的给定，等它退完。
        if (bullet_feeder_cool_down_ > hcs_sync::Duration::zero()) {
            bullet_feeder_cool_down_ = count_down(bullet_feeder_cool_down_, tick);
            return;
        }

        const double new_control_velocity = bullet_allowance > 1
                                              ? bullet_feeder_working_velocity_
                                              : bullet_feeder_safe_shot_velocity_;
        // 给定变大了（从停到转、从慢到快）：之前攒的"转得正常"不算数，重新开始看。
        if (new_control_velocity > *bullet_feeder_control_velocity_)
            bullet_feeder_working_status_ =
                std::min(hcs_sync::Duration::zero(), bullet_feeder_working_status_);
        *bullet_feeder_control_velocity_ = new_control_velocity;
    }

    /// bullet_feeder_working_status_ 是一个带符号的累计时间：
    ///   正：拨弹盘连续转得正常（转速过给定的一半）有多久，最多攒到 kJamWindow；
    ///   负：连续转不起来有多久，攒到 -kJamWindow 就判卡弹。
    /// 已经稳定转了 kJamWindow 之后突然转不动，不用等，立刻判卡弹（弹链本来是顺的，
    /// 这时候停下来只可能是卡了）。稳定转满 kJamWindow 才把"连着卡了几次"清零。
    void update_jam_detection(const hcs_sync::Tick& tick) {
        const double control_velocity = *bullet_feeder_control_velocity_;
        // 给定是正的才看（停着、退弹中、刚离开安全态给定还是 NaN，都不看）。
        // 写成 !(x > 0) 而不是 x <= 0：后者对 NaN 为假，会把 NaN 放进来。
        if (!(control_velocity > 0.0))
            return;

        const auto zero = hcs_sync::Duration::zero();
        if (*bullet_feeder_velocity_ > control_velocity / 2) {
            if (bullet_feeder_working_status_ < zero) {
                bullet_feeder_working_status_ = zero;
            } else if (bullet_feeder_working_status_ < kJamWindow) {
                bullet_feeder_working_status_ =
                    std::min(kJamWindow, bullet_feeder_working_status_ + tick.dt);
            } else {
                bullet_feeder_jammed_count_ = 0;
            }
        } else {
            if (bullet_feeder_working_status_ >= kJamWindow) {
                enter_jam_protection();
                logger().rt().info("Instant jammed! Count = {}", bullet_feeder_jammed_count_);
            } else if (bullet_feeder_working_status_ > zero) {
                bullet_feeder_working_status_ = zero;
            } else if (bullet_feeder_working_status_ > -kJamWindow) {
                bullet_feeder_working_status_ -= tick.dt;
            } else {
                enter_jam_protection();
                logger().rt().info("Jammed! Count = {}", bullet_feeder_jammed_count_);
            }
        }
    }

    void enter_jam_protection() {
        bullet_feeder_working_status_ = hcs_sync::Duration::zero();
        if (++bullet_feeder_jammed_count_ <= 2) {
            *bullet_feeder_control_velocity_ = bullet_feeder_eject_velocity_;
            bullet_feeder_cool_down_ = bullet_feeder_eject_time_;
        } else {
            *bullet_feeder_control_velocity_ = bullet_feeder_deep_eject_velocity_;
            bullet_feeder_cool_down_ = bullet_feeder_deep_eject_time_;
        }
    }

    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    static constexpr hcs_sync::Duration kJamWindow = std::chrono::milliseconds{500};
    static constexpr hcs_sync::Duration kTemporarySingleShotTime = std::chrono::milliseconds{500};

    double bullet_feeder_working_velocity_;
    double bullet_feeder_safe_shot_velocity_;
    double bullet_feeder_eject_velocity_;
    double bullet_feeder_deep_eject_velocity_;
    hcs_sync::Duration bullet_feeder_eject_time_;
    hcs_sync::Duration bullet_feeder_deep_eject_time_;

    hcs_sync::Duration single_shot_max_stop_delay_;
    hcs_sync::Duration single_shot_stop_remaining_{};

    InputInterface<bool> friction_ready_;
    InputInterface<bool> bullet_fired_;
    InputInterface<std::int64_t> control_bullet_allowance_limited_by_heat_;

    InputInterface<hcs_msgs::Switch> switch_right_;
    InputInterface<hcs_msgs::Switch> switch_left_;
    InputInterface<hcs_msgs::Mouse> mouse_;
    InputInterface<hcs_msgs::Keyboard> keyboard_;

    InputInterface<bool> should_shoot_;
    InputInterface<bool> single_shoot_;
    bool last_should_shoot_ = false;

    hcs_msgs::Switch last_switch_right_ = hcs_msgs::Switch::UNKNOWN;
    hcs_msgs::Switch last_switch_left_ = hcs_msgs::Switch::UNKNOWN;
    hcs_msgs::Mouse last_mouse_ = hcs_msgs::Mouse::zero();
    hcs_msgs::Keyboard last_keyboard_ = hcs_msgs::Keyboard::zero();

    InputInterface<double> bullet_feeder_velocity_;
    hcs_sync::Duration bullet_feeder_working_status_{}; ///< 带符号，见 update_jam_detection()
    int bullet_feeder_jammed_count_ = 0;
    hcs_sync::Duration bullet_feeder_cool_down_{};

    hcs_sync::Duration temporary_single_shot_remaining_{};
    OutputInterface<hcs_msgs::ShootMode> shoot_mode_;

    OutputInterface<double> bullet_feeder_control_velocity_;
};

} // namespace hcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    hcs_core::controller::shooting::BulletFeederController17mm, hcs_executor::Component)
