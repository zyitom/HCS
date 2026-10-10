#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_msgs/keyboard.hpp>
#include <hcs_msgs/mouse.hpp>
#include <hcs_msgs/shoot_condiction.hpp>
#include <hcs_msgs/shoot_mode.hpp>
#include <hcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace hcs_core::controller::shooting {

// 推杆式发射机构（英雄）：拨弹盘把弹丸送到膛口，推杆把它推进摩擦轮。
// 移植自 RMCS 的 putter_controller.cpp。
//
// 机构上的事实：光电传感器装在膛口。实测双中位会先让推杆退回，再靠堵转判定复位完成。
// 平时给推杆一个很小的回拉力，免得它滑下来。
//
// 过程：
//
//   上电 / 离开安全态     推杆以 -80 的速度退到底，堵转 50 ms 判定到位（记下起点）
//   PRELOADING           拨弹盘以恒定速度转；转不动了（堵转 150 ms）：
//                          光电被挡住 → 弹丸到位，进 PRELOADED
//                          不管到没到位，都反转一小段再继续（没到位的话就是单纯卡了一下）
//   PRELOADED            等开火
//   SHOOTING             推杆以 120 的速度前推，堵转 50 ms 当作弹丸已经打出去；
//                        然后以 -50 的速度退回，退 400 ms 后回到 PRELOADING
//
// 什么时候打（都要摩擦轮就绪、热量够、弹丸已到位）：
//   手动   鼠标左键按下（没按右键时），或左拨杆 中 → 下
//   自瞄   右拨杆 上 或鼠标右键按着，且 /auto_aim/should_shoot 为真；两发之间至少隔 1 s
//   强制   按着鼠标右键时 500 ms 内连点两下左键
//
// 安全态（任一拨杆 UNKNOWN，或双下）：两个力矩给定都是 NaN，推杆回到"未初始化"。
//
// 参数：bullet_feeder_velocity_{kp,ki,kd}、putter_return_velocity_{kp,ki,kd}，
//       以及各自可选的 _integral_min / _integral_max / _output_min / _output_max。
//
// 所有计时都按 Tick 的时间累计，不数拍，也不另外读时钟。
class PutterController
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    PutterController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        const auto set_pid_parameter = [this](pid::PidCalculator& pid, const std::string& name) {
            pid.kp = get_parameter(name + "_kp").as_double();
            pid.ki = get_parameter(name + "_ki").as_double();
            pid.kd = get_parameter(name + "_kd").as_double();
            get_parameter(name + "_integral_min", pid.integral_min);
            get_parameter(name + "_integral_max", pid.integral_max);
            get_parameter(name + "_output_min", pid.output_min);
            get_parameter(name + "_output_max", pid.output_max);
        };

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

        register_input("/gimbal/photoelectric_sensor", photoelectric_sensor_status_);
        register_input("/gimbal/grayscale_sensor", grayscale_sensor_status_);
        register_input("/gimbal/bullet_fired", bullet_fired_);

        register_input("/gimbal/putter/angle", putter_angle_);
        register_input("/gimbal/putter/velocity", putter_velocity_);

        set_pid_parameter(bullet_feeder_velocity_pid_, "bullet_feeder_velocity");
        set_pid_parameter(putter_return_velocity_pid_, "putter_return_velocity");

        register_output(
            "/gimbal/bullet_feeder/control_torque", bullet_feeder_control_torque_, kNan);
        register_output("/gimbal/putter/control_torque", putter_control_torque_, kNan);

        register_output("/gimbal/shoot/delay_ms", shoot_delay_ms_, kNan);

        // 自瞄没接的时候这个输入没有上游，读出来是静止的 false。
        register_input("/auto_aim/should_shoot", should_shoot_, false);

        register_output("/gimbal/shooter/mode", shoot_mode_, hcs_msgs::ShootMode::SINGLE);
        register_output("/gimbal/shooter/condiction", shoot_condiction_);
        register_output("/gimbal/shooter/preloaded_ready", preloaded_ready_, false);
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

        const auto zero = hcs_sync::Duration::zero();
        if (putter_initialized_) {
            if (bullet_feeder_reverse_remaining_ > zero) {
                // 拨弹盘卡弹保护：前一段反转，后一段松开。
                bullet_feeder_reverse_remaining_ =
                    std::max(zero, bullet_feeder_reverse_remaining_ - tick.dt);

                if (bullet_feeder_reverse_remaining_ > kReverseReleaseTime)
                    *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(
                        -kPreloadVelocity / 2 - *bullet_feeder_velocity_);
                else {
                    bullet_feeder_velocity_pid_.reset();
                    *bullet_feeder_control_torque_ = 0.0;
                }

                // 反转结束的那一拍，如果弹丸已经到位，就报"上膛完成"。
                *preloaded_ready_ = bullet_feeder_reverse_remaining_ == zero
                                 && shoot_stage_ == ShootStage::kPreloaded;
            } else {
                if (*friction_ready_) {
                    if (switch_right != Switch::DOWN)
                        update_trigger(tick, switch_right, switch_left, mouse);

                    if (shoot_stage_ == ShootStage::kPreloading) {
                        *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(
                            kPreloadVelocity - *bullet_feeder_velocity_);

                        update_locked_detection(tick);
                    }

                    if (shoot_stage_ == ShootStage::kShooting) {
                        if (shot_) {
                            // 打出去了：推杆退回。
                            *putter_control_torque_ =
                                putter_return_velocity_pid_.update(-50. - *putter_velocity_);
                            putter_timeout_detection(tick);
                        } else {
                            // 还没打出去：继续往前推。
                            *putter_control_torque_ =
                                putter_return_velocity_pid_.update(120. - *putter_velocity_);
                            update_putter_jam_detection(tick);
                        }
                    }
                } else {
                    // 摩擦轮没就绪：拨弹盘停住。
                    *bullet_feeder_control_torque_ = 0.;
                }

                // 不在发射中：给推杆一个很小的回拉力。
                if (shoot_stage_ != ShootStage::kShooting)
                    *putter_control_torque_ = -0.02;
            }
        } else {
            // 推杆还没初始化：先退到底。
            *putter_control_torque_ = putter_return_velocity_pid_.update(-80. - *putter_velocity_);
            update_putter_jam_detection(tick);
        }

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_mouse_ = mouse;
        last_keyboard_ = keyboard;
    }

private:
    /// 看这一拍要不要开火。时间用 tick.scheduled：周期域里唯一允许的时间来源。
    void update_trigger(
        const hcs_sync::Tick& tick, hcs_msgs::Switch switch_right, hcs_msgs::Switch switch_left,
        hcs_msgs::Mouse mouse) {
        using hcs_msgs::Switch;
        const auto now = tick.scheduled;

        const bool left_click_edge = !last_mouse_.left && mouse.left;
        if (left_click_edge) {
            if (last_click_time_ && now - *last_click_time_ < kDoubleClickWindow)
                click_count_++;
            else
                click_count_ = 1;
            last_click_time_ = now;
        }

        const bool manual_trigger =
            (left_click_edge && !mouse.right)
            || (last_switch_left_ == Switch::MIDDLE && switch_left == Switch::DOWN);

        const bool auto_fire_now = (switch_right == Switch::UP || mouse.right) && *should_shoot_;
        const bool auto_trigger_emergence = mouse.right && (click_count_ >= 2);
        const bool auto_trigger =
            auto_fire_now && (!last_fire_time_ || now - *last_fire_time_ > kAutoFireInterval);

        if (manual_trigger || auto_trigger || auto_trigger_emergence) {
            if (*control_bullet_allowance_limited_by_heat_ > 0
                && (shoot_stage_ == ShootStage::kPreloaded || shoot_first_)) {
                set_shooting();
                last_fire_time_ = now;
                shoot_first_ = false;
            }
        }
        if (auto_trigger_emergence)
            click_count_ = 0;
    }

    void reset_all_controls() {
        last_switch_right_ = hcs_msgs::Switch::UNKNOWN;
        last_switch_left_ = hcs_msgs::Switch::UNKNOWN;
        last_mouse_ = hcs_msgs::Mouse::zero();
        last_keyboard_ = hcs_msgs::Keyboard::zero();

        bullet_feeder_velocity_pid_.reset();
        *bullet_feeder_control_torque_ = kNan;

        shoot_stage_ = ShootStage::kPreloaded;

        putter_initialized_ = false;
        putter_startpoint_ = kNan;
        putter_return_velocity_pid_.reset();
        *putter_control_torque_ = kNan;

        *shoot_delay_ms_ = kNan;
    }

    void set_preloading() {
        logger().rt().info("PRELOADING");
        shoot_stage_ = ShootStage::kPreloading;
    }

    void set_preloaded() {
        logger().rt().info("PRELOADED");
        shoot_stage_ = ShootStage::kPreloaded;
    }

    void set_shooting() {
        logger().rt().info("SHOOTING");
        shoot_stage_ = ShootStage::kShooting;
    }

    /// 拨弹盘在出力却几乎不转，持续够久就是顶住了：光电被挡住说明弹丸到位；
    /// 不管哪种情况都反转一小段。
    void update_locked_detection(const hcs_sync::Tick& tick) {
        if (*bullet_feeder_velocity_ < 0.5 && *bullet_feeder_control_torque_ > 0.1)
            locked_time_ += tick.dt;
        else
            locked_time_ = hcs_sync::Duration::zero();

        if (locked_time_ > kLockedTimeout) {
            if (*photoelectric_sensor_status_)
                set_preloaded();
            enter_reverse_protection();
        }
    }

    /// 推杆几乎不动，持续够久就是堵转了。不在发射中：推杆退到底了，初始化完成。
    /// 发射中：推到头了，当作弹丸已经打出去。
    void update_putter_jam_detection(const hcs_sync::Tick& tick) {
        if (std::abs(*putter_velocity_) > 0.1 || std::isnan(*putter_control_torque_))
            putter_faulty_time_ = hcs_sync::Duration::zero();
        else
            putter_faulty_time_ += tick.dt;

        if (putter_faulty_time_ >= kPutterStallTimeout) {
            putter_faulty_time_ = hcs_sync::Duration::zero();
            if (shoot_stage_ != ShootStage::kShooting) {
                putter_initialized_ = true;
                putter_startpoint_ = *putter_angle_;
            } else {
                logger().rt().info("DETECT: Putter freezed");
                shot_ = true;
            }
        }
    }

    /// 推杆退回的时间到了：当作已经退到位，开始下一发的上膛。
    void putter_timeout_detection(const hcs_sync::Tick& tick) {
        if (shoot_stage_ != ShootStage::kShooting || !shot_)
            return;

        if (putter_return_time_ < kPutterReturnTime)
            putter_return_time_ += tick.dt;
        else {
            putter_return_time_ = hcs_sync::Duration::zero();
            logger().rt().info("PUTTER TIMEOUT");
            set_preloading();
            shot_ = false;
        }
    }

    void enter_reverse_protection() {
        locked_time_ = hcs_sync::Duration::zero();
        bullet_feeder_reverse_remaining_ = kReverseTime;
        bullet_feeder_velocity_pid_.reset();
    }

    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    /// 上膛时拨弹盘的转速。
    static constexpr double kPreloadVelocity = 5.0;

    static constexpr hcs_sync::Duration kDoubleClickWindow = std::chrono::milliseconds{500};
    static constexpr hcs_sync::Duration kAutoFireInterval = std::chrono::milliseconds{1000};
    static constexpr hcs_sync::Duration kLockedTimeout = std::chrono::milliseconds{150};
    static constexpr hcs_sync::Duration kPutterStallTimeout = std::chrono::milliseconds{50};
    static constexpr hcs_sync::Duration kPutterReturnTime = std::chrono::milliseconds{400};
    static constexpr hcs_sync::Duration kReverseTime = std::chrono::milliseconds{400};
    /// 反转保护还剩这么多的时候不再反转，松开。
    static constexpr hcs_sync::Duration kReverseReleaseTime = std::chrono::milliseconds{300};

    InputInterface<bool> photoelectric_sensor_status_;
    InputInterface<bool> grayscale_sensor_status_;
    InputInterface<bool> bullet_fired_;
    bool shot_ = false;        ///< 这一发已经打出去了（推杆推到头）
    bool shoot_first_ = true;  ///< 开机后的第一发不要求弹丸已到位

    InputInterface<bool> friction_ready_;

    InputInterface<hcs_msgs::Switch> switch_right_;
    InputInterface<hcs_msgs::Switch> switch_left_;
    InputInterface<hcs_msgs::Mouse> mouse_;
    InputInterface<hcs_msgs::Keyboard> keyboard_;

    hcs_msgs::Switch last_switch_right_ = hcs_msgs::Switch::UNKNOWN;
    hcs_msgs::Switch last_switch_left_ = hcs_msgs::Switch::UNKNOWN;
    hcs_msgs::Mouse last_mouse_ = hcs_msgs::Mouse::zero();
    hcs_msgs::Keyboard last_keyboard_ = hcs_msgs::Keyboard::zero();

    InputInterface<double> bullet_feeder_angle_;
    InputInterface<double> bullet_feeder_velocity_;

    InputInterface<std::int64_t> control_bullet_allowance_limited_by_heat_;

    bool putter_initialized_ = false;
    hcs_sync::Duration putter_faulty_time_{};
    hcs_sync::Duration putter_return_time_{};
    double putter_startpoint_ = kNan; ///< 推杆退到底时的角度
    pid::PidCalculator putter_return_velocity_pid_;
    InputInterface<double> putter_velocity_;

    enum class ShootStage { kPreloading, kPreloaded, kShooting };
    ShootStage shoot_stage_ = ShootStage::kPreloading;

    pid::PidCalculator bullet_feeder_velocity_pid_;

    OutputInterface<double> bullet_feeder_control_torque_;

    InputInterface<double> putter_angle_;
    OutputInterface<double> putter_control_torque_;

    OutputInterface<double> shoot_delay_ms_;

    InputInterface<bool> should_shoot_;
    std::optional<hcs_sync::Timestamp> last_fire_time_;  ///< 空：还没打过
    std::optional<hcs_sync::Timestamp> last_click_time_; ///< 空：还没点过
    int click_count_ = 0;

    hcs_sync::Duration locked_time_{};
    hcs_sync::Duration bullet_feeder_reverse_remaining_{};

    OutputInterface<hcs_msgs::ShootMode> shoot_mode_;
    OutputInterface<hcs_msgs::ShootCondiction> shoot_condiction_;
    OutputInterface<bool> preloaded_ready_;
};

} // namespace hcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::controller::shooting::PutterController, hcs_executor::Component)
