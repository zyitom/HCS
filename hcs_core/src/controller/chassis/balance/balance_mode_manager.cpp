#include <algorithm>
#include <cmath>

#include <hcs_msgs/switch.hpp>
#include <limits>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>

#include "controller/chassis/balance/balance_types.hpp"
#include "controller/chassis/balance/control_math.hpp"

namespace hcs_core::controller::chassis::balance {

// ============================================================================
// 平衡模式状态机（对应 Helios Chassis_Balance_Status_Handle::FSM 及各模式块）。
//
// 模式集合按 Helios 的活状态来：失能、起身（两阶段）、平衡（NORMAL/SPIN）、
// 跳跃（五阶段）、飞坡、摔倒（fatal 等待）、自救、上台阶。TANK/TOUCH_DOWN
// 是死状态不搬（PORTING.md §9）。离地检测默认禁用 → 飞坡不可达、跳跃的
// 蹬伸确认只剩超时路径，与 Helios 当前行为一致。
//
// 输出：
//   /chassis/balance/mode | process | jump_phase | inverted
//   /chassis/{left,right}_leg/control_length  腿长斜坡后的目标
//   /chassis/{left,right}_leg/control_angle   摆动角斜坡后的目标（多圈绝对角）
//
// Helios 按拍的计数全部换成秒制（dt 用 tick.dt_seconds()）。
// ============================================================================

class BalanceModeManager
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    BalanceModeManager()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        register_input("/remote/switch/left", switch_left_);
        register_input("/chassis/control_mode", control_mode_);
        register_input("/chassis/balance/jump_count", jump_count_);
        register_input("/chassis/balance/recover_count", recover_count_);
        register_input("/chassis/balance/turn_count", turn_count_);
        register_input("/chassis/balance/fly_request", fly_request_);
        register_input("/chassis/balance/leg_length_level", leg_length_level_);
        register_input("/chassis/balance/pitch", body_pitch_);
        register_input("/chassis/balance/roll", body_roll_);
        register_input("/chassis/balance/velocity", body_velocity_);
        register_input("/chassis/left_leg/length", left_length_);
        register_input("/chassis/right_leg/length", right_length_);
        register_input("/chassis/left_leg/angle", left_angle_);
        register_input("/chassis/right_leg/angle", right_angle_);
        register_input("/chassis/left_leg/angle_total", left_angle_total_);
        register_input("/chassis/right_leg/angle_total", right_angle_total_);
        register_input("/chassis/left_leg/grounded", left_grounded_);
        register_input("/chassis/right_leg/grounded", right_grounded_);
        register_input("/chassis/balance/acceleration_forward", acceleration_forward_);
        register_input("/chassis/balance/acceleration_lateral", acceleration_lateral_);

        register_output("/chassis/balance/mode", mode_, static_cast<std::uint8_t>(Mode::kDisabled));
        register_output("/chassis/balance/process", process_,
                        static_cast<std::uint8_t>(Process::kDisabled));
        register_output(
            "/chassis/balance/jump_phase", jump_phase_,
            static_cast<std::uint8_t>(JumpPhase::kComplete));
        register_output("/chassis/balance/inverted", inverted_, false);
        // 腿长档：操作手给档位，模式机在起身完成/落地后强制回 LOW
        //（Helios 里同一个 LL_STATE 变量被键位和 FSM 共同改写）。
        leg_length_level_effective_ = static_cast<LegLengthLevel>(0);
        register_output("/chassis/left_leg/control_length", left_control_length_, nan_);
        register_output("/chassis/right_leg/control_length", right_control_length_, nan_);
        register_output("/chassis/left_leg/control_angle", left_control_angle_, nan_);
        register_output("/chassis/right_leg/control_angle", right_control_angle_, nan_);

        read_parameters();
    }

    void before_updating() override {
        // 首拍斜坡从当前位置出发，而不是从 0 冲到目标。
        left_length_ramp_.reset(*left_length_);
        right_length_ramp_.reset(*right_length_);
        left_angle_ramp_.reset(*left_angle_total_);
        right_angle_ramp_.reset(*right_angle_total_);
        left_control_length_ramp_.reset(*left_length_);
        right_control_length_ramp_.reset(*right_length_);
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        const double dt = tick.dt_seconds();
        consume_edges();
        update_safety_counters(dt);

        if (enabled(*switch_left_)) {
            dispatch_enabled_modes(dt);
        } else {
            enter_mode(Mode::kDisabled);
            never_stood_ = true;
            fallen_elapsed_s_ = 0.0;
            healing_active_ = false;
            fatal_latched_ = false;
            *inverted_ = false;
        }

        update_phase(dt);
        update_ramps(dt);

        *mode_ = static_cast<std::uint8_t>(mode_current_);
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kPi = std::numbers::pi;

    // ── 边沿与计数器 ──────────────────────────────────────────────────────
    void consume_edges() {
        jump_request_ = *jump_count_ != last_jump_count_;
        last_jump_count_ = *jump_count_;
        rescue_request_ = *recover_count_ != last_recover_count_;
        last_recover_count_ = *recover_count_;
        if (*turn_count_ != last_turn_count_) {
            last_turn_count_ = *turn_count_;
            *inverted_ = !*inverted_;
        }
        if (*leg_length_level_ != last_leg_length_level_) {
            last_leg_length_level_ = *leg_length_level_;
            leg_length_level_effective_ = static_cast<LegLengthLevel>(*leg_length_level_);
            leg_change_lock_s_ = leg_change_lock_;
        }
    }

    /// 摔倒三判据（Helios 1488-1607，阈值以代码为准）。
    void update_safety_counters(double dt) {
        const double pitch = *body_pitch_;
        const double roll = *body_roll_;

        if (std::fabs(roll) > disaster_threshold_ || std::fabs(pitch) > disaster_threshold_)
            fatal_latched_ = true;

        if (std::fabs(roll) > fatal_roll_threshold_ || std::fabs(pitch) > fatal_pitch_threshold_)
            fatal_error_countdown_s_ += dt;
        else
            fatal_error_countdown_s_ = std::max(0.0, fatal_error_countdown_s_ - 2.0 * dt);
        if (fatal_error_countdown_s_ > fatal_error_duration_)
            fatal_latched_ = true;

        // 摔跤判据：pitch 超限（上台阶时放宽）或双腿摆角差过大，持续 0.1 s。
        const double crash_pitch_limit =
            mode_current_ == Mode::kUpStair ? crash_pitch_threshold_up_stair_
                                            : crash_pitch_threshold_;
        const double phi0_difference = *left_angle_total_ - *right_angle_total_;
        const bool crashing = (std::fabs(pitch) > crash_pitch_limit
                               && *left_grounded_ == 0 && *right_grounded_ == 0)
            || std::fabs(phi0_difference) > crash_phi0_difference_threshold_;
        crash_countdown_s_ = crashing ? crash_countdown_s_ + dt : 0.0;
    }

    static bool enabled(const hcs_msgs::Switch& value) {
        return value == hcs_msgs::Switch::MIDDLE || value == hcs_msgs::Switch::UP;
    }

    // ── 模式分发（Helios FSM 1332-1392）───────────────────────────────────
    void dispatch_enabled_modes(double dt) {
        if (fatal_latched_ && !healing_active_) {
            fallen_elapsed_s_ += dt;
            if (fallen_elapsed_s_ >= fallen_self_heal_delay_ || rescue_request_) {
                healing_active_ = true;
                enter_mode(Mode::kSelfHeal);
                never_stood_ = true;
                fallen_elapsed_s_ = 0.0;
                return;
            }
            enter_mode(Mode::kFallen);
            return;
        }
        if (healing_active_) {
            enter_mode(Mode::kSelfHeal);
            return;
        }
        if (crash_countdown_s_ > crash_duration_) {
            enter_mode(Mode::kSlowStart);
            return;
        }

        const auto requested = static_cast<Mode>(*control_mode_);
        if (never_stood_) {
            enter_mode(Mode::kSlowStart);
            return;
        }

        // 跳跃/飞坡/上台阶阶段自带粘性：完成前不回 NORMAL。
        if (mode_current_ == Mode::kJump || mode_current_ == Mode::kFly
            || mode_current_ == Mode::kUpStair) {
            return;
        }

        // 站立状态下的自动/手动入口。
        if ((mode_current_ == Mode::kNormal || mode_current_ == Mode::kSpin)
            && try_enter_jump())
            return;
        if ((mode_current_ == Mode::kNormal || mode_current_ == Mode::kSpin)
            && try_enter_up_stair(dt))
            return;
        if ((mode_current_ == Mode::kNormal || mode_current_ == Mode::kSpin)
            && try_enter_fly())
            return;

        enter_mode(requested == Mode::kSpin ? Mode::kSpin : Mode::kNormal);
    }

    // ── 模式入口 ──────────────────────────────────────────────────────────
    void enter_mode(Mode next) {
        if (next == mode_current_)
            return;
        (void)up_stair_phase_;
        // 模式切换瞬间按 Helios 语义由 process 的 DISABLED 一拍过渡（输出清零）。
        mode_current_ = next;
        phase_elapsed_s_ = 0.0;

        switch (next) {
        case Mode::kSlowStart:
            never_stood_ = true;
            slow_start_phase_ = 0; // 先收腿
            slow_start_pose_target_ = *body_pitch_ < 0.0 ? 1.23 : 1.39;
            left_angle_ramp_.reset(*left_angle_total_);
            right_angle_ramp_.reset(*right_angle_total_);
            rotate_target_left_ = *left_angle_total_;
            rotate_target_right_ = *right_angle_total_;
            slow_start_stable_s_ = 0.0;
            slow_start_elapsed_s_ = 0.0;
            process_current_ = Process::kDisabled;
            break;
        case Mode::kSelfHeal:
            never_stood_ = true;
            healing_active_ = true;
            self_heal_length_ = 0.34;
            rotate_target_left_ = *left_angle_total_;
            rotate_target_right_ = *right_angle_total_;
            left_angle_ramp_.reset(*left_angle_total_);
            right_angle_ramp_.reset(*right_angle_total_);
            heal_direction_ = (*body_pitch_ > 0.0 || *body_pitch_ < -2.8) ? -1.0 : 1.0;
            heal_stall_s_ = 0.0;
            heal_flip_cooldown_s_ = 0.0;
            self_heal_healing_ = false;
            process_current_ = Process::kPidOnly;
            break;
        case Mode::kUpStair:
            up_stair_phase_ = 1;
            left_angle_ramp_.reset(*left_angle_total_);
            right_angle_ramp_.reset(*right_angle_total_);
            process_current_ = Process::kPidOnly;
            break;
        case Mode::kJump:
            jump_phase_state_ = JumpPhase::kPress;
            jump_pressure_confirm_s_ = 0.0;
            jump_takeoff_confirm_s_ = 0.0;
            jump_touchdown_confirm_s_ = 0.0;
            jump_landing_confirm_s_ = 0.0;
            process_current_ = Process::kLqrOn;
            break;
        case Mode::kFly:
            process_current_ = Process::kLqrOn;
            break;
        case Mode::kNormal:
            never_stood_ = false;
            normal_init_lock_s_ = normal_init_lock_;
            process_current_ = Process::kLqrOn;
            break;
        case Mode::kSpin:
            process_current_ = Process::kLqrOn;
            break;
        case Mode::kFallen:
            process_current_ = Process::kDisabled;
            break;
        case Mode::kDisabled:
            process_current_ = Process::kDisabled;
            break;
        }
    }

    bool try_enter_jump() {
        if (!jump_request_)
            return false;
        if (std::fabs(*body_pitch_) >= 0.20 || std::fabs(*body_roll_) >= 0.20
            || std::fabs(*body_velocity_) >= 2.4)
            return false;
        enter_mode(Mode::kJump);
        return true;
    }

    /// 上台阶自动检测（Helios 1543-1572）：NORMAL + HIGH 档 + 双腿 phi0 折叠趋势。
    bool try_enter_up_stair(double dt) {
        if (*leg_length_level_ != static_cast<std::uint8_t>(LegLengthLevel::kHigh))
            return false;
        if (*left_angle_ < up_stair_fold_angle_ && *right_angle_ < up_stair_fold_angle_)
            up_stair_countdown_ += up_stair_count_gain_ * dt;
        else if (*left_angle_ > up_stair_unfold_angle_ && *right_angle_ > up_stair_unfold_angle_)
            up_stair_countdown_ = std::max(0.0, up_stair_countdown_ - dt);
        if (up_stair_countdown_ > up_stair_count_threshold_) {
            up_stair_countdown_ = 0.0;
            enter_mode(Mode::kUpStair);
            return true;
        }
        return false;
    }

    /// 飞坡自动检测（Helios 1433-1461）。离地检测禁用时双腿恒"着地"，
    /// 空中条件永假 → 本入口不可达（同 Helios 当前行为）。
    bool try_enter_fly() {
        if (*fly_request_)
            return false;
        if (jump_request_)
            return false;
        if (impact_lock_s_ > 0.0 || normal_init_lock_s_ > 0.0 || leg_change_lock_s_ > 0.0)
            return false;
        const bool both_airborne = *left_grounded_ == 1 && *right_grounded_ == 1;
        if (!both_airborne) {
            fly_enter_confirm_s_ = 0.0;
            return false;
        }
        fly_enter_confirm_s_ += fly_enter_hysteresis_;
        if (fly_enter_confirm_s_ < fly_enter_hysteresis_)
            return false;
        fly_enter_confirm_s_ = 0.0;
        enter_mode(Mode::kFly);
        return true;
    }

    // ── 各模式的每拍更新 ──────────────────────────────────────────────────
    void update_phase(double dt) {
        phase_elapsed_s_ += dt;
        normal_init_lock_s_ = std::max(0.0, normal_init_lock_s_ - dt);
        leg_change_lock_s_ = std::max(0.0, leg_change_lock_s_ - dt);
        impact_lock_s_ = std::max(0.0, impact_lock_s_ - dt);
        if (std::fabs(*acceleration_lateral_) > impact_acceleration_threshold_)
            impact_lock_s_ = impact_lock_time_;

        switch (mode_current_) {
        case Mode::kSlowStart: update_slow_start(dt); break;
        case Mode::kSelfHeal: update_self_heal(dt); break;
        case Mode::kUpStair: update_up_stair(dt); break;
        case Mode::kJump: update_jump(dt); break;
        case Mode::kFly: update_fly(dt); break;
        case Mode::kNormal:
        case Mode::kSpin:
            length_target_ = leg_length_base();
            rotate_target_left_ = 0.0;
            rotate_target_right_ = 0.0;
            process_current_ = Process::kLqrOn;
            break;
        case Mode::kFallen:
        case Mode::kDisabled:
            length_target_ = leg_length_base();
            process_current_ = Process::kDisabled;
            break;
        }

        *process_ = static_cast<std::uint8_t>(process_current_);
        *jump_phase_ = static_cast<std::uint8_t>(jump_phase_state_);
    }

    /// SlowStart：两阶段（先收腿到 0.24 以下，再转腿到 1.39/1.23），
    /// 姿态稳定 0.2 s 或超时 0.5 s 后转 NORMAL（Helios 1704-1782）。
    void update_slow_start(double dt) {
        process_current_ = Process::kPidOnly;
        length_target_ = 0.18;

        const bool legs_retracted = *left_length_ < 0.24 && *right_length_ < 0.24;
        if (slow_start_phase_ == 0 && legs_retracted) {
            slow_start_phase_ = 1;
            left_angle_ramp_.reset(*left_angle_total_);
            right_angle_ramp_.reset(*right_angle_total_);
            rotate_target_left_ = *left_angle_total_;
            rotate_target_right_ = *right_angle_total_;
        }
        if (slow_start_phase_ == 1) {
            // 目标角按 Helios rotate_handle：round·2π + slow_target，single > 2.40 抬 2π。
            rotate_target_left_ =
                rotate_target(*left_angle_total_, *left_angle_, slow_start_pose_target_);
            rotate_target_right_ =
                rotate_target(*right_angle_total_, *right_angle_, slow_start_pose_target_);
        } else {
            rotate_target_left_ = *left_angle_total_;
            rotate_target_right_ = *right_angle_total_;
        }

        const bool pose_stable =
            *left_angle_ < slow_start_pose_target_ + 0.2
            && *left_angle_ > slow_start_pose_target_ - 0.3
            && *right_angle_ < slow_start_pose_target_ + 0.2
            && *right_angle_ > slow_start_pose_target_ - 0.3 && legs_retracted;
        slow_start_stable_s_ = pose_stable && slow_start_phase_ == 1
            ? slow_start_stable_s_ + dt
            : 0.0;
        slow_start_elapsed_s_ += dt;

        const bool normal_init = slow_start_stable_s_ > slow_start_stable_duration_;
        const bool timeout_init = slow_start_elapsed_s_ > slow_start_timeout_
            && std::fabs(*body_pitch_) < kPi / 3.0 && pose_stable;
        if (normal_init || timeout_init) {
            slow_start_stable_s_ = 0.0;
            slow_start_elapsed_s_ = 0.0;
            enter_mode(Mode::kNormal);
            leg_length_level_effective_ = LegLengthLevel::kLow;
        }
    }

    /// 摆动目标：round·2π + slow_target，single_angle > 2.40 时目标再抬 2π
    ///（Helios rotate_handle 的 SLOW_START 分支）。
    static double rotate_target(double angle_total, double single_angle, double pose_target) {
        const double round_count = angle_total - single_angle;
        double target = round_count + pose_target;
        if (single_angle > 2.40)
            target += 2.0 * kPi;
        return target;
    }

    /// 自救：起腿 → 对齐 → 翻身（|thetab| ≥ 0.8 加转 2π，带堵转翻向）→ 收腿回位
    ///（Helios 1824-1956）。
    void update_self_heal(double dt) {
        process_current_ = self_heal_healing_ ? Process::kHealing : Process::kPidOnly;

        if (!self_heal_healing_ && *left_length_ > 0.32 && *right_length_ > 0.32) {
            self_heal_healing_ = true;
            left_angle_ramp_.reset(*left_angle_total_);
            right_angle_ramp_.reset(*right_angle_total_);
            rotate_target_left_ = *left_angle_total_;
            rotate_target_right_ = *right_angle_total_;
        }
        if (!self_heal_healing_) {
            length_target_ = self_heal_length_;
            return;
        }

        // 双腿摆角对齐：谁离铅垂近，另一条腿向它对齐。
        const double phi0_difference = wrap_angle(*left_angle_total_ - *right_angle_total_);
        if (std::fabs(phi0_difference) > 0.1) {
            length_target_ = self_heal_length_;
            if (std::fabs(*left_angle_ - kPi / 2.0) < std::fabs(*right_angle_ - kPi / 2.0))
                rotate_target_right_ = *right_angle_total_ + phi0_difference;
            else
                rotate_target_left_ = *left_angle_total_ - phi0_difference;
        } else if (std::fabs(*body_pitch_) >= 0.8) {
            // 翻身：整体加转 2π；堵转（有令不动）计 0.8 s 后翻向，冷却 1.5 s。
            length_target_ = self_heal_length_;
            rotate_target_left_ = *left_angle_total_ + heal_direction_ * 2.0 * kPi;
            rotate_target_right_ = *right_angle_total_ + heal_direction_ * 2.0 * kPi;

            const double dphi_left = std::fabs(*left_angle_total_ - phi0_total_last_left_);
            const double dphi_right = std::fabs(*right_angle_total_ - phi0_total_last_right_);
            phi0_total_last_left_ = *left_angle_total_;
            phi0_total_last_right_ = *right_angle_total_;
            heal_flip_cooldown_s_ = std::max(0.0, heal_flip_cooldown_s_ - dt);
            const bool little_motion =
                dphi_left < heal_stall_motion_threshold_ || dphi_right < heal_stall_motion_threshold_;
            if (little_motion)
                heal_stall_s_ += dt;
            else
                heal_stall_s_ = std::max(0.0, heal_stall_s_ - 2.0 * dt);
            if (heal_stall_s_ > heal_stall_flip_duration_ && heal_flip_cooldown_s_ == 0.0
                && std::fabs(*body_pitch_) > heal_flip_allow_pitch_) {
                heal_direction_ *= -1.0;
                heal_stall_s_ = 0.0;
                heal_flip_cooldown_s_ = heal_flip_cooldown_;
            }
        } else {
            // 回位：转回 π/4、收腿到 0.18，到位清 fatal 并锁 3 s。
            heal_stall_s_ = 0.0;
            rotate_target_left_ = *left_angle_total_ - wrap_angle(*left_angle_ - kPi / 4.0);
            rotate_target_right_ = *right_angle_total_ - wrap_angle(*right_angle_ - kPi / 4.0);
            length_target_ = 0.18;
            if (std::fabs(*left_angle_ - kPi / 4.0) < 0.2
                && std::fabs(*right_angle_ - kPi / 4.0) < 0.2 && *left_length_ < 0.22
                && *right_length_ < 0.22) {
                fatal_latched_ = false;
                healing_active_ = false;
                normal_init_lock_s_ = normal_init_lock_;
            }
        }
    }

    /// 上台阶：转腿到 round·2π − 0.2 并收腿，|phi0| < 0.5 持续 0.1 s → SlowStart
    ///（Helios 2001-2048）。
    void update_up_stair(double dt) {
        process_current_ = Process::kPidOnly;
        rotate_target_left_ = *left_angle_total_ - wrap_angle(*left_angle_ + 0.2);
        rotate_target_right_ = *right_angle_total_ - wrap_angle(*right_angle_ + 0.2);
        length_target_ = 0.17;
        if (std::fabs(*left_angle_) < 0.5 && std::fabs(*right_angle_) < 0.5)
            up_stair_timer_s_ += dt;
        else
            up_stair_timer_s_ = 0.0;
        if (up_stair_timer_s_ > up_stair_timer_threshold_) {
            up_stair_timer_s_ = 0.0;
            enter_mode(Mode::kSlowStart);
        }
    }

    /// 跳跃五阶段（Helios 2050-2210）。当前构建恒为大跳（level=2）。
    void update_jump(double dt) {
        switch (jump_phase_state_) {
        case JumpPhase::kPress:
            process_current_ = Process::kLqrOn;
            length_target_ = 0.20;
            if (*left_length_ < 0.20 && *right_length_ < 0.20)
                jump_pressure_confirm_s_ += dt;
            else
                jump_pressure_confirm_s_ = 0.0;
            if (jump_pressure_confirm_s_ > jump_pressure_confirm_
                || phase_elapsed_s_ > jump_pressure_timeout_) {
                jump_locked_swing_angle_ =
                    wrap_angle(0.5 * (*left_angle_ + *right_angle_));
                jump_takeoff_confirm_s_ = 0.0;
                phase_elapsed_s_ = 0.0;
                jump_phase_state_ = JumpPhase::kTakeOff;
            }
            break;
        case JumpPhase::kTakeOff: {
            process_current_ = Process::kLqrOn;
            double takeoff_length = take_off_length_high_;
            takeoff_length = std::clamp(takeoff_length + 0.02, 0.20, 0.40);
            length_target_ = takeoff_length;
            // 离地检测禁用时 grounded 恒 0，确认条件只走超时（同 Helios 现状）。
            const bool both_airborne_and_extended = *left_grounded_ == 1
                && *right_grounded_ == 1 && *left_length_ > 0.33 && *right_length_ > 0.33;
            if (both_airborne_and_extended)
                jump_takeoff_confirm_s_ += dt;
            else
                jump_takeoff_confirm_s_ = std::max(0.0, jump_takeoff_confirm_s_ - 2.0 * dt);
            const bool takeoff_ready = phase_elapsed_s_ > jump_takeoff_min_push_
                && jump_takeoff_confirm_s_ > jump_takeoff_confirm_;
            if (takeoff_ready || phase_elapsed_s_ > jump_takeoff_timeout_) {
                left_length_ramp_.reset(*left_length_);
                right_length_ramp_.reset(*right_length_);
                jump_touchdown_confirm_s_ = 0.0;
                phase_elapsed_s_ = 0.0;
                jump_phase_state_ = JumpPhase::kFlying;
            }
            break;
        }
        case JumpPhase::kFlying:
            process_current_ = Process::kPidOnly;
            length_target_ = 0.23;
            rotate_target_left_ = *left_angle_total_ - wrap_angle(*left_angle_ - jump_locked_swing_angle_);
            rotate_target_right_ = *right_angle_total_ - wrap_angle(*right_angle_ - jump_locked_swing_angle_);
            if (*left_length_ < 0.20 && *right_length_ < 0.20)
                jump_touchdown_confirm_s_ += dt;
            else
                jump_touchdown_confirm_s_ = 0.0;
            if (jump_touchdown_confirm_s_ > jump_touchdown_confirm_
                || phase_elapsed_s_ > jump_flying_timeout_) {
                jump_landing_confirm_s_ = 0.0;
                phase_elapsed_s_ = 0.0;
                jump_phase_state_ = JumpPhase::kLanding;
            }
            break;
        case JumpPhase::kLanding:
            process_current_ = Process::kLqrOn;
            length_target_ = 0.17;
            leg_length_level_effective_ = LegLengthLevel::kLow;
            if (*left_length_ < 0.25 && *right_length_ < 0.25)
                jump_landing_confirm_s_ += dt;
            else
                jump_landing_confirm_s_ = 0.0;
            if (jump_landing_confirm_s_ > jump_landing_confirm_
                || phase_elapsed_s_ > jump_landing_timeout_) {
                jump_phase_state_ = JumpPhase::kComplete;
                phase_elapsed_s_ = 0.0;
            }
            break;
        case JumpPhase::kComplete:
            enter_mode(Mode::kNormal);
            break;
        }
    }

    /// 飞坡：LL = 0.24，可靠触地（双腿 < 0.25 m 计 10 ms）或超时 1.0 s 退出
    ///（Helios 1959-1998 + 1468-1477；LL 值以代码为准）。
    void update_fly(double dt) {
        process_current_ = Process::kLqrOn;
        length_target_ = 0.24;
        const bool both_short = *left_length_ < 0.25 && *right_length_ < 0.25;
        fly_touchdown_confirm_s_ = both_short && phase_elapsed_s_ > fly_minimum_duration_
            ? fly_touchdown_confirm_s_ + dt
            : 0.0;
        if (fly_touchdown_confirm_s_ > fly_touchdown_confirm_
            || phase_elapsed_s_ > fly_timeout_) {
            fly_touchdown_confirm_s_ = 0.0;
            enter_mode(Mode::kNormal);
        }
    }

    // ── 斜坡输出 ──────────────────────────────────────────────────────────
    /// 各模式的斜坡速率表（Helios L0_speed / rotate_speed，已换算成秒制）。
    void update_ramps(double dt) {
        double length_rate = 0.2;
        double rotate_rate = 0.0;
        switch (mode_current_) {
        case Mode::kSlowStart: length_rate = 0.5; rotate_rate = 7.0; break;
        case Mode::kJump: length_rate = 3.0; rotate_rate = 3.0; break;
        case Mode::kSelfHeal: length_rate = 0.4; rotate_rate = 3.0; break;
        case Mode::kFly: length_rate = 1.0; rotate_rate = 6.0; break;
        case Mode::kUpStair: length_rate = 0.4; rotate_rate = 5.5; break;
        case Mode::kNormal:
        case Mode::kSpin:
        case Mode::kFallen:
        case Mode::kDisabled: length_rate = 0.2; rotate_rate = 0.0; break;
        }

        *left_control_length_ = left_length_ramp_.update(length_target_, length_rate, dt);
        *right_control_length_ = right_length_ramp_.update(length_target_, length_rate, dt);
        *left_control_angle_ = left_angle_ramp_.update(rotate_target_left_, rotate_rate, dt);
        *right_control_angle_ = right_angle_ramp_.update(rotate_target_right_, rotate_rate, dt);
    }

    [[nodiscard]] double leg_length_base() const {
        switch (leg_length_level_effective_) {
        case LegLengthLevel::kMid: return leg_length_mid_;
        case LegLengthLevel::kHigh: return leg_length_high_;
        case LegLengthLevel::kLow: break;
        }
        return leg_length_low_;
    }

    void read_parameters() {
        disaster_threshold_ = get_parameter("disaster_pitch_roll_threshold").as_double();
        fatal_roll_threshold_ = get_parameter("fatal_roll_threshold").as_double();
        fatal_pitch_threshold_ = get_parameter("fatal_pitch_threshold").as_double();
        fatal_error_duration_ = get_parameter("fatal_error_duration").as_double();
        fallen_self_heal_delay_ = get_parameter("fallen_self_heal_delay").as_double();
        crash_pitch_threshold_ = get_parameter("crash_pitch_threshold").as_double();
        crash_pitch_threshold_up_stair_ =
            get_parameter("crash_pitch_threshold_up_stair").as_double();
        crash_phi0_difference_threshold_ =
            get_parameter("crash_phi0_difference_threshold").as_double();
        crash_duration_ = get_parameter("crash_duration").as_double();
        normal_init_lock_ = get_parameter("normal_init_lock").as_double();
        leg_change_lock_ = get_parameter("leg_change_lock").as_double();
        slow_start_stable_duration_ = get_parameter("slow_start_stable_duration").as_double();
        slow_start_timeout_ = get_parameter("slow_start_timeout").as_double();
        jump_pressure_confirm_ = get_parameter("jump_pressure_confirm").as_double();
        jump_pressure_timeout_ = get_parameter("jump_pressure_timeout").as_double();
        jump_takeoff_confirm_ = get_parameter("jump_takeoff_confirm").as_double();
        jump_takeoff_min_push_ = get_parameter("jump_takeoff_min_push").as_double();
        jump_takeoff_timeout_ = get_parameter("jump_takeoff_timeout").as_double();
        jump_touchdown_confirm_ = get_parameter("jump_touchdown_confirm").as_double();
        jump_flying_timeout_ = get_parameter("jump_flying_timeout").as_double();
        jump_landing_confirm_ = get_parameter("jump_landing_confirm").as_double();
        jump_landing_timeout_ = get_parameter("jump_landing_timeout").as_double();
        take_off_length_high_ = get_parameter("take_off_length_high").as_double();
        fly_enter_hysteresis_ = get_parameter("fly_enter_confirm").as_double();
        fly_touchdown_confirm_ = get_parameter("fly_touchdown_confirm").as_double();
        fly_minimum_duration_ = get_parameter("fly_minimum_duration").as_double();
        fly_timeout_ = get_parameter("fly_timeout").as_double();
        impact_acceleration_threshold_ =
            get_parameter("impact_acceleration_threshold").as_double();
        impact_lock_time_ = get_parameter("impact_lock_time").as_double();
        up_stair_fold_angle_ = get_parameter("up_stair_fold_angle").as_double();
        up_stair_unfold_angle_ = get_parameter("up_stair_unfold_angle").as_double();
        up_stair_count_gain_ = get_parameter("up_stair_count_gain").as_double();
        up_stair_count_threshold_ = get_parameter("up_stair_count_threshold").as_double();
        up_stair_timer_threshold_ = get_parameter("up_stair_timer_threshold").as_double();
        heal_stall_flip_duration_ = get_parameter("heal_stall_flip_duration").as_double();
        heal_flip_cooldown_ = get_parameter("heal_flip_cooldown").as_double();
        heal_stall_motion_threshold_ = get_parameter("heal_stall_motion_threshold").as_double();
        heal_flip_allow_pitch_ = get_parameter("heal_flip_allow_pitch").as_double();
        leg_length_low_ = get_parameter("leg_length_low").as_double();
        leg_length_mid_ = get_parameter("leg_length_mid").as_double();
        leg_length_high_ = get_parameter("leg_length_high").as_double();
    }

    // ── 输入 ─────────────────────────────────────────────────────────────
    InputInterface<hcs_msgs::Switch> switch_left_;
    InputInterface<std::uint8_t> control_mode_;
    InputInterface<std::size_t> jump_count_;
    InputInterface<std::size_t> recover_count_;
    InputInterface<std::size_t> turn_count_;
    InputInterface<bool> fly_request_;
    InputInterface<std::uint8_t> leg_length_level_;
    InputInterface<double> body_pitch_;
    InputInterface<double> body_roll_;
    InputInterface<double> body_velocity_;
    InputInterface<double> left_length_;
    InputInterface<double> right_length_;
    InputInterface<double> left_angle_;
    InputInterface<double> right_angle_;
    InputInterface<double> left_angle_total_;
    InputInterface<double> right_angle_total_;
    InputInterface<std::uint8_t> left_grounded_;
    InputInterface<std::uint8_t> right_grounded_;
    InputInterface<double> acceleration_forward_;
    InputInterface<double> acceleration_lateral_;

    // ── 输出 ─────────────────────────────────────────────────────────────
    OutputInterface<std::uint8_t> mode_;
    OutputInterface<std::uint8_t> process_;
    OutputInterface<std::uint8_t> jump_phase_;
    OutputInterface<bool> inverted_;
    OutputInterface<double> left_control_length_;
    OutputInterface<double> right_control_length_;
    OutputInterface<double> left_control_angle_;
    OutputInterface<double> right_control_angle_;

    // ── 状态机状态 ───────────────────────────────────────────────────────
    Mode mode_current_ = Mode::kDisabled;
    Process process_current_ = Process::kDisabled;
    JumpPhase jump_phase_state_ = JumpPhase::kComplete;

    bool never_stood_ = true;
    bool fatal_latched_ = false;
    bool healing_active_ = false;
    bool self_heal_healing_ = false;

    double fatal_error_countdown_s_ = 0.0;
    double crash_countdown_s_ = 0.0;
    double fallen_elapsed_s_ = 0.0;
    double normal_init_lock_s_ = 0.0;
    double leg_change_lock_s_ = 0.0;
    double impact_lock_s_ = 0.0;
    double fly_enter_confirm_s_ = 0.0;
    double fly_touchdown_confirm_s_ = 0.0;
    double phase_elapsed_s_ = 0.0;
    double up_stair_countdown_ = 0.0;
    double up_stair_timer_s_ = 0.0;
    double heal_stall_s_ = 0.0;
    double heal_flip_cooldown_s_ = 0.0;
    double phi0_total_last_left_ = 0.0;
    double phi0_total_last_right_ = 0.0;

    int up_stair_phase_ = 1;
    int slow_start_phase_ = 0;
    double slow_start_pose_target_ = 1.39;
    double slow_start_stable_s_ = 0.0;
    double slow_start_elapsed_s_ = 0.0;
    double heal_direction_ = 1.0;
    double self_heal_length_ = 0.34;
    double jump_locked_swing_angle_ = 0.0;
    double jump_pressure_confirm_s_ = 0.0;
    double jump_takeoff_confirm_s_ = 0.0;
    double jump_touchdown_confirm_s_ = 0.0;
    double jump_landing_confirm_s_ = 0.0;

    double length_target_ = 0.17;
    double rotate_target_left_ = 0.0;
    double rotate_target_right_ = 0.0;
    Ramp left_length_ramp_;
    Ramp right_length_ramp_;
    Ramp left_angle_ramp_;
    Ramp right_angle_ramp_;
    Ramp left_control_length_ramp_;
    Ramp right_control_length_ramp_;

    std::size_t last_jump_count_ = 0;
    std::size_t last_recover_count_ = 0;
    std::size_t last_turn_count_ = 0;
    std::uint8_t last_leg_length_level_ = 0;
    LegLengthLevel leg_length_level_effective_ = LegLengthLevel::kLow;
    bool jump_request_ = false;
    bool rescue_request_ = false;

    // ── 参数（阈值与 Helios 一致，单位换算成秒/秒制）─────────────────────
    double disaster_threshold_ = 1.2;
    double fatal_roll_threshold_ = 0.90;
    double fatal_pitch_threshold_ = 0.82;
    double fatal_error_duration_ = 0.05;
    double fallen_self_heal_delay_ = 2.0;
    double crash_pitch_threshold_ = 0.45;
    double crash_pitch_threshold_up_stair_ = 0.65;
    double crash_phi0_difference_threshold_ = 0.69;
    double crash_duration_ = 0.10;
    double normal_init_lock_ = 3.0;
    double leg_change_lock_ = 0.5;
    double slow_start_stable_duration_ = 0.2;
    double slow_start_timeout_ = 0.5;
    double jump_pressure_confirm_ = 0.08;
    double jump_pressure_timeout_ = 0.45;
    double jump_takeoff_confirm_ = 0.012;
    double jump_takeoff_min_push_ = 0.018;
    double jump_takeoff_timeout_ = 0.18;
    double jump_touchdown_confirm_ = 0.01;
    double jump_flying_timeout_ = 1.2;
    double jump_landing_confirm_ = 0.05;
    double jump_landing_timeout_ = 0.7;
    double take_off_length_high_ = 0.35;
    double fly_enter_hysteresis_ = 0.003;
    double fly_touchdown_confirm_ = 0.01;
    double fly_minimum_duration_ = 0.1;
    double fly_timeout_ = 1.0;
    double impact_acceleration_threshold_ = 6.3;
    double impact_lock_time_ = 0.1;
    double up_stair_fold_angle_ = 1.23;
    double up_stair_unfold_angle_ = 1.32;
    double up_stair_count_gain_ = 3.0;
    double up_stair_count_threshold_ = 0.08;
    double up_stair_timer_threshold_ = 0.1;
    double heal_stall_flip_duration_ = 0.8;
    double heal_flip_cooldown_ = 1.5;
    double heal_stall_motion_threshold_ = 0.003;
    double heal_flip_allow_pitch_ = 1.05;
    double leg_length_low_ = 0.17;
    double leg_length_mid_ = 0.25;
    double leg_length_high_ = 0.36;
};

} // namespace hcs_core::controller::chassis::balance

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    hcs_core::controller::chassis::balance::BalanceModeManager, hcs_executor::Component)
