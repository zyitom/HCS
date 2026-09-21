#include "phase_machine.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace hcs_core::controller::balance {

namespace {

PhaseKind kind_of_impl(const Phase& phase) {
    return std::visit(
        [](const auto& p) -> PhaseKind {
            using T = std::decay_t<decltype(p)>;
            if constexpr (std::is_same_v<T, phase::Disabled>)
                return PhaseKind::Disabled;
            else if constexpr (std::is_same_v<T, phase::SlowStart>)
                return PhaseKind::SlowStart;
            else if constexpr (std::is_same_v<T, phase::Standing>)
                return PhaseKind::Standing;
            else if constexpr (std::is_same_v<T, phase::UpStair>)
                return PhaseKind::UpStair;
            else if constexpr (std::is_same_v<T, phase::Jump>)
                return PhaseKind::Jump;
            else if constexpr (std::is_same_v<T, phase::Fly>)
                return PhaseKind::Fly;
            else if constexpr (std::is_same_v<T, phase::Fallen>)
                return PhaseKind::Fallen;
            else
                return PhaseKind::SelfHeal;
        },
        phase);
}

} // namespace

ProcessFlag controller_for(const Phase& phase) {
    return std::visit(
        [](const auto& p) -> ProcessFlag {
            using T = std::decay_t<decltype(p)>;
            if constexpr (std::is_same_v<T, phase::Disabled> || std::is_same_v<T, phase::Fallen>)
                return ProcessFlag::Disabled;
            else if constexpr (std::is_same_v<T, phase::SlowStart>)
                return ProcessFlag::PidOnly;
            else if constexpr (std::is_same_v<T, phase::Standing>)
                return ProcessFlag::LqrOn;
            else if constexpr (std::is_same_v<T, phase::UpStair>)
                return ProcessFlag::PidOnly;
            else if constexpr (std::is_same_v<T, phase::Jump>) {
                return p.step == phase::Jump::Step::Flying ? ProcessFlag::PidOnly
                                                           : ProcessFlag::LqrOn;
            } else if constexpr (std::is_same_v<T, phase::Fly>)
                return ProcessFlag::LqrOn;
            else // SelfHeal
                return p.healing ? ProcessFlag::Healing : ProcessFlag::PidOnly;
        },
        phase);
}

PhaseKind kind_of(const Phase& phase) {
    return kind_of_impl(phase);
}

float PhaseMachine::rotate_handle(std::size_t leg, float round_count, float single_angle) {
    // v2:1184-1230。SLOW_START 分支的 slow_target 已并入 phase::SlowStart::pose_target；
    // SELF_HEAL 分支两个字面分支相同，按原样保留。
    if (std::holds_alternative<phase::SlowStart>(current_)) {
        const auto& slow = std::get<phase::SlowStart>(current_);
        float target_angle = round_count * 2 * kPi + slow.pose_target;
        float current_angle = round_count * 2 * kPi + single_angle;
        float diff = target_angle - current_angle;

        if (single_angle > 2.40f) {
            target_angle += 2 * kPi;
        } else {
            while (diff > kPi) {
                target_angle -= 2 * kPi;
                diff -= 2 * kPi;
            }
            while (diff < -kPi) {
                target_angle += 2 * kPi;
                diff += 2 * kPi;
            }
        }
        (void)leg;
        return target_angle;
    }
    if (std::holds_alternative<phase::SelfHeal>(current_)) {
        const auto& wbc = estimator_.estimate().wbc;
        (void)wbc;
        return (round_count + 1) * 2 * kPi + single_angle;
    }
    return 0.0f;
}

bool PhaseMachine::pose_stable_for_jump() const {
    const auto& wbc = estimator_.estimate().wbc;
    return (std::fabs(wbc.thetab) < 0.20f) && (std::fabs(wbc.roll) < 0.20f)
        && (std::fabs(wbc.s1) < 2.4f);
}

MachineView PhaseMachine::update(const OperatorCommand& command, float dt) {
    const auto& est = estimator_.estimate();
    const auto& wbc = est.wbc;
    const auto& leg = est.leg;
    const auto& ground = est.ground.if_off_gnd;
    const auto& p = params_;

    spin_active_past_ = spin_active_;
    jumping_ = std::holds_alternative<phase::Jump>(current_);
    process_flag_past_ = process_flag_;
    past_ = current_;

    // ── 输入 TD 与档位（v2:1289-1312） ─────────────────────────────────────
    const float v_want = clip(command.speed_level[0] * p.speed_command_gain, -p.speed_command_clip, p.speed_command_clip);
    const bool is_flipped = command.if_turn ^ is_inversed_;
    const float direction_sign = is_flipped ? -1.0f : 1.0f;
    v_raw_target_ = v_want * direction_sign;
    v_target_ = td_input_.update(v_want * direction_sign, {p.td_r_input, dt, p.td_h0_input});
    acc_fwd_ = td_input_.get_derivative() * 0.1f;

    dial_level_ = 0.0f;
    if (command.speed_level[2] == 1)
        dial_level_ = p.dial_level_low;
    else if (command.speed_level[2] == 2)
        dial_level_ = p.dial_level_high;
    else if (command.speed_level[2] == -1)
        dial_level_ = -p.dial_level_low;
    else if (command.speed_level[2] == -2)
        dial_level_ = -p.dial_level_high;

    const int jump_cmd = command.jump_rising_edge ? 2 : 0;

    // ── 入口分发（v2:1333-1376；Disabled 双义已拆分） ──────────────────────
    if (!command.if_enable) {
        current_ = phase::Disabled{};
    } else if (std::holds_alternative<phase::Fallen>(current_)) {
        auto& fallen = std::get<phase::Fallen>(current_);
        fallen.elapsed_s += dt;
        if (fallen.elapsed_s >= p.fallen_wait_s || command.save_changed || command.save_level) {
            current_ = phase::SelfHeal{};
        }
    } else if (std::holds_alternative<phase::SelfHeal>(current_)
               && !std::get<phase::SelfHeal>(current_).done) {
        std::get<phase::SelfHeal>(current_).elapsed_s = 0.0f;
    } else if (std::holds_alternative<phase::UpStair>(current_)
               || std::holds_alternative<phase::Jump>(current_)
               || std::holds_alternative<phase::Fly>(current_)) {
        // v2:1363-1369 —— PAST 保持语义：空中/上台阶/跳跃阶段不被入口分发拉回。
    } else if (std::holds_alternative<phase::Standing>(current_)) {
        if (command.if_free)
            spin_active_ = true;
        else
            spin_active_ = false;
    } else {
        current_ = phase::SlowStart{}; // v2:1357-1360 —— enable 且未站立 → SLOW_START
        spin_active_ = false;
    }

    // ── 腿长档位切换（v2:1378-1386） ───────────────────────────────────────
    if (command.leg_changed) {
        ll_state_ = (ll_state_ + 1) % 3;
        leg_change_lock_s_ = p.leg_change_lock_s;
    }

    // ── 安全检查：冲击锁存（v2:1397-1408） ─────────────────────────────────
    safety_.update_impact_lock(wbc.a_x, dt);

    // ── FLY 触地通道与进出（v2:1415-1477） ─────────────────────────────────
    safety_.update_touch_down(est, dt);
    const auto safety_view = safety_.view();

    const bool both_leg_off_ground = ground[kLeft] == 1 && ground[kRight] == 1;
    const bool standing_now_nonspin =
        std::holds_alternative<phase::Standing>(current_) && !spin_active_;
    const bool past_standing_nonspin =
        std::holds_alternative<phase::Standing>(past_) && !spin_active_past_;

    const bool can_enter_fly = past_standing_nonspin && jump_cmd == 0
                             && !safety_view.impact_lock_active && !standing_lock_active_;
    if (can_enter_fly && both_leg_off_ground) {
        fly_enter_accum_ += dt;
    } else {
        fly_enter_accum_ = 0.0f;
    }
    if (fly_enter_accum_ > p.fly_enter_hysteresis_s) {
        auto& fly = current_.emplace<phase::Fly>();
        float swing = 0.5f * (leg[kLeft].phi0 + leg[kRight].phi0);
        while (swing > kPi) swing -= 2.0f * kPi;
        while (swing < -kPi) swing += 2.0f * kPi;
        fly.locked_swing = swing;
        fly_enter_accum_ = 0.0f;
        estimator_.wbc().s = 0.0f;
        estimator_.wbc().s1 = 0.0f;
        estimator_.reset_kf();
    }

    if (std::holds_alternative<phase::Fly>(current_)) {
        auto& fly = std::get<phase::Fly>(current_);
        fly.elapsed_s += dt;
        if ((std::holds_alternative<phase::Fly>(past_)
             || std::holds_alternative<phase::Fly>(current_))
            && (safety_view.reliable_touch_down || fly.elapsed_s > p.fly_timeout_s)
            && fly.elapsed_s > p.fly_min_s) {
            current_ = phase::Standing{};
            ll_state_ = 0;
        }
    }

    // ── JUMP 进入（v2:1479-1487） ──────────────────────────────────────────
    if (jump_cmd != 0 && past_standing_nonspin && pose_stable_for_jump()) {
        auto& jump = current_.emplace<phase::Jump>();
        jump.level = (jump_cmd == 2) ? 2 : 1;
    }

    // ── 安全检查：灾难/累积翻车/摔跤（v2:1494-1531、1574-1607） ─────────────
    safety_.update_fatal(
        est, kind_of(current_), {leg[kLeft].phi0_total, leg[kRight].phi0_total},
        kind_of(current_) == PhaseKind::Standing || kind_of(current_) == PhaseKind::UpStair
            || kind_of(current_) == PhaseKind::Jump || kind_of(current_) == PhaseKind::Fly,
        dt);
    const auto fatal_view = safety_.view();
    // v2 的 is_fatal_error 置位后不会被重复的灾难检测复位计时（fatal_error_timer
    // 只在 if_enable==0 时清零）——已处于 Fallen 时不重建阶段，避免计时被清。
    if ((fatal_view.disaster || fatal_view.fatal_accumulated)
        && !std::holds_alternative<phase::Fallen>(current_)) {
        current_ = phase::Fallen{};
        spin_active_ = false;
    }
    if (fatal_view.crash) {
        current_ = phase::SlowStart{};
        spin_active_ = false;
    }

    // ── 小陀螺进入（v2:1533-1541） ─────────────────────────────────────────
    const bool spin_cmd = (dial_level_ != 0.0f) || command.if_free;
    if (spin_cmd && standing_now_nonspin) {
        spin_active_ = true;
    }

    // ── 上台阶自动检测（v2:1543-1572；用小陀螺进入之后的 PRESENT 语义） ────
    const bool standing_now_nonspin_post =
        std::holds_alternative<phase::Standing>(current_) && !spin_active_;
    if (standing_now_nonspin_post) {
        if (ll_state_ == 2) {
            if (leg[kLeft].phi0 < p.up_stair_phi_fold && leg[kRight].phi0 < p.up_stair_phi_fold) {
                up_stair_cnt_ += p.up_stair_enter_bonus;
            } else if (
                leg[kLeft].phi0 < p.up_stair_phi_fold_release
                && leg[kRight].phi0 < p.up_stair_phi_fold_release) {
                up_stair_cnt_ -= p.up_stair_enter_decay;
            } else {
                up_stair_cnt_ = 0.0f;
            }
        } else {
            up_stair_cnt_ = 0.0f;
        }
        if (up_stair_cnt_ > p.up_stair_enter_cap)
            up_stair_cnt_ = p.up_stair_enter_cap;
        if (up_stair_cnt_ > p.up_stair_enter_count) {
            current_ = phase::UpStair{};
            up_stair_cnt_ = 0.0f;
            spin_active_ = false;
        }
    }

    // ── 小陀螺退出清理（v2:1609-1617） ─────────────────────────────────────
    if (spin_active_past_
        && !(std::holds_alternative<phase::Standing>(current_) && spin_active_)) {
        follow_angle_ = wbc.phi;
        td_spin_.reset();
        estimator_.wbc().s = 0.0f;
        estimator_.wbc().s1 = 0.0f;
        estimator_.reset_kf();
    }

    // ── 小陀螺运转（v2:1619-1655） ─────────────────────────────────────────
    if (std::holds_alternative<phase::Standing>(current_) && spin_active_) {
        v_yaw_ = td_spin_.update(dial_level_, {p.td_r_input * p.td_r_spin_scale, dt, p.td_h0_input});
        if (ll_state_ != spin_last_ll_state_) {
            ll_want_target_[kLeft] = p.spin_ll_want[static_cast<std::size_t>(ll_state_)];
            ll_want_target_[kRight] = p.spin_ll_want[static_cast<std::size_t>(ll_state_)];
        }
        spin_last_ll_state_ = ll_state_;
    }

    // ── 阶段变化的一拍 DISABLED 与位移清零（v2:1658-1671） ──────────────────
    const bool phase_changed = current_.index() != past_.index();
    if (phase_changed) {
        process_flag_ = ProcessFlag::Disabled;
        if (kind_of(past_) == PhaseKind::Disabled
            && (kind_of(current_) == PhaseKind::SlowStart
                || kind_of(current_) == PhaseKind::Standing)) {
            estimator_.wbc().s = 0.0f;
        }
    }

    // ── Disabled 例行（v2:1673-1690；v2 的 DISABLED 双义 → Disabled 与 Fallen
    //    都要执行：翻车等待期间斜坡同样被复位到当前位形） ─────────────────────
    if (std::holds_alternative<phase::Disabled>(current_)
        || std::holds_alternative<phase::Fallen>(current_)) {
        v_target_ = 0.0f;
        v_yaw_ = 0.0f;
        follow_angle_ = wbc.phi;
        ramp_leg_len_[kLeft].reset(leg[kLeft].L0);
        ramp_leg_len_[kRight].reset(leg[kRight].L0);
        ramp_leg_rotate_[kLeft].reset(leg[kLeft].phi0);
        ramp_leg_rotate_[kRight].reset(leg[kRight].phi0);
        is_inversed_ = false;
        rotate_speed_ = p.rotate_speed_disabled;
        l0_speed_ = p.l0_speed_tank;
    }

    // ── SLOW_START（v2:1704-1782） ─────────────────────────────────────────
    if (auto* slow = std::get_if<phase::SlowStart>(&current_)) {
        if (!std::holds_alternative<phase::SlowStart>(past_)) {
            ramp_leg_rotate_[kLeft].reset(leg[kLeft].phi0_total);
            ramp_leg_rotate_[kRight].reset(leg[kRight].phi0_total);
            slow->pose_target = (wbc.thetab < 0.0f) ? 1.23f : 1.39f;
            slow->step = phase::SlowStart::Step::Retract;
            rotate_want_target_[kLeft] = leg[kLeft].phi0_total;
            rotate_want_target_[kRight] = leg[kRight].phi0_total;
        }

        ll_want_target_[kLeft] = p.slow_start_ll_want;
        ll_want_target_[kRight] = p.slow_start_ll_want;

        const bool legs_retracted =
            leg[kRight].L0 < p.slow_start_retract_thres && leg[kLeft].L0 < p.slow_start_retract_thres;

        if (slow->step == phase::SlowStart::Step::Retract && legs_retracted) {
            slow->step = phase::SlowStart::Step::Rotate;
            ramp_leg_rotate_[kLeft].reset(leg[kLeft].phi0_total);
            ramp_leg_rotate_[kRight].reset(leg[kRight].phi0_total);
            rotate_want_target_[kLeft] =
                rotate_handle(kLeft, leg[kLeft].round_count, leg[kLeft].phi0);
            rotate_want_target_[kRight] =
                rotate_handle(kRight, leg[kRight].round_count, leg[kRight].phi0);
        }

        const bool pose_stable =
            (leg[kLeft].phi0 < slow->pose_target + p.slow_start_pose_window_up
             && leg[kLeft].phi0 > slow->pose_target - p.slow_start_pose_window_down
             && leg[kRight].phi0 < slow->pose_target + p.slow_start_pose_window_up
             && leg[kRight].phi0 > slow->pose_target - p.slow_start_pose_window_down
             && legs_retracted);

        if (pose_stable && slow->step == phase::SlowStart::Step::Rotate) {
            slow->stable_s += dt;
        } else if (!pose_stable) {
            slow->stable_s = 0.0f;
        }

        slow->elapsed_s += dt;

        const bool normal_init = slow->stable_s > p.slow_start_stable_s;
        const bool timeout_init = slow->elapsed_s > p.slow_start_timeout_s
                               && std::fabs(wbc.thetab) < kPi / 3 && pose_stable;
        if (normal_init || timeout_init) {
            current_ = phase::Standing{};
            ll_state_ = 0;
            const float yaw_off = rad_format(command.follow_angle_cmd);
            is_inversed_ = (std::fabs(yaw_off) > kPi / 2.0f);
            follow_angle_ = wbc.phi;
            estimator_.wbc().s = 0.0f;
            estimator_.wbc().s1 = 0.0f;
            estimator_.reset_kf();
        }
    }

    // ── 站立（NORMAL/SPIN 共用，v2:1785-1821） ─────────────────────────────
    if (std::holds_alternative<phase::Standing>(current_)) {
        const bool enter_stand_mode = !std::holds_alternative<phase::Standing>(past_);
        const bool ll_state_changed = ll_state_ != stand_last_ll_state_;
        if (enter_stand_mode || ll_state_changed) {
            ll_want_target_[kLeft] = p.stand_ll_want[static_cast<std::size_t>(ll_state_)];
            ll_want_target_[kRight] = p.stand_ll_want[static_cast<std::size_t>(ll_state_)];
        }
        stand_last_ll_state_ = ll_state_;
        if (auto* standing = std::get_if<phase::Standing>(&current_))
            standing->elapsed_s += dt;
    }

    // ── SELF_HEAL（v2:1824-1956） ──────────────────────────────────────────
    if (auto* heal = std::get_if<phase::SelfHeal>(&current_)) {
        if (!std::holds_alternative<phase::SelfHeal>(past_)) {
            ll_want_target_[kLeft] = p.heal_ll_want;
            ll_want_target_[kRight] = p.heal_ll_want;
            rotate_want_target_[kLeft] = leg[kLeft].phi0_total;
            rotate_want_target_[kRight] = leg[kRight].phi0_total;
            heal->phi0_total_last[kLeft] = leg[kLeft].phi0_total;
            heal->phi0_total_last[kRight] = leg[kRight].phi0_total;
            ramp_leg_rotate_[kLeft].reset(leg[kLeft].phi0_total);
            ramp_leg_rotate_[kRight].reset(leg[kRight].phi0_total);
            heal->stall_s = 0.0f;
            heal->cooldown_s = 0.0f;
            heal->done = false;
            heal->healing = false;
            heal->dir = (wbc.thetab > 0 || wbc.thetab < -2.8f) ? -1 : 1;
        }

        if (command.save_changed && heal->healing) {
            heal->dir *= -1;
        }

        if (leg[kLeft].L0 > p.heal_extend_thres && leg[kRight].L0 > p.heal_extend_thres) {
            heal->healing = true;
        }

        if (heal->healing) {
            if (!self_heal_healing_past_) {
                ramp_leg_rotate_[kLeft].reset(leg[kLeft].phi0_total);
                ramp_leg_rotate_[kRight].reset(leg[kRight].phi0_total);
                rotate_want_target_[kLeft] = leg[kLeft].phi0_total;
                rotate_want_target_[kRight] = leg[kRight].phi0_total;
            }

            float phi0_err = leg[kLeft].phi0_total - leg[kRight].phi0_total;
            while (phi0_err > kPi) phi0_err -= 2.0f * kPi;
            while (phi0_err < -kPi) phi0_err += 2.0f * kPi;

            if (std::fabs(phi0_err) > 0.1f) {
                if (std::fabs(leg[kLeft].phi0 - kPi / 2) < std::fabs(leg[kRight].phi0 - kPi / 2)) {
                    rotate_want_target_[kRight] =
                        leg[kRight].round_count * 2 * kPi + leg[kLeft].phi0;
                } else {
                    rotate_want_target_[kLeft] =
                        leg[kLeft].round_count * 2 * kPi + leg[kRight].phi0;
                }
            } else {
                if (std::fabs(wbc.thetab) >= 0.8f) {
                    rotate_want_target_[kLeft] = leg[kLeft].phi0_total + heal->dir * 2 * kPi;
                    rotate_want_target_[kRight] = leg[kRight].phi0_total + heal->dir * 2 * kPi;
                    ll_want_target_[kLeft] = p.heal_ll_want;
                    ll_want_target_[kRight] = p.heal_ll_want;

                    const float dphi_total_l =
                        std::fabs(leg[kLeft].phi0_total - heal->phi0_total_last[kLeft]);
                    const float dphi_total_r =
                        std::fabs(leg[kRight].phi0_total - heal->phi0_total_last[kRight]);
                    heal->phi0_total_last[kLeft] = leg[kLeft].phi0_total;
                    heal->phi0_total_last[kRight] = leg[kRight].phi0_total;

                    const bool high_output_cmd = true; // v2:1910，原样保留
                    const bool little_motion =
                        (dphi_total_l < p.heal_stall_motion_thres)
                        || (dphi_total_r < p.heal_stall_motion_thres);

                    if (heal->cooldown_s > 0.0f)
                        heal->cooldown_s -= dt;

                    if (high_output_cmd && little_motion) {
                        heal->stall_s += dt;
                    } else if (heal->stall_s > 0.0f) {
                        heal->stall_s -= 2.0f * dt;
                    }
                    if (heal->stall_s < 0.0f)
                        heal->stall_s = 0.0f;

                    if (heal->stall_s > p.heal_stall_s && heal->cooldown_s <= 0.0f
                        && std::fabs(wbc.thetab) > p.heal_flip_allow_thetab) {
                        heal->dir *= -1;
                        heal->stall_s = 0.0f;
                        heal->cooldown_s = p.heal_flip_cooldown_s;
                        rotate_want_target_[kLeft] = leg[kLeft].phi0_total + heal->dir * 2 * kPi;
                        rotate_want_target_[kRight] = leg[kRight].phi0_total + heal->dir * 2 * kPi;
                    }
                } else {
                    heal->stall_s = 0.0f;
                    rotate_want_target_[kLeft] =
                        leg[kLeft].round_count * 2 * kPi + p.heal_phi_target;
                    rotate_want_target_[kRight] =
                        leg[kRight].round_count * 2 * kPi + p.heal_phi_target;
                    ll_want_target_[kLeft] = p.heal_ll_short;
                    ll_want_target_[kRight] = p.heal_ll_short;

                    if (std::fabs(leg[kLeft].phi0 - p.heal_phi_target) < p.heal_phi_window
                        && std::fabs(leg[kRight].phi0 - p.heal_phi_target) < p.heal_phi_window
                        && leg[kLeft].L0 < p.heal_ll_done && leg[kRight].L0 < p.heal_ll_done) {
                        heal->done = true;
                        estimator_.reset_kf();
                    }
                }
            }
        }
    }
    self_heal_healing_past_ =
        std::holds_alternative<phase::SelfHeal>(current_)
        && std::get<phase::SelfHeal>(current_).healing;

    // ── FLY 目标（v2:1959-1965） ───────────────────────────────────────────
    if (auto* fly = std::get_if<phase::Fly>(&current_)) {
        ll_state_ = 0;
        ll_want_target_[kLeft] = p.fly_ll_want;
        ll_want_target_[kRight] = p.fly_ll_want;
        (void)fly;
    }

    // ── UP_STAIR（v2:2001-2048） ───────────────────────────────────────────
    if (auto* up_stair = std::get_if<phase::UpStair>(&current_)) {
        if (!std::holds_alternative<phase::UpStair>(past_)) {
            up_stair->stable_s = 0.0f;
            ramp_leg_rotate_[kLeft].reset(leg[kLeft].phi0_total);
            ramp_leg_rotate_[kRight].reset(leg[kRight].phi0_total);
        }

        rotate_want_target_[kLeft] = leg[kLeft].round_count * 2 * kPi - 0.2f;
        rotate_want_target_[kRight] = leg[kRight].round_count * 2 * kPi - 0.2f;
        ll_want_target_[kLeft] = p.slow_start_ll_want;
        ll_want_target_[kRight] = p.slow_start_ll_want;

        if (std::fabs(leg[kLeft].phi0) < 0.5f && std::fabs(leg[kRight].phi0) < 0.5f) {
            up_stair->stable_s += dt;
        } else {
            up_stair->stable_s = 0.0f;
        }

        if (up_stair->stable_s > p.up_stair_stable_s) {
            auto& slow = current_.emplace<phase::SlowStart>();
            slow.pose_target = (wbc.thetab < 0.0f) ? 1.23f : 1.39f;
            ll_state_ = 0;
            flag_disabled_override_ = true;
            ramp_leg_rotate_[kLeft].reset(leg[kLeft].phi0_total);
            ramp_leg_rotate_[kRight].reset(leg[kRight].phi0_total);
            rotate_want_target_[kLeft] =
                rotate_handle(kLeft, leg[kLeft].round_count, leg[kLeft].phi0);
            rotate_want_target_[kRight] =
                rotate_handle(kRight, leg[kRight].round_count, leg[kRight].phi0);
            estimator_.wbc().s = 0.0f;
        }
    }

    // ── JUMP（v2:2050-2210） ───────────────────────────────────────────────
    if (auto* jump = std::get_if<phase::Jump>(&current_)) {
        if (!std::holds_alternative<phase::Jump>(past_)) {
            if (jump->level != 1 && jump->level != 2)
                jump->level = (jump_cmd == 2) ? 2 : 1;
            jump->step = phase::Jump::Step::Press;
            jump->step_elapsed_s = 0.0f;
            jump->confirm_s = 0.0f;
        }

        switch (jump->step) {
        case phase::Jump::Step::Press: {
            ll_want_target_[kLeft] = p.jump_press_ll_want;
            ll_want_target_[kRight] = p.jump_press_ll_want;
            jump->step_elapsed_s += dt;
            ramp_leg_rotate_[kLeft].reset(leg[kLeft].phi0_total);
            ramp_leg_rotate_[kRight].reset(leg[kRight].phi0_total);
            if (leg[kLeft].L0 < p.jump_press_ll_want && leg[kRight].L0 < p.jump_press_ll_want) {
                jump->confirm_s += dt;
            } else {
                jump->confirm_s = 0.0f;
            }
            if (jump->confirm_s > p.jump_press_confirm_s
                || jump->step_elapsed_s > p.jump_press_timeout_s) {
                float swing = 0.5f * (leg[kLeft].phi0 + leg[kRight].phi0);
                while (swing > kPi) swing -= 2.0f * kPi;
                while (swing < -kPi) swing += 2.0f * kPi;
                jump->locked_swing = swing;
                jump->confirm_s = 0.0f;
                jump->step_elapsed_s = 0.0f;
                jump->step = phase::Jump::Step::TakeOff;
            }
            break;
        }
        case phase::Jump::Step::TakeOff: {
            float takeoff_ll =
                (jump->level == 2) ? p.take_off_ll_high : p.take_off_ll_low;
            takeoff_ll = clip(takeoff_ll + p.jump_takeoff_ll_margin, 0.20f, 0.40f);
            ll_want_target_[kLeft] = takeoff_ll;
            ll_want_target_[kRight] = takeoff_ll;
            jump->step_elapsed_s += dt;

            if (ground[kLeft] == 1 && ground[kRight] == 1 && leg[kLeft].L0 > p.jump_takeoff_air_ll_thres
                && leg[kRight].L0 > p.jump_takeoff_air_ll_thres) {
                jump->confirm_s += dt;
            } else {
                jump->confirm_s = jump->confirm_s > p.jump_takeoff_decay_per_tick_s
                                    ? jump->confirm_s - p.jump_takeoff_decay_per_tick_s
                                    : 0.0f;
            }

            const bool takeoff_ready = (jump->step_elapsed_s > p.jump_takeoff_min_push_s)
                                    && (jump->confirm_s > p.jump_takeoff_confirm_s);
            if (takeoff_ready || jump->step_elapsed_s > p.jump_takeoff_timeout_s) {
                jump->confirm_s = 0.0f;
                jump->step_elapsed_s = 0.0f;
                ramp_leg_len_[kLeft].reset(leg[kLeft].L0);
                ramp_leg_len_[kRight].reset(leg[kRight].L0);
                jump->step = phase::Jump::Step::Flying;
            }
            break;
        }
        case phase::Jump::Step::Flying: {
            ll_want_target_[kLeft] = p.jump_flying_ll_want;
            ll_want_target_[kRight] = p.jump_flying_ll_want;
            jump->step_elapsed_s += dt;
            rotate_want_target_[kLeft] =
                leg[kLeft].round_count * 2 * kPi + jump->locked_swing;
            rotate_want_target_[kRight] =
                leg[kRight].round_count * 2 * kPi + jump->locked_swing;

            const bool both_legs_short =
                leg[kLeft].L0 < p.jump_flying_touch_ll && leg[kRight].L0 < p.jump_flying_touch_ll;
            if (both_legs_short) {
                jump->confirm_s += dt;
            } else {
                jump->confirm_s = 0.0f;
            }
            if (jump->confirm_s > p.jump_flying_confirm_s
                || jump->step_elapsed_s > p.jump_flying_timeout_s) {
                jump->confirm_s = 0.0f;
                jump->step_elapsed_s = 0.0f;
                jump->step = phase::Jump::Step::Landing;
            }
            break;
        }
        case phase::Jump::Step::Landing: {
            ll_want_target_[kLeft] = p.jump_landing_ll_want;
            ll_want_target_[kRight] = p.jump_landing_ll_want;
            ll_state_ = 0;
            jump->step_elapsed_s += dt;

            const bool both_legs_short_land = leg[kLeft].L0 < p.jump_landing_ll_thres
                                           && leg[kRight].L0 < p.jump_landing_ll_thres;
            if (both_legs_short_land) {
                jump->confirm_s += dt;
            } else {
                jump->confirm_s = 0.0f;
            }
            if (jump->confirm_s > p.jump_landing_confirm_s
                || jump->step_elapsed_s > p.jump_landing_timeout_s) {
                current_ = phase::Standing{}; // v2 COMPLETE → NORMAL
            }
            break;
        }
        }
    }

    // ── 跟随（v2:2212-2235） ───────────────────────────────────────────────
    if_follow_ = std::holds_alternative<phase::Standing>(current_) && !spin_active_;
    if (std::holds_alternative<phase::Standing>(current_) && !spin_active_) {
        const bool flipped = command.if_turn ^ is_inversed_;
        const float head_offset = flipped ? kPi : 0.0f;
        follow_angle_ = rad_format(wbc.phi + command.real_angle + head_offset);
    }

    // ── 斜坡速率选择（v2:2237-2273；NORMAL/ Fallen 不改 rotate_speed，怪癖保留） ──
    switch (kind_of(current_)) {
    case PhaseKind::Standing:
        if (!spin_active_)
            l0_speed_ = p.l0_speed_normal;
        else
            l0_speed_ = p.l0_speed_normal;
        break;
    case PhaseKind::SlowStart:
        l0_speed_ = p.l0_speed_slow_start;
        rotate_speed_ = p.rotate_speed_slow_start;
        break;
    case PhaseKind::Jump:
        l0_speed_ = p.l0_speed_jump;
        rotate_speed_ = p.rotate_speed_jump;
        break;
    case PhaseKind::SelfHeal:
        l0_speed_ = p.l0_speed_self_heal;
        rotate_speed_ = p.rotate_speed_self_heal;
        break;
    case PhaseKind::Fly:
        l0_speed_ = p.l0_speed_fly;
        rotate_speed_ = p.rotate_speed_fly;
        break;
    case PhaseKind::UpStair:
        l0_speed_ = p.l0_speed_up_stair;
        rotate_speed_ = p.rotate_speed_up_stair;
        break;
    default:
        break;
    }

    if (std::getenv("HCS_MTRACE") != nullptr) {
        std::fprintf(
            stderr, "[m%llu] kind=%d step=%d rotT=%.4f rot=%.4f phi0t=%.4f\n",
            (unsigned long long)debug_tick_++, static_cast<int>(kind_of(current_)),
            std::holds_alternative<phase::SlowStart>(current_)
                ? static_cast<int>(std::get<phase::SlowStart>(current_).step)
                : -1,
            rotate_want_target_[kLeft], estimator_.wbc().rotate_angle[kLeft],
            estimator_.estimate().leg[kLeft].phi0_total);
    }
    // ── 斜坡输出（v2:2275-2278） ───────────────────────────────────────────
    // 斜坡速率以秒制存储（PORTING.md §7），RampFunction 接口是每拍增量 → 乘 dt。
    estimator_.wbc().LL_want[kLeft] = ramp_leg_len_[kLeft](ll_want_target_[kLeft], l0_speed_ * dt);
    estimator_.wbc().LL_want[kRight] =
        ramp_leg_len_[kRight](ll_want_target_[kRight], l0_speed_ * dt);
    estimator_.wbc().rotate_angle[kLeft] =
        ramp_leg_rotate_[kLeft](rotate_want_target_[kLeft], rotate_speed_ * dt);
    estimator_.wbc().rotate_angle[kRight] =
        ramp_leg_rotate_[kRight](rotate_want_target_[kRight], rotate_speed_ * dt);

    if (leg_change_lock_s_ > 0.0f)
        leg_change_lock_s_ -= dt;

    standing_lock_active_ = std::holds_alternative<phase::Standing>(current_)
                         && std::get<phase::Standing>(current_).elapsed_s < params_.standing_lock_s;

    // ── 控制器结构（controller_for + UpStair 相位 2 的一拍 DISABLED） ───────
    if (flag_disabled_override_) {
        process_flag_ = ProcessFlag::Disabled;
        flag_disabled_override_ = false;
    } else {
        process_flag_ = controller_for(current_);
    }

    return MachineView{
        .process_flag = process_flag_,
        .process_flag_past = process_flag_past_,
        .phase_changed = phase_changed,
        .if_follow = if_follow_,
        .spinning = spin_active_,
        .v_target = v_target_,
        .v_raw_target = v_raw_target_,
        .acc_fwd = acc_fwd_,
        .yaw_rate_cmd = v_yaw_,
        .dial_level = dial_level_,
        .follow_angle = follow_angle_,
        .is_inversed = is_inversed_,
    };
}

} // namespace hcs_core::controller::balance
