#include "controller.hpp"

#include <cmath>

namespace hcs_core::controller::balance {

float BalanceController::torque_to_current(float torque) const {
    if (std::fabs(torque) < params_.torque_to_current_threshold)
        return 0.0f;
    const float coulomb = params_.torque_to_current_coulomb_k
                        * std::tanh(torque / params_.torque_to_current_coulomb_v);
    return -(params_.torque_to_current_k * (torque / params_.wheel_reduction) + coulomb)
         * params_.wheel_current_lsb_per_amp;
}

void BalanceController::update(
    const Estimate& est, const MachineView& view, const Phase& phase, bool jump_step_changed,
    float dt, uint64_t tick) {
    // v2 Chassis_Controller() 第一步：先 Update（状态向量 + process 分发），再模式分发。
    controller_update(est, view, phase, dt, tick);
    controller_dispatch(est, view, phase, jump_step_changed, dt, tick);
    motor_send(est, kind_of(phase), dt, tick);
}

void BalanceController::controller_update(
    const Estimate& est, const MachineView& view, const Phase& phase, float dt,
    uint64_t tick) {
    const auto& leg = est.leg;
    const auto& wbc = est.wbc;

    // v2:526-541 —— 阶段或控制器结构变化：复位 MPC/输出/轮 PID/卡转/KF 位移。
    if (view.phase_changed
        || view.process_flag != view.process_flag_past) {
        nlmpc_.Reset();
        roll_mpc_.Reset();
        output_[kLeft].final_tw = 0.0f;
        output_[kRight].final_tw = 0.0f;
        output_[kLeft].final_tl0 = 0.0f;
        output_[kLeft].final_tl1 = 0.0f;
        output_[kRight].final_tl0 = 0.0f;
        output_[kRight].final_tl1 = 0.0f;
        pid_wheel_current_l_.clear_i();
        pid_wheel_current_r_.clear_i();
        estimator_.reset_kf_stall();
        estimator_.wbc().s = 0.0f;
    }

    lqr_k(leg[kLeft].L0, leg[kRight].L0, k_matrix_);

    const float target_v = view.v_target;

    // ── 驻车刹车与启动助力（v2:553-625） ───────────────────────────────────
    constexpr float kParkCmdThresh = 0.05f;
    constexpr float kParkSpdThresh = 0.20f;
    constexpr float kParkPosGain = 5.0f;
    constexpr float kParkMaxPosErr = 0.30f;
    constexpr float kParkErrDeadband = 0.005f;
    constexpr float kLaunchRatio = 0.50f;
    constexpr float kLaunchMinRaw = 0.10f;
    constexpr float kLaunchPosGain = 0.35f;
    constexpr float kLaunchMaxOffset = 0.50f;
    constexpr float kPosFilterAlpha = 0.314f; // 60 Hz @1kHz

    float raw_pos = 0.0f;
    if (kind_of(phase) == PhaseKind::Standing && !view.spinning) {
        const float abs_raw = std::fabs(view.v_raw_target);
        if (abs_raw >= kParkCmdThresh) {
            park_brake_state_ = ParkBrake::Off;
            const float abs_spd = std::fabs(wbc.s1);
            if (abs_spd < kLaunchRatio * abs_raw && abs_raw > kLaunchMinRaw) {
                float deficit = abs_raw - abs_spd;
                float offset = kLaunchPosGain * deficit;
                if (offset > kLaunchMaxOffset)
                    offset = kLaunchMaxOffset;
                raw_pos = (view.v_raw_target > 0.0f ? 1.0f : -1.0f) * offset;
            }
        } else {
            switch (park_brake_state_) {
            case ParkBrake::Off:
                if (std::fabs(target_v) < kParkCmdThresh)
                    park_brake_state_ = ParkBrake::Waiting;
                break;
            case ParkBrake::Waiting:
                if (std::fabs(wbc.s1) < kParkSpdThresh) {
                    park_brake_state_ = ParkBrake::Engaged;
                    park_s_locked_ = wbc.s;
                }
                break;
            case ParkBrake::Engaged: {
                float pos_err = park_s_locked_ - wbc.s;
                if (std::fabs(pos_err) < kParkErrDeadband)
                    pos_err = 0.0f;
                pos_err = abs_clip(pos_err, kParkMaxPosErr);
                raw_pos = kParkPosGain * pos_err;
                break;
            }
            }
        }
    } else {
        park_brake_state_ = ParkBrake::Off;
    }

    pos_filter_state_ += kPosFilterAlpha * (raw_pos - pos_filter_state_);
    status_vector_[0] = pos_filter_state_;

    // v2:628-630
    float v_err = abs_clip(target_v - wbc.s1, 2.5f);
    if (std::fabs(v_err) < 0.005f)
        v_err = 0;

    // v2:632-642 —— SPIN/FLY/JUMP-TakeOff 断开速度跟踪。
    const bool jump_takeoff = std::holds_alternative<phase::Jump>(phase)
                           && std::get<phase::Jump>(phase).step == phase::Jump::Step::TakeOff;
    if (view.spinning || kind_of(phase) == PhaseKind::Fly || jump_takeoff) {
        status_vector_[1] = 0.0f;
    } else {
        status_vector_[1] = v_err;
    }

    // ── yaw 跟随（v2:663-700） ─────────────────────────────────────────────
    if (kind_of(phase) != PhaseKind::Fly && kind_of(phase) != PhaseKind::Fallen
        && kind_of(phase) != PhaseKind::Disabled && view.if_follow) {
        float raw_yaw_error = view.follow_angle - wbc.phi;
        while (raw_yaw_error > kPi) raw_yaw_error -= 2.0f * kPi;
        while (raw_yaw_error < -kPi) raw_yaw_error += 2.0f * kPi;

        const float abs_yaw_error = std::fabs(raw_yaw_error);
        float yaw_error_input = raw_yaw_error;

        const bool is_stuck = (std::fabs(wbc.s1) < 0.2f)
                           && (std::fabs(output_last_[kLeft].tw) + std::fabs(output_last_[kRight].tw)
                               > params_.t_max_wheel * 1.2f);

        if (abs_yaw_error > 0.8f && is_stuck) {
            const float penalty_factor = std::exp(-1.8f * (abs_yaw_error - 0.8f));
            yaw_error_input *= penalty_factor;
        }

        const float max_yaw_step = 0.5f;
        if (is_stuck && abs_yaw_error <= 0.8f) {
            smoothed_yaw_error_ *= 0.95f;
        } else {
            smoothed_yaw_error_ +=
                abs_clip(yaw_error_input - smoothed_yaw_error_, max_yaw_step);
        }

        status_vector_[2] = 0.85f * std::tanh(smoothed_yaw_error_ / 0.6f);
    } else {
        status_vector_[2] = 0.0f;
    }

    // ── yaw 速率（v2:702-754） ─────────────────────────────────────────────
    float yaw_rate_cmd_raw = 0.0f;
    if (view.spinning) {
        yaw_rate_cmd_raw = (view.yaw_rate_cmd - wbc.phi1);
    } else {
        if (view.if_follow) {
            yaw_rate_cmd_raw = -wbc.phi1;
        } else {
            yaw_rate_cmd_raw = (view.yaw_rate_cmd - wbc.phi1)
                             * smooth_saturation(std::fabs(view.v_target - wbc.s1), 0.05f, 4.0f);
        }
    }

    const bool airborne_or_landing = (kind_of(phase) == PhaseKind::Fly)
                                  || (est.ground.if_off_gnd[kLeft] != 0)
                                  || (est.ground.if_off_gnd[kRight] != 0);
    if (airborne_or_landing) {
        landing_recovery_factor_ = 0.15f;
    } else {
        landing_recovery_factor_ += 0.005f;
        if (landing_recovery_factor_ > 1.0f)
            landing_recovery_factor_ = 1.0f;
    }

    const float yaw_limit = 0.5f + (5.0f - 0.5f) * landing_recovery_factor_;
    const float yaw_slew = 0.08f + (0.4f - 0.08f) * landing_recovery_factor_;

    const float yaw_target = clip(yaw_rate_cmd_raw, -yaw_limit, yaw_limit);
    yaw_rate_cmd_limited_ += abs_clip(yaw_target - yaw_rate_cmd_limited_, yaw_slew);
    if (std::fabs(yaw_rate_cmd_limited_) < 0.001f)
        yaw_rate_cmd_limited_ = 0.0f;

    status_vector_[3] = view.spinning ? yaw_rate_cmd_raw : yaw_rate_cmd_limited_;
    if (kind_of(phase) == PhaseKind::Fly)
        status_vector_[3] = 0.0f;

    // ── 腿摆角预设（v2:756-830；pitch_preset_used 当前被强制清零，原样保留） ──
    const float adapt_leg_angle_preset = 0.0f;
    const bool in_jump_latch_window =
        (kind_of(phase) == PhaseKind::Jump)
        && (std::get<phase::Jump>(phase).step == phase::Jump::Step::Press
            || std::get<phase::Jump>(phase).step == phase::Jump::Step::TakeOff);
    const bool in_fly_latch_window = kind_of(phase) == PhaseKind::Fly;

    if (in_jump_latch_window && !adapt_leg_angle_preset_latched_valid_) {
        adapt_leg_angle_preset_latched_ = 0.0f;
        adapt_leg_angle_preset_latched_valid_ = true;
    } else if (in_fly_latch_window) {
        adapt_leg_angle_preset_latched_ = adapt_leg_angle_preset;
        adapt_leg_angle_preset_latched_valid_ = true;
    } else if (!in_jump_latch_window) {
        adapt_leg_angle_preset_latched_valid_ = false;
    }

    const bool is_jump_press_phase =
        (kind_of(phase) == PhaseKind::Jump)
        && std::get<phase::Jump>(phase).step == phase::Jump::Step::Press;
    float pitch_preset_used = params_.pitch_preset;
    if (!is_jump_press_phase) {
        const float low_pitch_offset = 0.01f;
        const float mid_pitch_offset = 0.0f;
        const float high_pitch_offset = -0.02f;

        float low_ll_ref = 0.17f;
        float mid_ll_ref = 0.25f;
        float high_ll_ref = 0.36f;
        if (view.spinning) {
            low_ll_ref = 0.16f;
            mid_ll_ref = 0.22f;
            high_ll_ref = 0.32f;
        }

        const float ll_avg_want = 0.5f * (wbc.LL_want[kLeft] + wbc.LL_want[kRight]);
        float pitch_offset = 0.0f;
        if (ll_avg_want <= mid_ll_ref) {
            const float low_to_mid_ratio =
                clip((ll_avg_want - low_ll_ref) / (mid_ll_ref - low_ll_ref), 0.0f, 1.0f);
            pitch_offset =
                low_pitch_offset + (mid_pitch_offset - low_pitch_offset) * low_to_mid_ratio;
        } else {
            const float mid_to_high_ratio =
                clip((ll_avg_want - mid_ll_ref) / (high_ll_ref - mid_ll_ref), 0.0f, 1.0f);
            pitch_offset =
                mid_pitch_offset + (high_pitch_offset - mid_pitch_offset) * mid_to_high_ratio;
        }
        pitch_preset_used += pitch_offset;
    }
    // v2:825-826 —— "Temporarily disable the leg swing angle bias while validating NLMPC."
    pitch_preset_used = 0.0f;

    const float adapt_leg_angle_preset_used =
        ((in_jump_latch_window || in_fly_latch_window) && adapt_leg_angle_preset_latched_valid_)
            ? adapt_leg_angle_preset_latched_
            : adapt_leg_angle_preset;

    if (wbc.LL_want[kLeft] > params_.wheel_speed_takeoff_ll_threshold
        && kind_of(phase) != PhaseKind::Fly) {
        status_vector_[4] = 0 - wbc.thetall + pitch_preset_used;
    } else {
        status_vector_[4] = 0 - wbc.thetall + adapt_leg_angle_preset_used + pitch_preset_used;
    }
    status_vector_[5] = 0 - wbc.thetall1;

    if (wbc.LL_want[kRight] > params_.wheel_speed_takeoff_ll_threshold
        && kind_of(phase) != PhaseKind::Fly) {
        status_vector_[6] = 0 - wbc.thetalr + pitch_preset_used;
    } else {
        status_vector_[6] = 0 - wbc.thetalr + adapt_leg_angle_preset_used + pitch_preset_used;
    }
    status_vector_[7] = 0 - wbc.thetalr1;

    const bool is_in_jump_window =
        (kind_of(phase) == PhaseKind::Jump)
        && (std::get<phase::Jump>(phase).step == phase::Jump::Step::Press
            || std::get<phase::Jump>(phase).step == phase::Jump::Step::TakeOff);
    const float target_thetab = is_in_jump_window ? params_.jump_press_thetab_ref : 0.0f;
    status_vector_[8] = abs_clip(target_thetab - wbc.thetab, params_.pitch_clip);
    status_vector_[9] = 0.0f - wbc.thetab1;

    // ── process_flag 分发（v2:873-1022；LESO/功率 QP 为死代码不移植） ─────────
    switch (view.process_flag) {
    case ProcessFlag::Disabled:
        output_[kLeft].tw = 0;
        output_[kRight].tw = 0;
        output_[kLeft].tl = 0;
        output_[kRight].tl = 0;
        output_[kLeft].final_tl0 = 0;
        output_[kLeft].final_tl1 = 0;
        output_[kRight].final_tl0 = 0;
        output_[kRight].final_tl1 = 0;
        estimator_.reset_kf();
        break;

    case ProcessFlag::LqrOn: {
        if (view.process_flag_past != ProcessFlag::LqrOn) {
            estimator_.reset_kf();
            pid_ll_.clear_i();
            pid_lr_.clear_i();
            pid_leg_rotate_l_.clear_i();
            pid_leg_rotate_r_.clear_i();
        }

        float nlmpc_output[4]{};
        bool nlmpc_ok = nlmpc_.Solve(
            leg[kLeft].L0, leg[kRight].L0, status_vector_.data(), wbc.thetall, wbc.thetalr,
            nlmpc_output);
        // v2 的 DWT 周期上限是 STM32 侧的实时性护栏；x86 上不设上限，
        // 阶段 3 用逐组件计时复核（PORTING.md §10-5）。

        if (nlmpc_ok) {
            output_[kLeft].tw = nlmpc_output[0];
            output_[kRight].tw = nlmpc_output[1];
            output_[kLeft].tl = nlmpc_output[2];
            output_[kRight].tl = nlmpc_output[3];
        } else {
            nlmpc_.MarkFallback();
            calculate_status_vector_fly(
                k_matrix_, status_vector_, output_[kLeft].tw, output_[kRight].tw,
                output_[kLeft].tl, output_[kRight].tl, est.ground.if_off_gnd[kLeft] != 0,
                est.ground.if_off_gnd[kRight] != 0, 0.1f);
        }

        output_[kLeft].final_tw = abs_clip(output_[kLeft].tw, params_.t_max_wheel);
        output_[kRight].final_tw = abs_clip(output_[kRight].tw, params_.t_max_wheel);

        if (est.ground.if_off_gnd[kLeft] != 0)
            output_[kLeft].final_tw = 0.0f;
        if (est.ground.if_off_gnd[kRight] != 0)
            output_[kRight].final_tw = 0.0f;

        output_[kLeft].tl = abs_clip(output_[kLeft].tl, params_.t_max_leg);
        output_[kRight].tl = abs_clip(output_[kRight].tl, params_.t_max_leg);
        break;
    }

    case ProcessFlag::PidOnly:
        if (view.process_flag_past == ProcessFlag::LqrOn) {
            estimator_.reset_kf();
            pid_wheel_current_l_.clear_i();
            pid_wheel_current_r_.clear_i();
        }

        if (kind_of(phase) == PhaseKind::SlowStart || kind_of(phase) == PhaseKind::SelfHeal) {
            output_[kLeft].tl = -pid_leg_rotate_l_.update(
                leg[kLeft].phi0_total, wbc.rotate_angle[kLeft], dt, tick);
            output_[kRight].tl = -pid_leg_rotate_r_.update(
                leg[kRight].phi0_total, wbc.rotate_angle[kRight], dt, tick);
            output_[kLeft].tw = 0;
            output_[kRight].tw = 0;
        } else if (
            kind_of(phase) == PhaseKind::Fly
            || (kind_of(phase) == PhaseKind::Jump
                && std::get<phase::Jump>(phase).step >= phase::Jump::Step::TakeOff)) {
            output_[kLeft].tw = 0;
            output_[kRight].tw = 0;
        } else if (kind_of(phase) == PhaseKind::UpStair) {
            output_[kLeft].tl = -pid_leg_rotate_l_.update(
                leg[kLeft].phi0_total, wbc.rotate_angle[kLeft], dt, tick);
            output_[kRight].tl = -pid_leg_rotate_r_.update(
                leg[kRight].phi0_total, wbc.rotate_angle[kRight], dt, tick);
            output_[kLeft].tw = 0;
            output_[kRight].tw = 0;
        }
        if (kind_of(phase) != PhaseKind::UpStair) {
            output_[kLeft].final_tw = 0.0f;
            output_[kRight].final_tw = 0.0f;
        }
        break;

    case ProcessFlag::Healing:
        output_[kLeft].tw = 0;
        output_[kRight].tw = 0;
        output_[kLeft].tl =
            -pid_leg_rotate_l_.update(leg[kLeft].phi0_total, wbc.rotate_angle[kLeft], dt, tick);
        output_[kRight].tl =
            -pid_leg_rotate_r_.update(leg[kRight].phi0_total, wbc.rotate_angle[kRight], dt, tick);
        output_[kLeft].final_tw = 0.0f;
        output_[kRight].final_tw = 0.0f;
        pid_wheel_current_l_.clear_i();
        pid_wheel_current_r_.clear_i();
        estimator_.reset_kf();
        break;
    }
}

void BalanceController::controller_dispatch(
    const Estimate& est, const MachineView& view, const Phase& phase, bool jump_step_changed,
    float dt, uint64_t tick) {
    const auto& leg = est.leg;
    const auto& wbc = est.wbc;
    const auto& p = params_;
    const auto mode = kind_of(phase);
    const float m_t = (p.M + 2 * (p.M_w + p.M_l));

    // v2 里 Fallen（fatal 的 DISABLED）与 Disabled 同样走无力短路。
    if (mode == PhaseKind::Disabled || mode == PhaseKind::Fallen) {
        output_[kLeft].tw = 0;
        output_[kRight].tw = 0;
        output_[kLeft].tl = 0;
        output_[kRight].tl = 0;
        output_[kLeft].final_tw = 0;
        output_[kRight].final_tw = 0;
        output_[kLeft].final_tl0 = 0;
        output_[kLeft].final_tl1 = 0;
        output_[kRight].final_tl0 = 0;
        output_[kRight].final_tl1 = 0;
        pid_ll_.clear_i();
        pid_lr_.clear_i();
        pid_leg_rotate_l_.clear_i();
        pid_leg_rotate_r_.clear_i();
        pid_wheel_current_l_.clear_i();
        pid_wheel_current_r_.clear_i();
        return;
    }

    bool is_roll_pid_enable = false;
    const float f_ff = -m_t * p.G / 2.0f;

    // ── 模式分发（v2:251-352） ─────────────────────────────────────────────
    switch (mode) {
    case PhaseKind::Standing:
        is_roll_pid_enable = true;
        if_jump_controller_enable_ = false;
        break;
    case PhaseKind::SlowStart:
        is_roll_pid_enable = false;
        if_jump_controller_enable_ = false;
        break;
    case PhaseKind::Jump: {
        const auto& jump = std::get<phase::Jump>(phase);
        switch (jump.step) {
        case phase::Jump::Step::Press:
            is_roll_pid_enable = true;
            if_jump_controller_enable_ = false;
            break;
        case phase::Jump::Step::TakeOff: {
            if_jump_controller_enable_ = true;

            // v2:277-305。注意：这里算出的 F_want 在 398-409 处会被 pid_LL 路径覆盖
            //（if_jump_controller_enable 分支），实际不生效——按原样保留，见 §9-13。
            const float az_ref = (jump.level == 2) ? p.az_ref_big : p.az_ref_small;
            const float kp_acc = p.takeoff_kp_acc;
            const float kd_acc = p.takeoff_kd_acc;

            float acc_err = az_ref - wbc.a_z;
            if (jump_step_changed) {
                takeoff_acc_err_last_ = acc_err;
                filter_takeoff_acc_err_d_.reset(
                    p.filter_takeoff_acc_err_d.past, p.filter_takeoff_acc_err_d.now);
            }

            float acc_err_d_raw = (acc_err - takeoff_acc_err_last_) / dt;
            float acc_err_d = filter_takeoff_acc_err_d_(acc_err_d_raw);
            takeoff_acc_err_last_ = acc_err;

            float add_force_total = m_t * (kp_acc * acc_err + kd_acc * acc_err_d);
            add_force_total = clip(add_force_total, 0.0f, m_t * p.G);
            const float add_force_leg = 0.5f * add_force_total;

            output_[kLeft].f_want = f_ff - add_force_leg;
            output_[kRight].f_want = f_ff - add_force_leg;
            break;
        }
        case phase::Jump::Step::Flying: {
            if_jump_controller_enable_ = true;
            output_[kLeft].f_want = 0.0f;
            output_[kRight].f_want = 0.0f;
            const float pitch_pd =
                clip((p.flying_kp_pitch * wbc.thetab + p.flying_kd_pitch * wbc.thetab1),
                     -p.flying_pitch_clip, p.flying_pitch_clip);
            const float theta_ref_l = wbc.rotate_angle[kLeft] + pitch_pd;
            const float theta_ref_r = wbc.rotate_angle[kRight] + pitch_pd;
            output_[kLeft].tl =
                -pid_leg_rotate_l_.update(leg[kLeft].phi0_total, theta_ref_l, dt, tick);
            output_[kRight].tl =
                -pid_leg_rotate_r_.update(leg[kRight].phi0_total, theta_ref_r, dt, tick);
            break;
        }
        case phase::Jump::Step::Landing:
            is_roll_pid_enable = true;
            if_jump_controller_enable_ = false;
            break;
        }
        break;
    }
    case PhaseKind::Fly:
        if_jump_controller_enable_ = true;
        is_roll_pid_enable = false;
        output_[kLeft].f_want = 0.0f;
        output_[kRight].f_want = 0.0f;
        break;
    case PhaseKind::SelfHeal:
        is_roll_pid_enable = false;
        if_jump_controller_enable_ = true;
        break;
    default:
        break;
    }

    // ── roll MPC / PID（v2:354-395） ───────────────────────────────────────
    bool roll_mpc_ok = false;
    float roll_differential_force = 0.0f;
    float roll_length_fallback = 0.0f;
    if (is_roll_pid_enable && est.ground.if_off_gnd[kLeft] == 0
        && est.ground.if_off_gnd[kRight] == 0) {
        const float roll_err = filter_roll_(wbc.roll);
        const float roll_d = filter_roll1_(wbc.roll1);

        roll_mpc_ok = roll_mpc_.Solve(
            roll_err, roll_d, p.roll_force_arm, p.roll_inertia, p.roll_df_max,
            roll_differential_force);

        float speed_factor = 0.0f;
        if (view.spinning) {
            const float abs_yaw_rate = std::fabs(wbc.phi1);
            speed_factor = clip((abs_yaw_rate - 3.0f) / 5.0f, 0.0f, 1.0f);
        }

        float ki_roll = 0.0005f - speed_factor * (0.0005f - 0.0003f);
        ki_roll += clip((std::fabs(roll_err) - 0.05f) * 0.01f, 0.0f, 0.002f);

        const float stab_limit = 0.15f;
        stab_roll_ += ki_roll * (0.0f - roll_err);
        stab_roll_ = abs_clip(stab_roll_, stab_limit);

        const float kd_base = 0.02f;
        const float kd_var = kd_base + 0.002f * std::exp(-std::fabs(roll_err) * 20.0f);
        const float kd_actual = kd_var - speed_factor * (kd_var - 0.00003f);

        const float d_term =
            (std::fabs(roll_d) > 0.01f) ? (kd_actual * (0.0f - roll_d)) : 0.0f;

        roll_length_fallback = abs_clip(stab_roll_ + d_term, stab_limit);
        f_roll_add_ = roll_mpc_ok ? roll_differential_force : roll_length_fallback;
    } else {
        stab_roll_ = 0.0f;
        f_roll_add_ = 0.0f;
        roll_mpc_.Reset();
    }

    // ── 腿长 F_want（v2:398-414） ──────────────────────────────────────────
    if (!if_jump_controller_enable_) {
        const float roll_offset = roll_mpc_ok ? 0.0f : roll_length_fallback;
        output_[kLeft].f_want =
            f_ff - pid_ll_.update(
                       leg[kLeft].L0, clip(wbc.LL_want[kLeft] - roll_offset, 0.18f, 0.40f), dt,
                       tick);
        output_[kRight].f_want =
            f_ff - pid_lr_.update(
                       leg[kRight].L0, clip(wbc.LL_want[kRight] + roll_offset, 0.18f, 0.40f),
                       dt, tick);
    } else {
        const float roll_offset = roll_mpc_ok ? 0.0f : roll_length_fallback;
        output_[kLeft].f_want =
            -pid_ll_.update(leg[kLeft].L0, wbc.LL_want[kLeft] - roll_offset, dt, tick);
        output_[kRight].f_want =
            -pid_lr_.update(leg[kRight].L0, wbc.LL_want[kRight] + roll_offset, dt, tick);
    }
    if (roll_mpc_ok) {
        output_[kLeft].f_want += roll_differential_force;
        output_[kRight].f_want -= roll_differential_force;
    }

    // ── TAKE_OFF 俯仰补偿（v2:416-429） ────────────────────────────────────
    float tl_comp_l = 0.0f;
    float tl_comp_r = 0.0f;
    if (mode == PhaseKind::Jump
        && std::get<phase::Jump>(phase).step == phase::Jump::Step::TakeOff) {
        const float f_total_l = output_[kLeft].f_want + leg[kLeft].spring_force;
        const float f_total_r = output_[kRight].f_want + leg[kRight].spring_force;
        tl_comp_l = f_total_l * leg[kLeft].CoM_x * p.takeoff_pitch_comp;
        tl_comp_r = f_total_r * leg[kRight].CoM_x * p.takeoff_pitch_comp;
    }
    output_[kLeft].tl += tl_comp_l;
    output_[kRight].tl += tl_comp_r;

    float spring_comp_l_base = leg[kLeft].spring_force;
    float spring_comp_r_base = leg[kRight].spring_force;

    output_last_[kLeft] = output_[kLeft];
    output_last_[kRight] = output_[kRight];

    // ── 柔顺权重（v2:440-475） ─────────────────────────────────────────────
    const bool is_jump_flying =
        (mode == PhaseKind::Jump
         && std::get<phase::Jump>(phase).step == phase::Jump::Step::Flying);
    for (const std::size_t leg_i : {kLeft, kRight}) {
        float target_weight = p.compliance_gnd;
        if (mode == PhaseKind::Fly) {
            if (est.ground.if_off_gnd[leg_i] == 1) {
                target_weight = p.compliance_air;
            } else if (est.ground.if_off_gnd[leg_i] == 2) {
                target_weight = p.compliance_impact;
            }
        }
        compliance_weight_[leg_i] +=
            (target_weight - compliance_weight_[leg_i]) * p.compliance_filter;

        if (leg_i == kLeft) {
            if (!is_jump_flying)
                output_[kLeft].f_want *= compliance_weight_[kLeft];
            spring_comp_l_base *= compliance_weight_[kLeft];
        } else {
            if (!is_jump_flying)
                output_[kRight].f_want *= compliance_weight_[kRight];
            spring_comp_r_base *= compliance_weight_[kRight];
        }
    }

    // ── VMC（v2:477-482） ──────────────────────────────────────────────────
    leg_vmc(
        leg[kLeft].phi0, leg[kLeft].phi[0], leg[kLeft].phi[1], leg[kLeft].phi[2],
        leg[kLeft].phi[3], p.L1, leg[kLeft].L0, output_[kLeft].f_want + spring_comp_l_base,
        output_[kLeft].tl, output_[kLeft].final_tl0, output_[kLeft].final_tl1);
    leg_vmc(
        leg[kRight].phi0, leg[kRight].phi[0], leg[kRight].phi[1], leg[kRight].phi[2],
        leg[kRight].phi[3], p.L1, leg[kRight].L0, output_[kRight].f_want + spring_comp_r_base,
        output_[kRight].tl, output_[kRight].final_tl0, output_[kRight].final_tl1);
}

void BalanceController::motor_send(const Estimate& est, PhaseKind mode, float dt, uint64_t tick) {
    const auto& p = params_;

    if (mode != PhaseKind::Disabled && mode != PhaseKind::Fallen) {
        output_[kLeft].final_tw = abs_clip(output_[kLeft].final_tw, p.motor_send_wheel_clip);
        output_[kRight].final_tw = abs_clip(output_[kRight].final_tw, p.motor_send_wheel_clip);

        commands_.leg_torque[0] = abs_clip(output_[kLeft].final_tl1, p.motor_send_leg_clip); // bl1
        commands_.leg_torque[1] = abs_clip(output_[kLeft].final_tl0, p.motor_send_leg_clip); // bl0
        commands_.leg_torque[2] = abs_clip(-output_[kRight].final_tl1, p.motor_send_leg_clip); // br1
        commands_.leg_torque[3] = abs_clip(-output_[kRight].final_tl0, p.motor_send_leg_clip); // br0

        const float current_l = torque_to_current(output_[kLeft].final_tw);
        const float current_r = torque_to_current(-output_[kRight].final_tw);
        commands_.wheel_current[0] = pid_wheel_current_l_.update(
            est.input_wheel_current[0], abs_clip(current_l, p.wheel_current_clip), dt, tick);
        commands_.wheel_current[1] = pid_wheel_current_r_.update(
            est.input_wheel_current[1], abs_clip(current_r, p.wheel_current_clip), dt, tick);
    } else {
        for (auto& t : commands_.leg_torque) t = 0.0f;
        for (auto& c : commands_.wheel_current) c = 0.0f;
    }

    telemetry_.final_tw[0] = output_[kLeft].final_tw;
    telemetry_.final_tw[1] = output_[kRight].final_tw;
    telemetry_.final_tl0[0] = output_[kLeft].final_tl0;
    telemetry_.final_tl0[1] = output_[kRight].final_tl0;
    telemetry_.final_tl1[0] = output_[kLeft].final_tl1;
    telemetry_.final_tl1[1] = output_[kRight].final_tl1;
    telemetry_.f_want[0] = output_[kLeft].f_want;
    telemetry_.f_want[1] = output_[kRight].f_want;
}

} // namespace hcs_core::controller::balance
