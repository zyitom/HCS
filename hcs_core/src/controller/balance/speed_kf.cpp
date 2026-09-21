#include "speed_kf.hpp"

#include <cmath>

namespace hcs_core::controller::balance {

float SpeedKf::update(
    float wheel_spd_l, float wheel_spd_r, float yaw_rate_z, float acc_x, float leg_p_l,
    float leg_p_r, bool if_off_gnd_l, bool if_off_gnd_r, float leg_dphi0_l, float leg_dphi0_r,
    float dt) {
    auto& kf = state_;

    if (dt > 0.05f || dt < 0.0f) {
        dt = 0.001f;
    }

    const float wheel_spd_arr[2] = {wheel_spd_l, wheel_spd_r};
    for (int i = 0; i < 2; ++i) {
        const float wheel_drop =
            std::fabs(kf.last_wheel_spd[i]) - std::fabs(wheel_spd_arr[i]);
        if (wheel_drop > KfState::kStallWheelDropThres
            && std::fabs(kf.last_wheel_spd[i]) > 0.5f) {
            kf.wheel_stall[i] = true;
            kf.stall_recover_cnt[i] = 0.0f;
        }
        if (kf.wheel_stall[i]) {
            kf.stall_recover_cnt[i] += dt;
            if (kf.stall_recover_cnt[i] > KfState::kStallRecoverTime
                && std::fabs(wheel_spd_arr[i]) > KfState::kStallWheelSpdThres) {
                kf.wheel_stall[i] = false;
                kf.stall_recover_cnt[i] = 0.0f;
            }
        }
        kf.last_wheel_spd[i] = wheel_spd_arr[i];
    }

    bool true_off_gnd_l = false, true_off_gnd_r = false;
    kf.off_gnd_timer[0] = if_off_gnd_l ? (kf.off_gnd_timer[0] + dt) : 0.0f;
    kf.off_gnd_timer[1] = if_off_gnd_r ? (kf.off_gnd_timer[1] + dt) : 0.0f;
    if (kf.off_gnd_timer[0] > KfState::kOffGndDebounceTime)
        true_off_gnd_l = true;
    if (kf.off_gnd_timer[1] > KfState::kOffGndDebounceTime)
        true_off_gnd_r = true;

    const float p_delta_l = std::fabs(leg_p_l - kf.last_leg_p[0]);
    const float p_delta_r = std::fabs(leg_p_r - kf.last_leg_p[1]);
    kf.last_leg_p[0] = leg_p_l;
    kf.last_leg_p[1] = leg_p_r;

    if (p_delta_l > KfState::kImpactPDeltaThres || p_delta_r > KfState::kImpactPDeltaThres) {
        kf.impact_timer = KfState::kImpactCooldownTime;
    }

    float current_acc = abs_clip(acc_x, 20.0f);
    if (kf.impact_timer > 0.0f) {
        kf.impact_timer -= dt;
        current_acc = abs_clip(current_acc, 1.0f);
    }

    const float f = (kf.acc_last + current_acc) * 0.5f;
    float v_predict = kf.v_est + f * dt;
    kf.acc_last = current_acc;
    kf.cov += KfState::kQ * dt;

    const float dynamic_r_base = KfState::kRBase + 10.0f * std::fabs(yaw_rate_z);

    const float z_left = wheel_spd_l + yaw_rate_z * KfState::kWheelHalfTrack;
    const float z_right = wheel_spd_r - yaw_rate_z * KfState::kWheelHalfTrack;

    float r_resid_l, r_resid_r;
    if (kf.resid_valid) {
        r_resid_l = kf.resid_sq_sum_l + kf.cov_post;
        r_resid_r = kf.resid_sq_sum_r + kf.cov_post;
    } else {
        r_resid_l = KfState::kRResidMin;
        r_resid_r = KfState::kRResidMin;
    }
    if (r_resid_l < KfState::kRResidMin) r_resid_l = KfState::kRResidMin;
    if (r_resid_r < KfState::kRResidMin) r_resid_r = KfState::kRResidMin;

    const float r_swing_l = KfState::kRSwingGain * std::fabs(leg_dphi0_l);
    const float r_swing_r = KfState::kRSwingGain * std::fabs(leg_dphi0_r);

    float r_left = dynamic_r_base + r_resid_l + r_swing_l;
    float r_right = dynamic_r_base + r_resid_r + r_swing_r;

    if (true_off_gnd_l)
        r_left = KfState::kRMax;
    if (true_off_gnd_r)
        r_right = KfState::kRMax;
    if (kf.wheel_stall[0])
        r_left = KfState::kRMax;
    if (kf.wheel_stall[1])
        r_right = KfState::kRMax;
    if (kf.impact_timer > 0.0f) {
        r_left = KfState::kRMax;
        r_right = KfState::kRMax;
    }

    const float z_avg = 0.5f * (std::fabs(z_left) + std::fabs(z_right));
    if (z_avg < 0.05f) {
        r_left /= 5.0f;
        r_right /= 5.0f;
    }

    if (true_off_gnd_l || true_off_gnd_r) {
        v_predict *= std::exp(-KfState::kAirDecayRate * dt);
    }

    r_left = clip(r_left, KfState::kRHighSpeedMin, KfState::kRMax);
    r_right = clip(r_right, KfState::kRHighSpeedMin, KfState::kRMax);

    const float w_left = 1.0f / (r_left + 1e-6f);
    const float w_right = 1.0f / (r_right + 1e-6f);
    const float w_sum = w_left + w_right;

    float wheel_measure = v_predict;
    float r_dynamic = KfState::kRMax;
    if (w_sum > 1e-5f) {
        wheel_measure = (z_left * w_left + z_right * w_right) / w_sum;
        r_dynamic = 1.0f / w_sum;
    }

    const float k_gain = kf.cov / (kf.cov + r_dynamic);
    const float err = wheel_measure - v_predict;
    kf.v_est = v_predict + k_gain * err;
    kf.cov = (1.0f - k_gain) * kf.cov;

    if (kf.cov < KfState::kCovMin)
        kf.cov = KfState::kCovMin;
    else if (kf.cov > KfState::kCovMax)
        kf.cov = KfState::kCovMax;

    kf.cov_post = kf.cov;
    const bool trust_l =
        !(true_off_gnd_l || kf.wheel_stall[0] || kf.impact_timer > 0.0f);
    const bool trust_r =
        !(true_off_gnd_r || kf.wheel_stall[1] || kf.impact_timer > 0.0f);
    if (trust_l) {
        const float resid_l = z_left - kf.v_est;
        kf.resid_sq_sum_l = KfState::kResidEmaAlpha * kf.resid_sq_sum_l
                          + (1.0f - KfState::kResidEmaAlpha) * resid_l * resid_l;
    }
    if (trust_r) {
        const float resid_r = z_right - kf.v_est;
        kf.resid_sq_sum_r = KfState::kResidEmaAlpha * kf.resid_sq_sum_r
                          + (1.0f - KfState::kResidEmaAlpha) * resid_r * resid_r;
    }
    kf.resid_valid = true;

    const float wheel_avg = 0.5f * (std::fabs(wheel_spd_l) + std::fabs(wheel_spd_r));
    if (wheel_avg < 0.02f && std::fabs(acc_x) < 0.05f) {
        kf.v_est *= std::exp(-KfState::kZuptDecayRate * dt);
        if (std::fabs(kf.v_est) < 0.005f)
            kf.v_est = 0.0f;
    } else if (wheel_avg < 0.02f && std::fabs(kf.v_est) < 0.1f) {
        kf.v_est *= std::exp(-(KfState::kZuptDecayRate * 0.2f) * dt);
    }

    return kf.v_est;
}

} // namespace hcs_core::controller::balance
