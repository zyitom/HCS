#pragma once

// 速度卡尔曼滤波（Helios module::spd_estimation + KF_s 的逐语义移植）。
// 原实现的 dt 来自 DWT（>0.05s 或 <0 时退回 1 ms），此处由调用方传入。

#include <array>
#include <cstdint>

#include "math.hpp"

namespace hcs_core::controller::balance {

struct KfState {
    float cov = 100.0f;
    float v_est = 0.0f;
    float acc_last = 0.0f;

    // 常数（KF_s，原值）
    static constexpr float kQ = 1.3f;
    static constexpr float kRBase = 400.0f;
    static constexpr float kRMax = 1500.0f;
    static constexpr float kRHighSpeedMin = 5.0f;
    static constexpr float kCovMin = 0.01f;
    static constexpr float kCovMax = 100.0f;
    static constexpr float kWheelHalfTrack = 0.2296f;
    static constexpr float kImpactPDeltaThres = 30.0f;
    static constexpr float kImpactCooldownTime = 0.15f;
    static constexpr float kOffGndDebounceTime = 0.05f;
    static constexpr float kAirDecayRate = 1.0f;
    static constexpr float kZuptDecayRate = 10.5f;
    static constexpr float kStallWheelSpdThres = 0.1f;
    static constexpr float kStallWheelDropThres = 2.0f;
    static constexpr float kStallRecoverTime = 0.5f;
    static constexpr float kResidEmaAlpha = 0.99f;
    static constexpr float kRSwingGain = 80.0f;
    static constexpr float kRResidMin = 1.0f;

    bool wheel_stall[2] = {false, false};
    float last_wheel_spd[2] = {0.0f, 0.0f};
    float stall_recover_cnt[2] = {0.0f, 0.0f};
    float last_leg_p[2] = {0.0f, 0.0f};

    float off_gnd_timer[2] = {0.0f, 0.0f};
    float impact_timer = 0.0f;

    float resid_sq_sum_l = 0.0f;
    float resid_sq_sum_r = 0.0f;
    float cov_post = 100.0f;
    bool resid_valid = false;

    void reset() {
        cov = 100.0f;
        v_est = 0.0f;
        acc_last = 0.0f;
        wheel_stall[0] = false;
        wheel_stall[1] = false;
        last_wheel_spd[0] = 0.0f;
        last_wheel_spd[1] = 0.0f;
        stall_recover_cnt[0] = 0.0f;
        stall_recover_cnt[1] = 0.0f;
        last_leg_p[0] = 0.0f;
        last_leg_p[1] = 0.0f;
        off_gnd_timer[0] = 0.0f;
        off_gnd_timer[1] = 0.0f;
        impact_timer = 0.0f;
        resid_sq_sum_l = 0.0f;
        resid_sq_sum_r = 0.0f;
        cov_post = 100.0f;
        resid_valid = false;
    }
};

class SpeedKf {
public:
    /// 与 Helios spd_estimation 逐行对应。acc_x 单位 m/s^2（Helios 传 a_y）。
    float update(
        float wheel_spd_l, float wheel_spd_r, float yaw_rate_z, float acc_x, float leg_p_l,
        float leg_p_r, bool if_off_gnd_l, bool if_off_gnd_r, float leg_dphi0_l = 0.0f,
        float leg_dphi0_r = 0.0f, float dt = 0.001f);

    void reset() {
        state_.reset();
    }

    void reset_stall() {
        state_.wheel_stall[0] = false;
        state_.wheel_stall[1] = false;
        state_.stall_recover_cnt[0] = 0.0f;
        state_.stall_recover_cnt[1] = 0.0f;
    }

    [[nodiscard]] bool wheel_stall_left() const { return state_.wheel_stall[0]; }
    [[nodiscard]] bool wheel_stall_right() const { return state_.wheel_stall[1]; }

private:
    KfState state_;
};

} // namespace hcs_core::controller::balance
