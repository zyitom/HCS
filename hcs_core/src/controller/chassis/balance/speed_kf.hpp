#pragma once

#include <algorithm>
#include <cmath>

namespace hcs_core::controller::chassis::balance {

// ============================================================================
// 单标量速度卡尔曼滤波，逐行移植自 Helios
// User_Code/module/algorithm/balance_alg/balance_algorithm.cpp 的
// spd_estimation() / KF_s（float → double）。
//
// 状态：前向速度；观测量：左右轮速 ± yaw_rate·半轮距（腿摆动与伸缩的补偿
// 在上层做完再进来）。R 按工况自适应：转速越快越大、冲击/离地/堵转时拉满、
// 低速时收紧。原始实现里的 dt 来自 DWT 计数器，这里改成调用方按拍传
// tick.dt_seconds()——跳拍时行为与 Helios 的 0.001 兜底不同，是有意差异。
// ============================================================================

class SpeedKalmanFilter {
public:
    struct Parameters {
        double q = 1.3;
        double r_base = 400.0;
        double r_max = 1500.0;
        double r_high_speed_min = 5.0;
        double cov_min = 0.01;
        double cov_max = 100.0;
        double wheel_half_track = 0.2296;
        double impact_p_delta_thres = 30.0;
        double impact_cooldown_time = 0.15;   ///< 冲击冷却 [s]
        double off_gnd_debounce_time = 0.05;  ///< 离地防抖 [s]
        double air_decay_rate = 1.0;          ///< 空中速度指数衰减率
        double zupt_decay_rate = 10.5;        ///< 零速修正衰减率
        double stall_wheel_spd_thres = 0.1;
        double stall_wheel_drop_thres = 2.0;
        double stall_recover_time = 0.5;
        double resid_ema_alpha = 0.99;        ///< 残差方差 EMA（~100 ms 窗口 @1 kHz）
        double r_swing_gain = 80.0;           ///< 腿摆动 R 前馈增益
        double r_resid_min = 1.0;             ///< Sage-Husa R 估计下限
    };

    void reset() {
        cov_ = 100.0;
        velocity_estimate_ = 0.0;
        last_acceleration_ = 0.0;
        wheel_stall_[0] = wheel_stall_[1] = false;
        last_wheel_speed_[0] = last_wheel_speed_[1] = 0.0;
        stall_recover_time_[0] = stall_recover_time_[1] = 0.0;
        last_leg_pressure_[0] = last_leg_pressure_[1] = 0.0;
        off_ground_timer_[0] = off_ground_timer_[1] = 0.0;
        impact_timer_ = 0.0;
        residual_squared_sum_[0] = residual_squared_sum_[1] = 0.0;
        posterior_cov_ = 100.0;
        residual_valid_ = false;
    }

    /// @param leg_pressures 左右腿支撑力 P [N]（冲击检测用）
    /// @param legs_off_ground 上层给的粗判离地（Helios 传 P < 20）
    /// @param leg_angle_velocities 左右腿摆动角速率（R 前馈）
    double update(double wheel_speed_left, double wheel_speed_right, double yaw_rate_z,
                  double acceleration_x, const double leg_pressures[2],
                  const bool legs_off_ground[2], const double leg_angle_velocities[2],
                  double dt) {
        const double wheel_speed[2] = {wheel_speed_left, wheel_speed_right};
        for (int i = 0; i < 2; ++i) {
            const double wheel_drop = std::fabs(last_wheel_speed_[i]) - std::fabs(wheel_speed[i]);
            if (wheel_drop > parameters.stall_wheel_drop_thres
                && std::fabs(last_wheel_speed_[i]) > 0.5) {
                wheel_stall_[i] = true;
                stall_recover_time_[i] = 0.0;
            }
            if (wheel_stall_[i]) {
                stall_recover_time_[i] += dt;
                if (stall_recover_time_[i] > parameters.stall_recover_time
                    && std::fabs(wheel_speed[i]) > parameters.stall_wheel_spd_thres) {
                    wheel_stall_[i] = false;
                    stall_recover_time_[i] = 0.0;
                }
            }
            last_wheel_speed_[i] = wheel_speed[i];
        }

        bool truly_off_ground[2] = {false, false};
        for (int i = 0; i < 2; ++i) {
            off_ground_timer_[i] =
                legs_off_ground[i] ? off_ground_timer_[i] + dt : 0.0;
            truly_off_ground[i] = off_ground_timer_[i] > parameters.off_gnd_debounce_time;
        }

        for (int i = 0; i < 2; ++i) {
            const double pressure_delta =
                std::fabs(leg_pressures[i] - last_leg_pressure_[i]);
            last_leg_pressure_[i] = leg_pressures[i];
            if (pressure_delta > parameters.impact_p_delta_thres)
                impact_timer_ = parameters.impact_cooldown_time;
        }

        double current_acceleration = std::clamp(acceleration_x, -20.0, 20.0);
        if (impact_timer_ > 0.0) {
            impact_timer_ -= dt;
            current_acceleration = std::clamp(current_acceleration, -1.0, 1.0);
        }

        const double f = (last_acceleration_ + current_acceleration) * 0.5;
        double velocity_predict = velocity_estimate_ + f * dt;
        last_acceleration_ = current_acceleration;
        cov_ += parameters.q * dt;

        const double dynamic_r_base = parameters.r_base + 10.0 * std::fabs(yaw_rate_z);

        const double z_left = wheel_speed_left + yaw_rate_z * parameters.wheel_half_track;
        const double z_right = wheel_speed_right - yaw_rate_z * parameters.wheel_half_track;

        double r_resid_left = parameters.r_resid_min;
        double r_resid_right = parameters.r_resid_min;
        if (residual_valid_) {
            r_resid_left = residual_squared_sum_[0] + posterior_cov_;
            r_resid_right = residual_squared_sum_[1] + posterior_cov_;
        }
        r_resid_left = std::max(r_resid_left, parameters.r_resid_min);
        r_resid_right = std::max(r_resid_right, parameters.r_resid_min);

        const double r_swing_left = parameters.r_swing_gain * std::fabs(leg_angle_velocities[0]);
        const double r_swing_right = parameters.r_swing_gain * std::fabs(leg_angle_velocities[1]);

        double r_left = dynamic_r_base + r_resid_left + r_swing_left;
        double r_right = dynamic_r_base + r_resid_right + r_swing_right;

        if (truly_off_ground[0])
            r_left = parameters.r_max;
        if (truly_off_ground[1])
            r_right = parameters.r_max;
        if (wheel_stall_[0])
            r_left = parameters.r_max;
        if (wheel_stall_[1])
            r_right = parameters.r_max;
        if (impact_timer_ > 0.0) {
            r_left = parameters.r_max;
            r_right = parameters.r_max;
        }

        // 双轮量测都接近零时收紧 R，帮助零速钉住。
        const double z_average = 0.5 * (std::fabs(z_left) + std::fabs(z_right));
        if (z_average < 0.05) {
            r_left /= 5.0;
            r_right /= 5.0;
        }

        if (truly_off_ground[0] || truly_off_ground[1])
            velocity_predict *= std::exp(-parameters.air_decay_rate * dt);

        r_left = std::clamp(r_left, parameters.r_high_speed_min, parameters.r_max);
        r_right = std::clamp(r_right, parameters.r_high_speed_min, parameters.r_max);

        const double w_left = 1.0 / (r_left + 1e-6);
        const double w_right = 1.0 / (r_right + 1e-6);
        const double w_sum = w_left + w_right;

        double wheel_measure = velocity_predict;
        double r_dynamic = parameters.r_max;
        if (w_sum > 1e-5) {
            wheel_measure = (z_left * w_left + z_right * w_right) / w_sum;
            r_dynamic = 1.0 / w_sum;
        }

        const double gain = cov_ / (cov_ + r_dynamic);
        const double error = wheel_measure - velocity_predict;
        velocity_estimate_ = velocity_predict + gain * error;
        cov_ = (1.0 - gain) * cov_;
        cov_ = std::clamp(cov_, parameters.cov_min, parameters.cov_max);
        posterior_cov_ = cov_;

        const bool trust_left =
            !(truly_off_ground[0] || wheel_stall_[0] || impact_timer_ > 0.0);
        const bool trust_right =
            !(truly_off_ground[1] || wheel_stall_[1] || impact_timer_ > 0.0);
        if (trust_left) {
            const double residual = z_left - velocity_estimate_;
            residual_squared_sum_[0] = parameters.resid_ema_alpha * residual_squared_sum_[0]
                + (1.0 - parameters.resid_ema_alpha) * residual * residual;
        }
        if (trust_right) {
            const double residual = z_right - velocity_estimate_;
            residual_squared_sum_[1] = parameters.resid_ema_alpha * residual_squared_sum_[1]
                + (1.0 - parameters.resid_ema_alpha) * residual * residual;
        }
        residual_valid_ = true;

        // 零速修正（ZUPT）。
        const double wheel_average = 0.5 * (std::fabs(wheel_speed_left) + std::fabs(wheel_speed_right));
        if (wheel_average < 0.02 && std::fabs(acceleration_x) < 0.05) {
            velocity_estimate_ *= std::exp(-parameters.zupt_decay_rate * dt);
            if (std::fabs(velocity_estimate_) < 0.005)
                velocity_estimate_ = 0.0;
        } else if (wheel_average < 0.02 && std::fabs(velocity_estimate_) < 0.1) {
            velocity_estimate_ *= std::exp(-(parameters.zupt_decay_rate * 0.2) * dt);
        }

        return velocity_estimate_;
    }

    [[nodiscard]] bool wheel_stalled(int index) const { return wheel_stall_[index]; }

    Parameters parameters{};

private:
    double cov_ = 100.0;
    double velocity_estimate_ = 0.0;
    double last_acceleration_ = 0.0;
    bool wheel_stall_[2] = {false, false};
    double last_wheel_speed_[2] = {0.0, 0.0};
    double stall_recover_time_[2] = {0.0, 0.0};
    double last_leg_pressure_[2] = {0.0, 0.0};
    double off_ground_timer_[2] = {0.0, 0.0};
    double impact_timer_ = 0.0;
    double residual_squared_sum_[2] = {0.0, 0.0};
    double posterior_cov_ = 100.0;
    bool residual_valid_ = false;
};

} // namespace hcs_core::controller::chassis::balance
