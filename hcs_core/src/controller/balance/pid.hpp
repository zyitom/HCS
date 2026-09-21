#pragma once

// Helios module::PID（POSITION_PID + Improve 位掩码）的逐语义移植。
//
// 与 Helios 实现的两处显式差异：
//   1. dt 由调用方传入，不再用 DWT 计数；
//   2. "dt > 0.05 → 不更新任何状态直接返回 0" 的怪癖原样保留（它影响历史值，
//      对拍依赖这一点）。
//
// 其余（改进项应用顺序：梯形积分 → 变速积分 → 微分先行 → 微分滤波 → 积分限幅，
// 输出滤波在 iout 累加之后、限幅之前）与原实现逐行对应。

#include <cmath>
#include <cstdint>

#include "params.hpp"

namespace hcs_core::controller::balance {

class Pid {
public:
    explicit Pid(const PidConfig& config)
        : config_(config) {}

    void clear_i() {
        iout_ = 0.0f;
    }

    /// @param tick 当前拍号（用于复刻 Helios 的 DWT 语义：PID 不是每拍都被调用时，
    /// dt 按真实间隔放大；间隔 > 50 ms 时不更新任何状态直接返回 0）。
    [[nodiscard]] float update(float measurement, float setpoint, float dt, uint64_t tick) {
        const uint64_t gap = tick > last_tick_ ? tick - last_tick_ : 1;
        last_tick_ = tick;
        const float effective_dt = dt * static_cast<float>(gap);
        if (effective_dt > 0.05f) {
            return 0.0f;
        }
        dt = effective_dt;

        get_[kNow] = measurement;
        set_[kNow] = setpoint;
        err_[kNow] = setpoint - measurement;

        if ((config_.max_err > 0.0f && std::fabs(err_[kNow]) > config_.max_err)
            || (config_.deadband > 0.0f && std::fabs(err_[kNow]) < config_.deadband)) {
            return 0.0f;
        }

        pout_ = config_.kp * err_[kNow];
        iterm_ = config_.ki * err_[kNow] * dt;
        dout_ = config_.kd * (err_[kNow] - err_[kLast]) / dt;

        apply_improvements(dt);

        iout_ += iterm_;
        pos_out_ = pout_ + iout_ + dout_;

        if (config_.improve & kOutputFilter) {
            const float alpha = dt / (dt + config_.output_lpf);
            const float dalpha = config_.output_lpf / (dt + config_.output_lpf);
            pos_out_ = pos_out_ * alpha + last_pos_out_ * dalpha;
        }

        pos_out_ = abs_clip(pos_out_, config_.max_output);

        last_pos_out_ = pos_out_;
        last_dout_ = dout_;
        last_iterm_ = iterm_;

        err_[kLlast] = err_[kLast];
        err_[kLast] = err_[kNow];
        get_[kLlast] = get_[kLast];
        get_[kLast] = get_[kNow];
        set_[kLlast] = set_[kLast];
        set_[kLast] = set_[kNow];

        // 与 Helios 相同的 NaN 兜底：不清 iout 之外的历史，只清这三项。
        if (std::isnan(iout_)) iout_ = 0.0f;
        if (std::isnan(pout_)) pout_ = 0.0f;
        if (std::isnan(dout_)) dout_ = 0.0f;

        return pos_out_;
    }

    [[nodiscard]] const PidConfig& config() const { return config_; }

private:
    static constexpr uint8_t kIntegralLimit = 0x01;
    static constexpr uint8_t kDifferentialForward = 0x02;
    static constexpr uint8_t kTrapezoidIntegral = 0x04;
    static constexpr uint8_t kChangingIntegration = 0x08;
    static constexpr uint8_t kDerivativeFilter = 0x10;
    static constexpr uint8_t kOutputFilter = 0x20;

    static constexpr int kLlast = 0;
    static constexpr int kLast = 1;
    static constexpr int kNow = 2;

    void apply_improvements(float dt) {
        if (config_.improve & kTrapezoidIntegral) {
            iterm_ = ((err_[kNow] + err_[kLast]) * 0.5f) * dt;
        }
        if (config_.improve & kChangingIntegration) {
            const float cur_err = std::fabs(err_[kNow]);
            if (cur_err > config_.integral_max) {
                iterm_ = 0.0f;
            } else if (cur_err > config_.integral_min) {
                iterm_ *= (config_.integral_max - cur_err)
                        / (config_.integral_max - config_.integral_min);
            }
        }
        if (config_.improve & kDifferentialForward) {
            dout_ = config_.kd * (get_[kLast] - get_[kNow]) / dt;
        }
        if (config_.improve & kDerivativeFilter) {
            const float alpha = dt / (dt + config_.d_lpf);
            const float dalpha = config_.d_lpf / (dt + config_.d_lpf);
            dout_ = dout_ * alpha + last_dout_ * dalpha;
        }
        if (config_.improve & kIntegralLimit) {
            iout_ = abs_clip(iout_, config_.integral_limit);
        }
    }

    PidConfig config_;

    float set_[3]{};
    float get_[3]{};
    float err_[3]{};
    float pout_{};
    float iterm_{};
    float iout_{};
    float dout_{};
    float pos_out_{};
    float last_pos_out_{};
    float last_dout_{};
    float last_iterm_{};
    uint64_t last_tick_ = 0;
};

} // namespace hcs_core::controller::balance
