#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace hcs_core::controller::chassis::balance {

// ============================================================================
// 移植自 Helios 的两个标量小工具：跟踪微分器 TD（user_lib 的 module::TD）
// 与斜坡函数 RampFunction。数学不变，float → double，h 改为按拍传入的
// tick.dt_seconds()。
// ============================================================================

/// 二阶跟踪微分器（最速跟踪）。x1 跟踪输入、x2 是其微分估计。
class TrackingDifferentiator {
public:
    struct Parameters {
        double r = 25.0; ///< 快速因子
        double h0 = 0.0; ///< 滤波因子（Helios 取 10·h）
    };

    Parameters parameters{};

    void reset() {
        x1_ = 0.0;
        x2_ = 0.0;
    }

    double update(double input, double dt) {
        const double d = parameters.r * parameters.h0 * parameters.h0;
        const double a0 = parameters.h0 * x2_;
        const double y = x1_ - input + a0;
        const double a1 = std::sqrt(d * (d + 8.0 * std::fabs(y)));
        const double a2 = a0 + (y > 0 ? 1.0 : -1.0) * (a1 - d) / 2.0;

        double a;
        if (std::fabs(y) > d * parameters.h0)
            a = a2;
        else
            a = a0 + y;

        double fst;
        if (std::fabs(a) > d)
            fst = -parameters.r * (a > 0 ? 1.0 : -1.0);
        else
            fst = -parameters.r * a / d;

        x1_ += x2_ * dt;
        x2_ += fst * dt;
        return x1_;
    }

    [[nodiscard]] double derivative() const { return x2_; }

private:
    double x1_ = 0.0;
    double x2_ = 0.0;
};

/// 斜坡：|目标 − 当前| ≤ 速率时直接到位，否则每拍走一步。
/// Helios 把速率写成 m/tick 或 rad/tick，这里直接用秒制速率（m/s、rad/s）。
class Ramp {
public:
    double update(double target, double rate_per_second, double dt) {
        const double difference = target - value_;
        const double max_step = rate_per_second * dt;
        if (std::fabs(difference) <= max_step)
            value_ = target;
        else
            value_ += difference > 0 ? max_step : -max_step;
        return value_;
    }

    void reset(double value = 0.0) { value_ = value; }
    [[nodiscard]] double value() const { return value_; }

private:
    double value_ = 0.0;
};

/// 角度归一化到 (−π, π]。
inline double wrap_angle(double angle) {
    constexpr double two_pi = 2.0 * std::numbers::pi;
    angle = std::fmod(angle + std::numbers::pi, two_pi);
    if (angle < 0.0)
        angle += two_pi;
    return angle - std::numbers::pi;
}

} // namespace hcs_core::controller::chassis::balance
