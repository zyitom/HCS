#pragma once

// Helios module::user_lib 的 RampFunction / FirstOrderFilter / TD，
// 语义逐一对应；dt 由调用方显式传入（原实现按 DWT/固定 1 ms）。

#include <cmath>

#include "math.hpp"
#include "params.hpp"

namespace hcs_core::controller::balance {

class RampFunction {
public:
    [[nodiscard]] float operator()(const float target_value, const float ramp_rate) {
        float delta = target_value - output;
        if (std::fabs(delta) <= ramp_rate) {
            return target_value;
        }
        output += ramp_rate * (delta / std::fabs(delta));
        return output;
    }

    void reset(float value_now) {
        output = value_now;
    }

private:
    float output = 0.0f;
};

class FirstOrderFilter {
public:
    FirstOrderFilter(float rate_past_data, float rate_now_data)
        : rate_past(rate_past_data)
        , rate_now(rate_now_data) {}

    /// 与 Helios 一致：换系数时直接重建对象（TAKE_OFF 入口会重置滤波器）。
    void reset(float rate_past_data, float rate_now_data) {
        rate_past = rate_past_data;
        rate_now = rate_now_data;
        input = 0.0f;
        out = 0.0f;
    }

    [[nodiscard]] float operator()(const float new_input) {
        input = new_input;
        out = rate_past * out + rate_now * input;
        return out;
    }

private:
    float input = 0.0f;
    float out = 0.0f;
    float rate_past;
    float rate_now;
};

struct TdConfig {
    float r;  // 快速因子
    float h;  // 积分步长（秒）
    float h0; // 滤波因子
};

class Td {
public:
    /// delta_u 保留 Helios TD::update 的第三参（v2 中恒为默认 0）。
    float update(float input, const TdConfig& config, float delta_u = 0.0f) {
        x1 += delta_u;

        float d = config.r * config.h0 * config.h0;
        float a0 = config.h0 * x2;
        float y = x1 - input + a0;
        float a1 = std::sqrt(d * (d + 8.0f * std::fabs(y)));
        float a2 = a0 + ((y > 0) ? 1.0f : -1.0f) * (a1 - d) / 2.0f;

        float a;
        if (std::fabs(y) > d * config.h0)
            a = a2;
        else
            a = a0 + y;

        float fst;
        if (std::fabs(a) > d)
            fst = -config.r * ((a > 0) ? 1.0f : -1.0f);
        else
            fst = -config.r * a / d;

        x1 += x2 * config.h;
        x2 += fst * config.h;

        return x1;
    }

    void reset() {
        x1 = 0.0f;
        x2 = 0.0f;
    }

    [[nodiscard]] float get_derivative() const {
        return x2;
    }

private:
    float x1 = 0.0f;
    float x2 = 0.0f;
};

} // namespace hcs_core::controller::balance
