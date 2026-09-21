#pragma once

// 从 Helios User_Code/app/chassis_app/balance/balance_nlmpc.hpp 移植（2026-09-18）。
// 接口、常量与内部布局保持不变；诊断字段用于阶段 3 的实时性测量。

#include <cstdint>

namespace hcs_core::controller::balance {

class BalanceNLMPC {
public:
    static constexpr int NX = 10;
    static constexpr int NU = 4;
    static constexpr int N = 10;

    struct Diagnostics {
        uint32_t solve_count = 0;
        uint32_t fallback_count = 0;
        uint32_t last_cycles = 0;
        uint32_t max_cycles = 0;
        uint32_t riccati_rebuild_count = 0;
        uint8_t admm_iterations = 0;
        float residual = 0.0f;
    };

    void Reset();
    bool Solve(
        float left_leg_length, float right_leg_length, const float reference_minus_state[NX],
        float theta_left, float theta_right, float output[NU]);
    bool GetDiscreteModel(const float*& ad, const float*& bd) const;
    void MarkFallback() {
        ++diagnostics_.fallback_count;
    }
    void SetCycles(uint32_t cycles);
    [[nodiscard]] const Diagnostics& diagnostics() const {
        return diagnostics_;
    }

private:
    bool BuildRiccati();
    bool BuildOnlineModel(float ll, float lr, float theta_left, float theta_right);
    void BackwardPass();
    void ForwardPass(const float error[NX]);

    float ad_[NX][NX]{};
    float bd_[NX][NU]{};
    float p_[N + 1][NX][NX]{};
    float gain_[N][NU][NX]{};
    float h_inv_[N][NU][NU]{};
    float linear_[N + 1][NX]{};
    float feedforward_[N][NU]{};
    float x_[N + 1][NX]{};
    float u_[N][NU]{};
    float projected_[N][NU]{};
    float dual_[N][NU]{};
    // Riccati workspace lives in BSS, not on the 1 kHz task stack.
    float ws_pb_[NX][NU]{};
    float ws_pa_[NX][NX]{};
    float ws_h_[NU][NU]{};
    float ws_btp_a_[NU][NX]{};
    float ws_atp_a_[NX][NX]{};
    Diagnostics diagnostics_{};
    bool model_valid_ = false;
    bool riccati_valid_ = false;
    uint8_t riccati_age_ = 0;
    float gain_ll_ = 0.0f;
    float gain_lr_ = 0.0f;
    float gain_theta_left_ = 0.0f;
    float gain_theta_right_ = 0.0f;
};

class BalanceRollMPC {
public:
    static constexpr int N = 10;
    void Reset();
    bool Solve(
        float roll, float roll_rate, float force_arm, float inertia, float force_limit,
        float& differential_force);

private:
    void Build(float force_arm, float inertia);
    bool initialized_ = false;
    float last_force_arm_ = 0.0f;
    float last_inertia_ = 0.0f;
    float gain_[N][2]{};
    float h_inv_[N]{};
    float projected_[N]{};
    float dual_[N]{};
};

} // namespace hcs_core::controller::balance
