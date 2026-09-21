#include "nlmpc.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace hcs_core::controller::balance {

// 从 Helios balance_nlmpc.cpp 移植（2026-09-18）：arm_math → std，命名空间更新；
// 算法、常量与浮点运算顺序保持不变（内部 DT 固定 1 ms，与 1 kHz 更新绑定）。
namespace {

constexpr float DT = 0.001f;
constexpr float RHO = 1.0f;
constexpr int ADMM_ITERATIONS = 6;
constexpr float Q[BalanceNLMPC::NX] =
    {20.0f, 5.0f, 25.0f, 6.0f, 300.0f, 12.0f,
     300.0f, 12.0f, 200.0f, 10.0f};
constexpr float R[BalanceNLMPC::NU] = {1.0f, 1.0f, 0.25f, 0.25f};
constexpr float LIMIT[BalanceNLMPC::NU] = {5.0f, 5.0f, 40.0f, 40.0f};
constexpr int DYNAMIC_ROWS[5] = {1, 3, 5, 7, 9};
constexpr float TERMINAL_WEIGHT = 50.0f;

inline bool finite_matrix(const float *data, int count)
{
    for (int i = 0; i < count; ++i)
        if (!std::isfinite(data[i])) return false;
    return true;
}

bool invert_spd_4x4(const float input[4][4], float inverse[4][4])
{
    float lower[4][4]{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j <= i; ++j) {
            float value = input[i][j];
            for (int k = 0; k < j; ++k) value -= lower[i][k] * lower[j][k];
            if (i == j) {
                if (!std::isfinite(value) || value <= 1e-7f) return false;
                lower[i][j] = std::sqrt(value);
            } else {
                lower[i][j] = value / lower[j][j];
            }
        }
    }

    for (int column = 0; column < 4; ++column) {
        float y[4]{};
        for (int i = 0; i < 4; ++i) {
            float value = i == column ? 1.0f : 0.0f;
            for (int k = 0; k < i; ++k) value -= lower[i][k] * y[k];
            y[i] = value / lower[i][i];
        }
        for (int i = 3; i >= 0; --i) {
            float value = y[i];
            for (int k = i + 1; k < 4; ++k) value -= lower[k][i] * inverse[k][column];
            inverse[i][column] = value / lower[i][i];
        }
    }
    return true;
}

} // namespace {（文件内私有）

void BalanceNLMPC::Reset()
{
    std::memset(projected_, 0, sizeof(projected_));
    std::memset(dual_, 0, sizeof(dual_));
    diagnostics_.residual = 0.0f;
    riccati_valid_ = false;
    riccati_age_ = 0;
}

void BalanceNLMPC::SetCycles(uint32_t cycles)
{
    diagnostics_.last_cycles = cycles;
    diagnostics_.max_cycles = std::max(diagnostics_.max_cycles, cycles);
}

bool BalanceNLMPC::BuildOnlineModel(float ll, float lr, float theta_left, float theta_right)
{
    model_valid_ = false;
    constexpr float rw = 0.06f;
    constexpr float half_track = 0.2296f;
    constexpr float mw = 0.25867f;
    constexpr float ml = 2.6f;
    constexpr float mb = 18.0f;
    constexpr float iz = 0.2657f;
    constexpr float iw = 4.98e-4f;
    constexpr float ib = 0.353848333f;
    constexpr float gravity = 9.8f;

    if (!std::isfinite(ll) || !std::isfinite(lr) || ll < 0.14f || ll > 0.42f ||
        lr < 0.14f || lr > 0.42f) return false;
    theta_left = std::clamp(theta_left, -1.22173f, 1.22173f);
    theta_right = std::clamp(theta_right, -1.22173f, 1.22173f);
    const float cl = std::cos(theta_left);
    const float sl = std::sin(theta_left);
    const float cr = std::cos(theta_right);
    const float sr = std::sin(theta_right);
    const float ll2 = ll * ll;
    const float lr2 = lr * lr;
    const float inv_4_track2 = 0.25f / (half_track * half_track);

    const float com_x_l = 0.2208f * ll2 - 0.0553f * ll - 0.0301f;
    const float com_x_r = 0.2208f * lr2 - 0.0553f * lr - 0.0301f;
    const float com_y_l = 0.2716f * ll - 0.001f;
    const float com_y_r = 0.2716f * lr - 0.001f;
    const float lwl = ll - com_y_l;
    const float lwr = lr - com_y_r;
    const float leg_proj_l = lwl * cl - com_x_l * sl;
    const float leg_proj_r = lwr * cr - com_x_r * sr;
    const float ill = 0.1158f * ll + 0.0143f;
    const float ilr = 0.1158f * lr + 0.0143f;

    const float wheel_diag = iw + iz * rw * rw * inv_4_track2
                           + 0.25f * rw * rw * mb + rw * rw * (ml + mw);
    const float wheel_cross = -iz * rw * rw * inv_4_track2 + 0.25f * rw * rw * mb;
    const float yaw_wl = iz * rw * ll * cl * inv_4_track2;
    const float yaw_wr = iz * rw * lr * cr * inv_4_track2;
    float mass[4][4]{};
    mass[0][0] = mass[1][1] = wheel_diag;
    mass[0][1] = mass[1][0] = wheel_cross;
    mass[0][2] = mass[2][0] = yaw_wl + 0.25f * rw * ll * mb * cl + rw * leg_proj_l * ml;
    mass[1][2] = mass[2][1] = -yaw_wl + 0.25f * rw * ll * mb * cl;
    mass[0][3] = mass[3][0] = -yaw_wr + 0.25f * rw * lr * mb * cr;
    mass[1][3] = mass[3][1] = yaw_wr + 0.25f * rw * lr * mb * cr + rw * leg_proj_r * ml;
    mass[2][2] = ill + iz * ll2 * cl * cl * inv_4_track2
                       + 0.25f * ll2 * mb + (lwl * lwl + com_x_l * com_x_l) * ml;
    mass[3][3] = ilr + iz * lr2 * cr * cr * inv_4_track2
                       + 0.25f * lr2 * mb + (lwr * lwr + com_x_r * com_x_r) * ml;
    mass[2][3] = mass[3][2] = -iz * ll * lr * cl * cr * inv_4_track2
                                   + 0.25f * ll * lr * mb;

    const float inv_2_track = 0.5f / half_track;
    const float transform[4][4] = {
        {0.5f * rw, 0.5f * rw, 0.0f, 0.0f},
        {-rw * inv_2_track, rw * inv_2_track, -ll * cl * inv_2_track, lr * cr * inv_2_track},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}};
    // M is symmetric positive definite. LDLT plus four triangular solves is
    // cheaper than a generic inverse and directly yields C=S*M^-1.
    float l[4][4]{};
    float d[4]{};
    for (int i = 0; i < 4; ++i) {
        l[i][i] = 1.0f;
        for (int j = 0; j <= i; ++j) {
            float value = mass[i][j];
            for (int k = 0; k < j; ++k) value -= l[i][k] * d[k] * l[j][k];
            if (i == j) {
                if (!std::isfinite(value) || value < 1e-7f) return false;
                d[i] = value;
            } else {
                l[i][j] = value / d[j];
            }
        }
    }
    float c[4][4]{};
    for (int rhs = 0; rhs < 4; ++rhs) {
        float y[4]{}, z[4]{}, solution[4]{};
        for (int i = 0; i < 4; ++i) {
            y[i] = transform[rhs][i];
            for (int k = 0; k < i; ++k) y[i] -= l[i][k] * y[k];
            z[i] = y[i] / d[i];
        }
        for (int i = 3; i >= 0; --i) {
            solution[i] = z[i];
            for (int k = i + 1; k < 4; ++k) solution[i] -= l[k][i] * solution[k];
            c[rhs][i] = solution[i];
        }
    }

    const float stiffness_l = -gravity * (0.5f * ll * mb * cl + leg_proj_l * ml);
    const float stiffness_r = -gravity * (0.5f * lr * mb * cr + leg_proj_r * ml);
    constexpr float e[4][4] = {
        {1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f},
        {-1.0f, 0.0f, 1.0f, 0.0f}, {0.0f, -1.0f, 0.0f, 1.0f}};

    std::memset(ad_, 0, sizeof(ad_));
    std::memset(bd_, 0, sizeof(bd_));
    for (int i = 0; i < NX; ++i) ad_[i][i] = 1.0f;
    for (int position = 0; position < 5; ++position) ad_[2 * position][2 * position + 1] = DT;
    for (int q = 0; q < 4; ++q) {
        const int row = 2 * q + 1;
        ad_[row][4] = -DT * c[q][2] * stiffness_l;
        ad_[row][6] = -DT * c[q][3] * stiffness_r;
        for (int input = 0; input < NU; ++input)
            for (int k = 0; k < 4; ++k) bd_[row][input] += DT * c[q][k] * e[k][input];
    }
    bd_[9][2] = -DT / ib;
    bd_[9][3] = -DT / ib;
    model_valid_ = finite_matrix(&ad_[0][0], NX * NX) && finite_matrix(&bd_[0][0], NX * NU);
    return model_valid_;
}

bool BalanceNLMPC::GetDiscreteModel(const float *&ad, const float *&bd) const
{
    if (!model_valid_) return false;
    ad = &ad_[0][0];
    bd = &bd_[0][0];
    return true;
}

bool BalanceNLMPC::BuildRiccati()
{
    std::memset(p_[N], 0, sizeof(p_[N]));
    for (int i = 0; i < NX; ++i) p_[N][i][i] = TERMINAL_WEIGHT * Q[i];

    for (int stage = N - 1; stage >= 0; --stage) {
        const float (*next_p)[NX] = p_[stage + 1];

        // A and B only have dynamics in five velocity rows. Expanding these
        // products avoids the generic dense kernels used by the old path.
        for (int i = 0; i < NX; ++i) {
            for (int input = 0; input < NU; ++input) {
                float value = 0.0f;
                for (int row : DYNAMIC_ROWS) value += next_p[i][row] * bd_[row][input];
                ws_pb_[i][input] = value;
            }

            std::memcpy(ws_pa_[i], next_p[i], sizeof(ws_pa_[i]));
            for (int position = 0; position < 5; ++position)
                ws_pa_[i][2 * position + 1] += DT * next_p[i][2 * position];
            for (int q = 0; q < 4; ++q) {
                const int row = 2 * q + 1;
                ws_pa_[i][4] += next_p[i][row] * ad_[row][4];
                ws_pa_[i][6] += next_p[i][row] * ad_[row][6];
            }
        }

        for (int i = 0; i < NU; ++i) {
            for (int j = 0; j < NU; ++j) {
                float value = i == j ? R[i] + RHO : 0.0f;
                for (int row : DYNAMIC_ROWS) value += bd_[row][i] * ws_pb_[row][j];
                ws_h_[i][j] = value;
            }
            for (int j = 0; j < NX; ++j) {
                float value = 0.0f;
                for (int row : DYNAMIC_ROWS) value += bd_[row][i] * ws_pa_[row][j];
                ws_btp_a_[i][j] = value;
            }
        }
        if (!invert_spd_4x4(ws_h_, h_inv_[stage])) return false;
        for (int i = 0; i < NU; ++i)
            for (int j = 0; j < NX; ++j) {
                float value = 0.0f;
                for (int k = 0; k < NU; ++k) value += h_inv_[stage][i][k] * ws_btp_a_[k][j];
                gain_[stage][i][j] = value;
            }

        for (int i = 0; i < NX; ++i) {
            std::memcpy(ws_atp_a_[i], ws_pa_[i], sizeof(ws_atp_a_[i]));
        }
        for (int position = 0; position < 5; ++position) {
            const int state = 2 * position;
            const int velocity = state + 1;
            for (int j = 0; j < NX; ++j) ws_atp_a_[velocity][j] += DT * ws_pa_[state][j];
        }
        for (int q = 0; q < 4; ++q) {
            const int row = 2 * q + 1;
            for (int j = 0; j < NX; ++j) {
                ws_atp_a_[4][j] += ad_[row][4] * ws_pa_[row][j];
                ws_atp_a_[6][j] += ad_[row][6] * ws_pa_[row][j];
            }
        }
        for (int i = 0; i < NX; ++i)
            for (int j = 0; j <= i; ++j) {
                float value = ws_atp_a_[i][j] + (i == j ? Q[i] : 0.0f);
                for (int input = 0; input < NU; ++input)
                    value -= ws_btp_a_[input][i] * gain_[stage][input][j];
                p_[stage][i][j] = value;
                p_[stage][j][i] = value;
            }
    }
    return finite_matrix(&gain_[0][0][0], N * NU * NX) &&
           finite_matrix(&h_inv_[0][0][0], N * NU * NU);
}

void BalanceNLMPC::BackwardPass()
{
    std::memset(linear_[N], 0, sizeof(linear_[N]));
    for (int stage = N - 1; stage >= 0; --stage) {
        float rhs[NU]{};
        for (int i = 0; i < NU; ++i) {
            rhs[i] = RHO * (dual_[stage][i] - projected_[stage][i]);
            for (int row : DYNAMIC_ROWS) rhs[i] += bd_[row][i] * linear_[stage + 1][row];
        }
        for (int i = 0; i < NU; ++i) {
            float value = 0.0f;
            for (int j = 0; j < NU; ++j) value += h_inv_[stage][i][j] * rhs[j];
            feedforward_[stage][i] = value;
        }
        const float *next = linear_[stage + 1];
        std::memcpy(linear_[stage], next, sizeof(linear_[stage]));
        for (int position = 0; position < 5; ++position)
            linear_[stage][2 * position + 1] += DT * next[2 * position];
        for (int q = 0; q < 4; ++q) {
            const int row = 2 * q + 1;
            linear_[stage][4] += ad_[row][4] * next[row];
            linear_[stage][6] += ad_[row][6] * next[row];
        }
        for (int i = 0; i < NX; ++i)
            for (int j = 0; j < NU; ++j) linear_[stage][i] -= gain_[stage][j][i] * rhs[j];
    }
}

void BalanceNLMPC::ForwardPass(const float error[NX])
{
    std::memcpy(x_[0], error, sizeof(x_[0]));
    for (int stage = 0; stage < N; ++stage) {
        for (int i = 0; i < NU; ++i) {
            u_[stage][i] = -feedforward_[stage][i];
            for (int j = 0; j < NX; ++j) u_[stage][i] -= gain_[stage][i][j] * x_[stage][j];
        }
        std::memcpy(x_[stage + 1], x_[stage], sizeof(x_[stage + 1]));
        for (int position = 0; position < 5; ++position)
            x_[stage + 1][2 * position] += DT * x_[stage][2 * position + 1];
        for (int q = 0; q < 4; ++q) {
            const int row = 2 * q + 1;
            x_[stage + 1][row] += ad_[row][4] * x_[stage][4]
                                      + ad_[row][6] * x_[stage][6];
            for (int input = 0; input < NU; ++input)
                x_[stage + 1][row] += bd_[row][input] * u_[stage][input];
        }
    }
}

bool BalanceNLMPC::Solve(float ll, float lr, const float reference_minus_state[NX],
                         float theta_left, float theta_right, float output[NU])
{
    ++diagnostics_.solve_count;
    if (!finite_matrix(reference_minus_state, NX) ||
        !BuildOnlineModel(ll, lr, theta_left, theta_right)) return false;

    for (int stage = 0; stage < N - 1; ++stage)
        for (int i = 0; i < NU; ++i) {
            projected_[stage][i] = projected_[stage + 1][i];
            dual_[stage][i] = dual_[stage + 1][i];
        }
    constexpr uint8_t max_riccati_age = 5;
    const bool model_changed = std::fabs(ll - gain_ll_) > 0.002f ||
                               std::fabs(lr - gain_lr_) > 0.002f ||
                               std::fabs(theta_left - gain_theta_left_) > 0.01f ||
                               std::fabs(theta_right - gain_theta_right_) > 0.01f;
    if (!riccati_valid_ || model_changed || riccati_age_ >= max_riccati_age) {
        if (!BuildRiccati()) { riccati_valid_ = false; return false; }
        gain_ll_ = ll;
        gain_lr_ = lr;
        gain_theta_left_ = theta_left;
        gain_theta_right_ = theta_right;
        riccati_age_ = 0;
        riccati_valid_ = true;
        ++diagnostics_.riccati_rebuild_count;
    } else {
        ++riccati_age_;
    }

    float error[NX];
    for (int i = 0; i < NX; ++i) error[i] = -reference_minus_state[i];
    error[0] = reference_minus_state[0];
    error[1] = reference_minus_state[1];
    diagnostics_.admm_iterations = ADMM_ITERATIONS;
    for (int iteration = 0; iteration < ADMM_ITERATIONS; ++iteration) {
        BackwardPass();
        ForwardPass(error);
        diagnostics_.residual = 0.0f;
        for (int stage = 0; stage < N; ++stage) for (int i = 0; i < NU; ++i) {
            const float previous = projected_[stage][i];
            projected_[stage][i] = std::clamp(u_[stage][i] + dual_[stage][i], -LIMIT[i], LIMIT[i]);
            dual_[stage][i] += u_[stage][i] - projected_[stage][i];
            diagnostics_.residual = std::max(diagnostics_.residual,
                                             std::fabs(projected_[stage][i] - previous));
        }
        if (iteration >= 2 && diagnostics_.residual < 1e-3f) {
            diagnostics_.admm_iterations = static_cast<uint8_t>(iteration + 1);
            break;
        }
    }
    for (int i = 0; i < NU; ++i) output[i] = projected_[0][i];
    return finite_matrix(output, NU);
}

void BalanceRollMPC::Reset()
{
    std::memset(projected_, 0, sizeof(projected_));
    std::memset(dual_, 0, sizeof(dual_));
}

void BalanceRollMPC::Build(float force_arm, float inertia)
{
    const float b = 2.0f * force_arm / inertia;
    const float a[2][2] = {{1.0f, DT}, {0.0f, 1.0f}};
    const float bd[2] = {0.5f * DT * DT * b, DT * b};
    constexpr float q[2] = {20000.0f, 30.0f};
    constexpr float r = 0.0005f;
    float p[2][2] = {{5.0f * q[0], 0.0f}, {0.0f, 5.0f * q[1]}};
    for (int stage = N - 1; stage >= 0; --stage) {
        float pa[2][2]{};
        for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k) pa[i][j] += p[i][k] * a[k][j];
        const float h = r + RHO + bd[0] * (p[0][0] * bd[0] + p[0][1] * bd[1])
                                  + bd[1] * (p[1][0] * bd[0] + p[1][1] * bd[1]);
        h_inv_[stage] = 1.0f / h;
        for (int j = 0; j < 2; ++j)
            gain_[stage][j] = h_inv_[stage] * (bd[0] * pa[0][j] + bd[1] * pa[1][j]);
        float next[2][2]{};
        for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) next[i][j] += a[k][i] * pa[k][j];
            next[i][j] -= (bd[0] * pa[0][i] + bd[1] * pa[1][i]) * gain_[stage][j];
            if (i == j) next[i][j] += q[i];
        }
        std::memcpy(p, next, sizeof(p));
    }
    last_force_arm_ = force_arm;
    last_inertia_ = inertia;
    initialized_ = true;
}

bool BalanceRollMPC::Solve(float roll, float roll_rate, float force_arm, float inertia,
                           float force_limit, float &differential_force)
{
    if (!std::isfinite(roll) || !std::isfinite(roll_rate) ||
        force_arm <= 0.0f || inertia <= 0.0f || force_limit <= 0.0f) return false;
    if (!initialized_ || std::fabs(force_arm - last_force_arm_) > 1e-5f ||
        std::fabs(inertia - last_inertia_) > 1e-5f) Build(force_arm, inertia);
    for (int i = 0; i < N - 1; ++i) {
        projected_[i] = projected_[i + 1];
        dual_[i] = dual_[i + 1];
    }
    const float b = 2.0f * force_arm / inertia;
    const float bd[2] = {0.5f * DT * DT * b, DT * b};
    for (int iteration = 0; iteration < ADMM_ITERATIONS; ++iteration) {
        float linear[N + 1][2]{};
        float feedforward[N]{};
        for (int stage = N - 1; stage >= 0; --stage) {
            const float rhs = RHO * (dual_[stage] - projected_[stage])
                            + bd[0] * linear[stage + 1][0] + bd[1] * linear[stage + 1][1];
            feedforward[stage] = h_inv_[stage] * rhs;
            linear[stage][0] = linear[stage + 1][0] - gain_[stage][0] * rhs;
            linear[stage][1] = DT * linear[stage + 1][0] + linear[stage + 1][1]
                             - gain_[stage][1] * rhs;
        }
        float x[2] = {roll, roll_rate};
        for (int stage = 0; stage < N; ++stage) {
            const float u = -gain_[stage][0] * x[0] - gain_[stage][1] * x[1] - feedforward[stage];
            projected_[stage] = std::clamp(u + dual_[stage], -force_limit, force_limit);
            dual_[stage] += u - projected_[stage];
            x[0] += DT * x[1] + bd[0] * u;
            x[1] += bd[1] * u;
        }
    }
    differential_force = projected_[0];
    return std::isfinite(differential_force);
}

} // namespace hcs_core::controller::balance

