#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace hcs_core::controller::chassis::balance {

// ============================================================================
// 平衡整车的 NLMPC（主）与滚转差动 MPC，逐行移植自 Helios
// User_Code/app/chassis_app/balance/balance_nlmpc.{hpp,cpp}
//（arm_math 的三角函数 → std::，float → double，其余结构不变）。
//
// NLMPC：4 自由度（v、ω_z、θ_L、θ_R）在线线性化 + 10 步 ADMM 有限时域，
// 状态/控制维数与 Helios 相同；Riccati 表按模型变化（|ΔLL| > 2 mm 或
// |Δθ| > 10 mrad）或 5 拍年龄重建。求解失败或模型无效返回 false，
// 调用方回退 lqr_gain_table 的 LQR 路径。
//
// RollMPC：单输入（左右腿差动支撑力）10 步 ADMM，比 NLMPC 简单得多，
// 用于小陀螺时的滚转镇定（失败回退 stab_roll 积分）。
// ============================================================================

class BalanceNlmpc {
public:
    static constexpr int kStateSize = 10;
    static constexpr int kControlSize = 4;
    static constexpr int kHorizon = 10;

    void reset() {
        std::memset(projected_, 0, sizeof(projected_));
        std::memset(dual_, 0, sizeof(dual_));
        residual_ = 0.0;
        riccati_valid_ = false;
        riccati_age_ = 0;
    }

    /// @param left_length/right_length 左右腿长 [m]，决定在线模型。
    /// @param reference_minus_state 参考量减状态量（Helios 的 Status_vector）。
    /// @param theta_left/right 左右腿对铅垂的摆角 [rad]。
    /// @param output 四通道控制 [N·m]（左轮、右轮、左髋、右髋）。
    /// @return 求解成功且输出有限。
    bool solve(
        double left_length, double right_length, const double reference_minus_state[kStateSize],
        double theta_left, double theta_right, double output[kControlSize]) {
        if (!is_finite(reference_minus_state, kStateSize)
            || !build_online_model(left_length, right_length, theta_left, theta_right))
            return false;

        for (int stage = 0; stage < kHorizon - 1; ++stage)
            for (int i = 0; i < kControlSize; ++i) {
                projected_[stage][i] = projected_[stage + 1][i];
                dual_[stage][i] = dual_[stage + 1][i];
            }

        constexpr int kMaxRiccatiAge = 5;
        const bool model_changed = std::fabs(left_length - gain_left_length_) > 0.002
            || std::fabs(right_length - gain_right_length_) > 0.002
            || std::fabs(theta_left - gain_theta_left_) > 0.01
            || std::fabs(theta_right - gain_theta_right_) > 0.01;
        if (!riccati_valid_ || model_changed || riccati_age_ >= kMaxRiccatiAge) {
            if (!build_riccati()) {
                riccati_valid_ = false;
                return false;
            }
            gain_left_length_ = left_length;
            gain_right_length_ = right_length;
            gain_theta_left_ = theta_left;
            gain_theta_right_ = theta_right;
            riccati_age_ = 0;
            riccati_valid_ = true;
        } else {
            ++riccati_age_;
        }

        double error[kStateSize];
        for (int i = 0; i < kStateSize; ++i)
            error[i] = -reference_minus_state[i];
        error[0] = reference_minus_state[0]; // 位置参考按原符号进预测
        error[1] = reference_minus_state[1];

        for (int iteration = 0; iteration < kAdmmIterations; ++iteration) {
            backward_pass();
            forward_pass(error);
            residual_ = 0.0;
            for (int stage = 0; stage < kHorizon; ++stage)
                for (int i = 0; i < kControlSize; ++i) {
                    const double previous = projected_[stage][i];
                    projected_[stage][i] = std::clamp(
                        u_[stage][i] + dual_[stage][i], -kLimit[i], kLimit[i]);
                    dual_[stage][i] += u_[stage][i] - projected_[stage][i];
                    residual_ = std::max(residual_, std::fabs(projected_[stage][i] - previous));
                }
            if (iteration >= 2 && residual_ < 1e-3)
                break;
        }
        for (int i = 0; i < kControlSize; ++i)
            output[i] = projected_[0][i];
        return is_finite(output, kControlSize);
    }

    [[nodiscard]] double residual() const { return residual_; }

private:
    static constexpr double kDt = 0.001;
    static constexpr double kRho = 1.0;
    static constexpr int kAdmmIterations = 6;
    static constexpr double kQ[kStateSize] = {20.0, 5.0, 25.0, 6.0, 300.0, 12.0,
                                              300.0, 12.0, 200.0, 10.0};
    static constexpr double kR[kControlSize] = {1.0, 1.0, 0.25, 0.25};
    static constexpr double kLimit[kControlSize] = {5.0, 5.0, 40.0, 40.0};
    static constexpr int kDynamicRows[5] = {1, 3, 5, 7, 9};
    static constexpr double kTerminalWeight = 50.0;

    static bool is_finite(const double* data, int count) {
        for (int i = 0; i < count; ++i)
            if (!std::isfinite(data[i]))
                return false;
        return true;
    }

    static bool invert_spd_4x4(const double input[4][4], double inverse[4][4]) {
        double lower[4][4]{};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j <= i; ++j) {
                double value = input[i][j];
                for (int k = 0; k < j; ++k)
                    value -= lower[i][k] * lower[j][k];
                if (i == j) {
                    if (!std::isfinite(value) || value <= 1e-7)
                        return false;
                    lower[i][j] = std::sqrt(value);
                } else {
                    lower[i][j] = value / lower[j][j];
                }
            }
        for (int column = 0; column < 4; ++column) {
            double y[4]{};
            for (int i = 0; i < 4; ++i) {
                double value = i == column ? 1.0 : 0.0;
                for (int k = 0; k < i; ++k)
                    value -= lower[i][k] * y[k];
                y[i] = value / lower[i][i];
            }
            for (int i = 3; i >= 0; --i) {
                double value = y[i];
                for (int k = i + 1; k < 4; ++k)
                    value -= lower[k][i] * inverse[k][column];
                inverse[i][column] = value / lower[i][i];
            }
        }
        return true;
    }

    bool build_online_model(double ll, double lr, double theta_left, double theta_right) {
        model_valid_ = false;
        constexpr double rw = 0.06;
        constexpr double half_track = 0.2296;
        constexpr double mw = 0.25867;
        constexpr double ml = 2.6;
        constexpr double mb = 18.0;
        constexpr double iz = 0.2657;
        constexpr double iw = 4.98e-4;
        constexpr double ib = 0.353848333;
        constexpr double gravity = 9.8;

        if (!std::isfinite(ll) || !std::isfinite(lr) || ll < 0.14 || ll > 0.42 || lr < 0.14
            || lr > 0.42)
            return false;
        theta_left = std::clamp(theta_left, -1.22173, 1.22173);
        theta_right = std::clamp(theta_right, -1.22173, 1.22173);
        const double cl = std::cos(theta_left);
        const double sl = std::sin(theta_left);
        const double cr = std::cos(theta_right);
        const double sr = std::sin(theta_right);
        const double ll2 = ll * ll;
        const double lr2 = lr * lr;
        constexpr double inv_4_track2 = 0.25 / (half_track * half_track);

        const double com_x_l = 0.2208 * ll2 - 0.0553 * ll - 0.0301;
        const double com_x_r = 0.2208 * lr2 - 0.0553 * lr - 0.0301;
        const double com_y_l = 0.2716 * ll - 0.001;
        const double com_y_r = 0.2716 * lr - 0.001;
        const double lwl = ll - com_y_l;
        const double lwr = lr - com_y_r;
        const double leg_proj_l = lwl * cl - com_x_l * sl;
        const double leg_proj_r = lwr * cr - com_x_r * sr;
        const double ill = 0.1158 * ll + 0.0143;
        const double ilr = 0.1158 * lr + 0.0143;

        const double wheel_diag = iw + iz * rw * rw * inv_4_track2 + 0.25 * rw * rw * mb
                                + rw * rw * (ml + mw);
        const double wheel_cross = -iz * rw * rw * inv_4_track2 + 0.25 * rw * rw * mb;
        const double yaw_wl = iz * rw * ll * cl * inv_4_track2;
        const double yaw_wr = iz * rw * lr * cr * inv_4_track2;
        double mass[4][4]{};
        mass[0][0] = mass[1][1] = wheel_diag;
        mass[0][1] = mass[1][0] = wheel_cross;
        mass[0][2] = mass[2][0] = yaw_wl + 0.25 * rw * ll * mb * cl + rw * leg_proj_l * ml;
        mass[1][2] = mass[2][1] = -yaw_wl + 0.25 * rw * ll * mb * cl;
        mass[0][3] = mass[3][0] = -yaw_wr + 0.25 * rw * lr * mb * cr;
        mass[1][3] = mass[3][1] = yaw_wr + 0.25 * rw * lr * mb * cr + rw * leg_proj_r * ml;
        mass[2][2] = ill + iz * ll2 * cl * cl * inv_4_track2 + 0.25 * ll2 * mb
                   + (lwl * lwl + com_x_l * com_x_l) * ml;
        mass[3][3] = ilr + iz * lr2 * cr * cr * inv_4_track2 + 0.25 * lr2 * mb
                   + (lwr * lwr + com_x_r * com_x_r) * ml;
        mass[2][3] = mass[3][2] = -iz * ll * lr * cl * cr * inv_4_track2 + 0.25 * ll * lr * mb;

        constexpr double inv_2_track = 0.5 / half_track;
        const double transform[4][4] = {
            {0.5 * rw, 0.5 * rw, 0.0, 0.0},
            {-rw * inv_2_track, rw * inv_2_track, -ll * cl * inv_2_track, lr * cr * inv_2_track},
            {0.0, 0.0, 1.0, 0.0},
            {0.0, 0.0, 0.0, 1.0}};
        // M 对称正定：LDLᵀ 分解 + 四次三角回代得到 C = S·M⁻¹。
        double l[4][4]{};
        double d[4]{};
        for (int i = 0; i < 4; ++i) {
            l[i][i] = 1.0;
            for (int j = 0; j <= i; ++j) {
                double value = mass[i][j];
                for (int k = 0; k < j; ++k)
                    value -= l[i][k] * d[k] * l[j][k];
                if (i == j) {
                    if (!std::isfinite(value) || value < 1e-7)
                        return false;
                    d[i] = value;
                } else {
                    l[i][j] = value / d[j];
                }
            }
        }
        double c[4][4]{};
        for (int rhs = 0; rhs < 4; ++rhs) {
            double y[4]{}, z[4]{}, solution[4]{};
            for (int i = 0; i < 4; ++i) {
                y[i] = transform[rhs][i];
                for (int k = 0; k < i; ++k)
                    y[i] -= l[i][k] * y[k];
                z[i] = y[i] / d[i];
            }
            for (int i = 3; i >= 0; --i) {
                solution[i] = z[i];
                for (int k = i + 1; k < 4; ++k)
                    solution[i] -= l[k][i] * solution[k];
                c[rhs][i] = solution[i];
            }
        }

        const double stiffness_l = -gravity * (0.5 * ll * mb * cl + leg_proj_l * ml);
        const double stiffness_r = -gravity * (0.5 * lr * mb * cr + leg_proj_r * ml);
        constexpr double e[4][4] = {{1.0, 0.0, 0.0, 0.0},
                                    {0.0, 1.0, 0.0, 0.0},
                                    {-1.0, 0.0, 1.0, 0.0},
                                    {0.0, -1.0, 0.0, 1.0}};

        std::memset(ad_, 0, sizeof(ad_));
        std::memset(bd_, 0, sizeof(bd_));
        for (int i = 0; i < kStateSize; ++i)
            ad_[i][i] = 1.0;
        for (int position = 0; position < 5; ++position)
            ad_[2 * position][2 * position + 1] = kDt;
        for (int q = 0; q < 4; ++q) {
            const int row = 2 * q + 1;
            ad_[row][4] = -kDt * c[q][2] * stiffness_l;
            ad_[row][6] = -kDt * c[q][3] * stiffness_r;
            for (int input = 0; input < kControlSize; ++input)
                for (int k = 0; k < 4; ++k)
                    bd_[row][input] += kDt * c[q][k] * e[k][input];
        }
        bd_[9][2] = -kDt / ib;
        bd_[9][3] = -kDt / ib;
        model_valid_ =
            is_finite(&ad_[0][0], kStateSize * kStateSize)
            && is_finite(&bd_[0][0], kStateSize * kControlSize);
        return model_valid_;
    }

    bool build_riccati() {
        std::memset(p_[kHorizon], 0, sizeof(p_[kHorizon]));
        for (int i = 0; i < kStateSize; ++i)
            p_[kHorizon][i][i] = kTerminalWeight * kQ[i];

        for (int stage = kHorizon - 1; stage >= 0; --stage) {
            const auto (*next_p)[kStateSize] = p_[stage + 1];

            for (int i = 0; i < kStateSize; ++i) {
                for (int input = 0; input < kControlSize; ++input) {
                    double value = 0.0;
                    for (const int row : kDynamicRows)
                        value += next_p[i][row] * bd_[row][input];
                    ws_pb_[i][input] = value;
                }
                std::memcpy(ws_pa_[i], next_p[i], sizeof(ws_pa_[i]));
                for (int position = 0; position < 5; ++position)
                    ws_pa_[i][2 * position + 1] += kDt * next_p[i][2 * position];
                for (int q = 0; q < 4; ++q) {
                    const int row = 2 * q + 1;
                    ws_pa_[i][4] += next_p[i][row] * ad_[row][4];
                    ws_pa_[i][6] += next_p[i][row] * ad_[row][6];
                }
            }

            for (int i = 0; i < kControlSize; ++i) {
                for (int j = 0; j < kControlSize; ++j) {
                    double value = i == j ? kR[i] + kRho : 0.0;
                    for (const int row : kDynamicRows)
                        value += bd_[row][i] * ws_pb_[row][j];
                    ws_h_[i][j] = value;
                }
                for (int j = 0; j < kStateSize; ++j) {
                    double value = 0.0;
                    for (const int row : kDynamicRows)
                        value += bd_[row][i] * ws_pa_[row][j];
                    ws_btp_a_[i][j] = value;
                }
            }
            if (!invert_spd_4x4(ws_h_, h_inv_[stage]))
                return false;
            for (int i = 0; i < kControlSize; ++i)
                for (int j = 0; j < kStateSize; ++j) {
                    double value = 0.0;
                    for (int k = 0; k < kControlSize; ++k)
                        value += h_inv_[stage][i][k] * ws_btp_a_[k][j];
                    gain_[stage][i][j] = value;
                }

            for (int i = 0; i < kStateSize; ++i)
                std::memcpy(ws_atp_a_[i], ws_pa_[i], sizeof(ws_atp_a_[i]));
            for (int position = 0; position < 5; ++position) {
                const int state = 2 * position;
                const int velocity = state + 1;
                for (int j = 0; j < kStateSize; ++j)
                    ws_atp_a_[velocity][j] += kDt * ws_pa_[state][j];
            }
            for (int q = 0; q < 4; ++q) {
                const int row = 2 * q + 1;
                for (int j = 0; j < kStateSize; ++j) {
                    ws_atp_a_[4][j] += ad_[row][4] * ws_pa_[row][j];
                    ws_atp_a_[6][j] += ad_[row][6] * ws_pa_[row][j];
                }
            }
            for (int i = 0; i < kStateSize; ++i)
                for (int j = 0; j <= i; ++j) {
                    double value = ws_atp_a_[i][j] + (i == j ? kQ[i] : 0.0);
                    for (int input = 0; input < kControlSize; ++input)
                        value -= ws_btp_a_[input][i] * gain_[stage][input][j];
                    p_[stage][i][j] = value;
                    p_[stage][j][i] = value;
                }
        }
        return is_finite(&gain_[0][0][0], kHorizon * kControlSize * kStateSize)
            && is_finite(&h_inv_[0][0][0], kHorizon * kControlSize * kControlSize);
    }

    void backward_pass() {
        std::memset(linear_[kHorizon], 0, sizeof(linear_[kHorizon]));
        for (int stage = kHorizon - 1; stage >= 0; --stage) {
            double rhs[kControlSize]{};
            for (int i = 0; i < kControlSize; ++i) {
                rhs[i] = kRho * (dual_[stage][i] - projected_[stage][i]);
                for (const int row : kDynamicRows)
                    rhs[i] += bd_[row][i] * linear_[stage + 1][row];
            }
            for (int i = 0; i < kControlSize; ++i) {
                double value = 0.0;
                for (int j = 0; j < kControlSize; ++j)
                    value += h_inv_[stage][i][j] * rhs[j];
                feedforward_[stage][i] = value;
            }
            const double* next = linear_[stage + 1];
            std::memcpy(linear_[stage], next, sizeof(linear_[stage]));
            for (int position = 0; position < 5; ++position)
                linear_[stage][2 * position + 1] += kDt * next[2 * position];
            for (int q = 0; q < 4; ++q) {
                const int row = 2 * q + 1;
                linear_[stage][4] += ad_[row][4] * next[row];
                linear_[stage][6] += ad_[row][6] * next[row];
            }
            for (int i = 0; i < kStateSize; ++i)
                for (int j = 0; j < kControlSize; ++j)
                    linear_[stage][i] -= gain_[stage][j][i] * rhs[j];
        }
    }

    void forward_pass(const double error[kStateSize]) {
        std::memcpy(x_[0], error, sizeof(x_[0]));
        for (int stage = 0; stage < kHorizon; ++stage) {
            for (int i = 0; i < kControlSize; ++i) {
                u_[stage][i] = -feedforward_[stage][i];
                for (int j = 0; j < kStateSize; ++j)
                    u_[stage][i] -= gain_[stage][i][j] * x_[stage][j];
            }
            std::memcpy(x_[stage + 1], x_[stage], sizeof(x_[stage + 1]));
            for (int position = 0; position < 5; ++position)
                x_[stage + 1][2 * position] += kDt * x_[stage][2 * position + 1];
            for (int q = 0; q < 4; ++q) {
                const int row = 2 * q + 1;
                x_[stage + 1][row] += ad_[row][4] * x_[stage][4] + ad_[row][6] * x_[stage][6];
                for (int input = 0; input < kControlSize; ++input)
                    x_[stage + 1][row] += bd_[row][input] * u_[stage][input];
            }
        }
    }

    double ad_[kStateSize][kStateSize]{};
    double bd_[kStateSize][kControlSize]{};
    double p_[kHorizon + 1][kStateSize][kStateSize]{};
    double gain_[kHorizon][kControlSize][kStateSize]{};
    double h_inv_[kHorizon][kControlSize][kControlSize]{};
    double linear_[kHorizon + 1][kStateSize]{};
    double feedforward_[kHorizon][kControlSize]{};
    double x_[kHorizon + 1][kStateSize]{};
    double u_[kHorizon][kControlSize]{};
    double projected_[kHorizon][kControlSize]{};
    double dual_[kHorizon][kControlSize]{};
    double ws_pb_[kStateSize][kControlSize]{};
    double ws_pa_[kStateSize][kStateSize]{};
    double ws_h_[kControlSize][kControlSize]{};
    double ws_btp_a_[kControlSize][kStateSize]{};
    double ws_atp_a_[kStateSize][kStateSize]{};
    bool model_valid_ = false;
    bool riccati_valid_ = false;
    int riccati_age_ = 0;
    double gain_left_length_ = 0.0;
    double gain_right_length_ = 0.0;
    double gain_theta_left_ = 0.0;
    double gain_theta_right_ = 0.0;
    double residual_ = 0.0;
};

class BalanceRollMpc {
public:
    static constexpr int kHorizon = 10;

    void reset() {
        std::memset(projected_, 0, sizeof(projected_));
        std::memset(dual_, 0, sizeof(dual_));
    }

    /// @param roll/roll_rate 滚转角/角速率 [rad, rad/s]
    /// @param force_arm 差动力力臂 [m]，inertia 滚转惯量 [kg·m²]
    /// @param force_limit 差动力限幅 [N]
    /// @param differential_force 输出的左右腿差动支撑力 [N]
    bool solve(double roll, double roll_rate, double force_arm, double inertia,
               double force_limit, double& differential_force) {
        if (!std::isfinite(roll) || !std::isfinite(roll_rate) || force_arm <= 0.0
            || inertia <= 0.0 || force_limit <= 0.0)
            return false;
        if (!initialized_ || std::fabs(force_arm - last_force_arm_) > 1e-5
            || std::fabs(inertia - last_inertia_) > 1e-5)
            build(force_arm, inertia);

        for (int i = 0; i < kHorizon - 1; ++i) {
            projected_[i] = projected_[i + 1];
            dual_[i] = dual_[i + 1];
        }
        const double b = 2.0 * force_arm / inertia;
        const double bd[2] = {0.5 * kDt * kDt * b, kDt * b};
        for (int iteration = 0; iteration < kAdmmIterations; ++iteration) {
            double linear[kHorizon + 1][2]{};
            double feedforward[kHorizon]{};
            for (int stage = kHorizon - 1; stage >= 0; --stage) {
                const double rhs = kRho * (dual_[stage] - projected_[stage])
                    + bd[0] * linear[stage + 1][0] + bd[1] * linear[stage + 1][1];
                feedforward[stage] = h_inv_[stage] * rhs;
                linear[stage][0] = linear[stage + 1][0] - gain_[stage][0] * rhs;
                linear[stage][1] = kDt * linear[stage + 1][0] + linear[stage + 1][1]
                    - gain_[stage][1] * rhs;
            }
            double x[2] = {roll, roll_rate};
            for (int stage = 0; stage < kHorizon; ++stage) {
                const double u = -gain_[stage][0] * x[0] - gain_[stage][1] * x[1]
                    - feedforward[stage];
                projected_[stage] = std::clamp(u + dual_[stage], -force_limit, force_limit);
                dual_[stage] += u - projected_[stage];
                x[0] += kDt * x[1] + bd[0] * u;
                x[1] += bd[1] * u;
            }
        }
        differential_force = projected_[0];
        return std::isfinite(differential_force);
    }

private:
    static constexpr double kDt = 0.001;
    static constexpr double kRho = 1.0;
    static constexpr int kAdmmIterations = 6;

    void build(double force_arm, double inertia) {
        const double b = 2.0 * force_arm / inertia;
        const double a[2][2] = {{1.0, kDt}, {0.0, 1.0}};
        const double bd[2] = {0.5 * kDt * kDt * b, kDt * b};
        constexpr double q[2] = {20000.0, 30.0};
        constexpr double r = 0.0005;
        double p[2][2] = {{5.0 * q[0], 0.0}, {0.0, 5.0 * q[1]}};
        for (int stage = kHorizon - 1; stage >= 0; --stage) {
            double pa[2][2]{};
            for (int i = 0; i < 2; ++i)
                for (int j = 0; j < 2; ++j)
                    for (int k = 0; k < 2; ++k)
                        pa[i][j] += p[i][k] * a[k][j];
            const double h = r + kRho + bd[0] * (p[0][0] * bd[0] + p[0][1] * bd[1])
                + bd[1] * (p[1][0] * bd[0] + p[1][1] * bd[1]);
            h_inv_[stage] = 1.0 / h;
            for (int j = 0; j < 2; ++j)
                gain_[stage][j] = h_inv_[stage] * (bd[0] * pa[0][j] + bd[1] * pa[1][j]);
            double next[2][2]{};
            for (int i = 0; i < 2; ++i)
                for (int j = 0; j < 2; ++j) {
                    for (int k = 0; k < 2; ++k)
                        next[i][j] += a[k][i] * pa[k][j];
                    next[i][j] -= (bd[0] * pa[0][i] + bd[1] * pa[1][i]) * gain_[stage][j];
                    if (i == j)
                        next[i][j] += q[i];
                }
            std::memcpy(p, next, sizeof(p));
        }
        last_force_arm_ = force_arm;
        last_inertia_ = inertia;
        initialized_ = true;
    }

    bool initialized_ = false;
    double last_force_arm_ = 0.0;
    double last_inertia_ = 0.0;
    double gain_[kHorizon][2]{};
    double h_inv_[kHorizon]{};
    double projected_[kHorizon]{};
    double dual_[kHorizon]{};
};

} // namespace hcs_core::controller::chassis::balance
