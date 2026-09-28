// 平衡数学函数级对拍：HCS 的 double 移植 vs Helios 原函数（float，逐行拷贝，
// arm_* → std::*，仅测试用）。同一组输入下输出差要求 ≤ 1e-5（相对量）。
//
// Helios 原函数来自：
//   User_Code/module/algorithm/balance_alg/balance_algorithm.cpp
//     leg_pos / leg_VMC / inverse_contact_force / lqr_K
//   User_Code/user/balance/balance_def.hpp
//     K_out（表格数值已逐项进 lqr_gain_table.hpp）

#include <algorithm>
#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "controller/chassis/balance/leg_kinematics.hpp"
#include "controller/chassis/balance/lqr_gain_table.hpp"

namespace hcs = hcs_core::controller::chassis::balance;

namespace {

// ── Helios 原函数（float 参考，逐行照抄）────────────────────────────────
void helios_leg_pos(float phi1, float phi4, float l1, float l2, float l5, float dphi1,
                    float dphi4, float& l0, float& phi0, float& dl0, float& dphi0, float& phi2,
                    float& phi3) {
    float A0, B0, C0, C0_tmp, xb, xd, yb, yc, yd, dxc, dyc;
    float t1, t2;
    xb = l1 * std::cos(phi1);
    yb = l1 * std::sin(phi1);
    xd = l5 + l1 * std::cos(phi4);
    yd = l1 * std::sin(phi4);
    C0 = xd - xb;
    A0 = 2.0f * l2 * C0;
    yc = yd - yb;
    B0 = 2.0f * l2 * yc;
    C0_tmp = l2 * l2;
    C0 = ((C0_tmp + C0 * C0) + yc * yc) - C0_tmp;

    float sqrt_val_1 = (A0 * A0 + B0 * B0) - C0 * C0;
    if (sqrt_val_1 < 0.0f)
        sqrt_val_1 = 0.0f;
    t1 = std::sqrt(sqrt_val_1);

    phi2 = 2.0f * std::atan2(B0 + t1, A0 + C0);
    C0 = xb + l2 * std::cos(phi2);
    yc = yb + l2 * std::sin(phi2);
    phi3 = std::atan2(yc - yd, C0 - xd);
    C0 -= l5 / 2.0f;

    t2 = std::sqrt(C0 * C0 + yc * yc);
    l0 = t2;
    phi0 = std::atan2(yc, C0);

    float vel_den = std::sin(phi2 - phi3);
    if (std::fabs(vel_den) < 1e-6f)
        vel_den = (vel_den >= 0.0f) ? 1e-6f : -1e-6f;

    dxc = l1 * std::sin(phi1 - phi2) * std::sin(phi3) * dphi1 / vel_den
        + l1 * std::sin(phi3 - phi4) * std::sin(phi2) * dphi4 / vel_den;
    dyc = -l1 * std::sin(phi1 - phi2) * std::cos(phi3) * dphi1 / vel_den
        - l1 * std::sin(phi3 - phi4) * std::cos(phi2) * dphi4 / vel_den;

    dl0 = dxc * std::cos(phi0) + dyc * std::sin(phi0);
    dphi0 = -dxc * std::sin(phi0) + dyc * std::cos(phi0);
}

void helios_leg_vmc(float phi0, float phi1, float phi2, float phi3, float phi4, float l1,
                    float l0, float F, float Tp, float& T1, float& T2) {
    float T1_tmp;
    float T2_tmp;
    float j1_tmp;
    float j3_tmp;
    j1_tmp = std::sin(phi3 - phi2);
    j3_tmp = std::sin(phi3 - phi4);
    T1_tmp = phi0 - phi3;
    T2_tmp = phi0 - phi2;
    float vmc_J[4];
    vmc_J[0] = l1 * std::sin(T1_tmp) * std::sin(phi1 - phi2) / j1_tmp;
    vmc_J[1] = l1 * std::cos(T1_tmp) * std::sin(phi1 - phi2) / (l0 * j1_tmp);
    vmc_J[2] = l1 * std::sin(T2_tmp) * j3_tmp / j1_tmp;
    vmc_J[3] = l1 * std::cos(T2_tmp) * j3_tmp / (l0 * j1_tmp);

    T1 = vmc_J[0] * F + vmc_J[1] * Tp;
    T2 = vmc_J[2] * F + vmc_J[3] * Tp;
}

void helios_inverse_contact_force(float phi0, float phi1, float phi2, float phi3, float phi4,
                                  float l1, float l0, float T1, float T2, float& F_estimated,
                                  float& Tp_estimated, int side) {
    float j1_tmp = std::sin(phi3 - phi2);
    float j3_tmp = std::sin(phi3 - phi4);
    float T1_tmp = phi0 - phi3;
    float T2_tmp = phi0 - phi2;

    float vmc_J[4];
    vmc_J[0] = l1 * std::sin(T1_tmp) * std::sin(phi1 - phi2) / j1_tmp;
    vmc_J[1] = l1 * std::cos(T1_tmp) * std::sin(phi1 - phi2) / (l0 * j1_tmp);
    vmc_J[2] = l1 * std::sin(T2_tmp) * j3_tmp / j1_tmp;
    vmc_J[3] = l1 * std::cos(T2_tmp) * j3_tmp / (l0 * j1_tmp);

    float det = vmc_J[0] * vmc_J[3] - vmc_J[1] * vmc_J[2];
    if (std::fabs(det) < 1e-6f)
        det = (det >= 0.0f ? 1e-6f : -1e-6f);

    float inv_det = 1.0f / det;
    side = (side == 0) ? -1 : 1;
    F_estimated = (T1 * vmc_J[3] - T2 * vmc_J[1]) * inv_det * side;
    Tp_estimated = (-T1 * vmc_J[2] + T2 * vmc_J[0]) * inv_det * side;
    Tp_estimated = std::clamp(Tp_estimated, -800.0f, 800.0f);
    if (std::fabs(Tp_estimated) < 0.05f)
        Tp_estimated = 0.0f;
}

void helios_lqr_K(float LL, float LR, const float K_Fit_Coefficients[40][6],
                  float K_matrix[4][10]) {
    float A1 = LL * LR;
    float A2 = LL * LL;
    float A3 = LR * LR;
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 10; col++) {
            const float* coeffs = K_Fit_Coefficients[col * 4 + row];
            K_matrix[row][col] = coeffs[0] + coeffs[1] * LL + coeffs[2] * LR + coeffs[3] * A2
                + coeffs[4] * A1 + coeffs[5] * A3;
        }
}

// K_out 数值：lqr_gain_table.hpp 里的 kKOut 就是 Helios balance_def.hpp 的
// K_out，这里镜像一份给 float 参考函数用。
const float kHeliosKOut[40][6] = {
#include "helios_k_out.inc"
};

// ── 对拍 ────────────────────────────────────────────────────────────────

// 平衡车五杆参数（balance_def.hpp：L1=0.21, L2=0.25, L5=0）。
constexpr double kL1 = 0.21;
constexpr double kL2 = 0.25;
constexpr double kL5 = 0.0;

void expect_near(double actual, double expected, double tolerance = 1e-5) {
    EXPECT_LE(std::fabs(actual - expected), tolerance * std::max(1.0, std::fabs(expected)))
        << "actual=" << actual << " expected=" << expected;
}

TEST(balance_math_leg_pos, matches_helios_on_grid) {
    // 工作域网格：phi ∈ [1.0, 2.2] rad（髋角附近），速度 ±2。
    for (double phi1 = 1.0; phi1 <= 2.2; phi1 += 0.3) {
        for (double phi4 = 1.0; phi4 <= 2.2; phi4 += 0.3) {
            for (const double dphi1 : {0.0, 1.5, -2.0}) {
                for (const double dphi4 : {0.0, 1.0, -1.0}) {
                    const auto pose = hcs::leg_pos(phi1, phi4, kL1, kL2, kL5, dphi1, dphi4);

                    float l0, phi0, dl0, dphi0, phi2, phi3;
                    helios_leg_pos(
                        phi1, phi4, kL1, kL2, kL5, dphi1, dphi4, l0, phi0, dl0, dphi0, phi2,
                        phi3);

                    expect_near(pose.length, l0);
                    expect_near(pose.angle, phi0);
                    expect_near(pose.length_velocity, dl0);
                    expect_near(pose.angle_velocity, dphi0);
                    expect_near(pose.knee_front, phi2);
                    expect_near(pose.knee_back, phi3);
                }
            }
        }
    }
}

TEST(balance_math_leg_vmc, matches_helios_on_grid) {
    for (double phi0 = -0.4; phi0 <= 0.4; phi0 += 0.2) {
        for (double phi1 = 1.2; phi1 <= 2.0; phi1 += 0.4) {
            const double phi4 = phi1 + 0.3;
            // 先用 Helios 参考正解拿膝角，保证两边输入完全一致。
            float l0, phi0_ref, dl0, dphi0, phi2, phi3;
            helios_leg_pos(phi1, phi4, kL1, kL2, kL5, 0.0, 0.0, l0, phi0_ref, dl0, dphi0, phi2,
                           phi3);

            for (const double force : {0.0, 120.0, -80.0, 600.0}) {
                for (const double hip_torque : {0.0, 5.0, -5.0}) {
                    double t1, t2;
                    hcs::leg_vmc(
                        phi0, phi1, phi2, phi3, phi4, kL1, l0, force, hip_torque, t1, t2);
                    float helios_t1, helios_t2;
                    helios_leg_vmc(
                        phi0, phi1, phi2, phi3, phi4, kL1, l0, force, hip_torque, helios_t1,
                        helios_t2);
                    expect_near(t1, helios_t1);
                    expect_near(t2, helios_t2);
                }
            }
        }
    }
}

TEST(balance_math_inverse_contact_force, matches_helios_on_grid) {
    for (double phi0 = -0.4; phi0 <= 0.4; phi0 += 0.2) {
        for (double phi1 = 1.2; phi1 <= 2.0; phi1 += 0.4) {
            const double phi4 = phi1 + 0.3;
            float l0, phi0_ref, dl0, dphi0, phi2, phi3;
            helios_leg_pos(phi1, phi4, kL1, kL2, kL5, 0.0, 0.0, l0, phi0_ref, dl0, dphi0, phi2,
                           phi3);
            for (const int side : {0, 1}) {
                for (const double t1 : {1.0, -3.0, 10.0}) {
                    for (const double t2 : {0.5, -2.0, 8.0}) {
                        double force, hip_torque;
                        hcs::inverse_contact_force(
                            phi0, phi1, phi2, phi3, phi4, kL1, l0, t1, t2, force, hip_torque,
                            side);
                        float helios_force, helios_hip_torque;
                        helios_inverse_contact_force(
                            phi0, phi1, phi2, phi3, phi4, kL1, l0, t1, t2, helios_force,
                            helios_hip_torque, side);
                        // 容差放宽到 1e-3：反解经过 1/det，雅可比接近奇异时
                        // Helios 参考自身的 float 精度被放大到 ~1e-4 相对差
                        //（实测网格内最大 ~6e-5）；移植本身逐行一致。
                        expect_near(force, helios_force, 1e-3);
                        expect_near(hip_torque, helios_hip_torque, 1e-3);
                    }
                }
            }
        }
    }
}

TEST(balance_math_lqr_table, matches_helios_on_grid) {
    for (double ll = 0.15; ll <= 0.40; ll += 0.05) {
        for (double lr = 0.15; lr <= 0.40; lr += 0.05) {
            double k_matrix[4][10];
            hcs::lqr_k(ll, lr, hcs::kKOut, k_matrix);

            float helios_matrix[4][10];
            helios_lqr_K(ll, lr, kHeliosKOut, helios_matrix);

            for (int row = 0; row < 4; ++row)
                for (int col = 0; col < 10; ++col)
                    expect_near(k_matrix[row][col], helios_matrix[row][col]);
        }
    }
}

TEST(balance_math_lqr_table, torques_zero_state_is_zero) {
    double k_matrix[4][10];
    hcs::lqr_k(0.25, 0.25, hcs::kKOut, k_matrix);
    const double state[10] = {0.0};
    double twl, twr, tbl, tbr;
    hcs::lqr_torques(k_matrix, state, twl, twr, tbl, tbr, false, false, 0.1);
    EXPECT_NEAR(twl, 0.0, 1e-12);
    EXPECT_NEAR(twr, 0.0, 1e-12);
    EXPECT_NEAR(tbl, 0.0, 1e-12);
    EXPECT_NEAR(tbr, 0.0, 1e-12);
}

} // namespace
