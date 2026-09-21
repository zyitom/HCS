#pragma once

// 腿机构与 LQR 的纯数学内核。数值语义逐行对应 Helios
// User_Code/module/algorithm/balance_alg/balance_algorithm.cpp（arm_math → std）。

#include <array>
#include <cstddef>

#include "lqr_tables.hpp"
#include "params.hpp"
#include "math.hpp"

namespace hcs_core::controller::balance {

inline constexpr std::size_t kLeft = 0;
inline constexpr std::size_t kRight = 1;

/// 五连杆正运动学。输入 phi1/phi4 为髋电机角（v2 的 phi[0]/phi[3]），输出腿长、
/// 模型腿角及其速率和中间角（v2 的 phi[1]/phi[2]）。
struct LegKinematics {
    float l0{};
    float phi0{};
    float dl0{};
    float dphi0{};
    float phi2{};
    float phi3{};
};

[[nodiscard]] LegKinematics leg_pos(
    float phi1, float phi4, float l1, float l2, float l5, float dphi1, float dphi4);

/// VMC：F/Tp → 双关节力矩 (T1, T2)。
void leg_vmc(
    float phi0, float phi1, float phi2, float phi3, float phi4, float l1, float l0, float f,
    float tp, float& t1, float& t2);

/// 接触力估计。side：kLeft → -1，kRight → +1（Helios 的 side 语义原样保留）。
void inverse_contact_force(
    float phi0, float phi1, float phi2, float phi3, float phi4, float l1, float l0, float t1,
    float t2, float& f_estimated, float& tp_estimated, int side);

/// K 矩阵插值（10×4 转置存储，K_out[col*4+row]）。
void lqr_k(float ll, float lr, std::array<std::array<float, 10>, 4>& k_matrix);

/// 状态向量 → (Tlwl, Tlwr, Tbll, Tblr)。fly 版本：离地腿只用腿部通道。
void calculate_status_vector(
    const std::array<std::array<float, 10>, 4>& k_matrix,
    const std::array<float, 10>& status_vector, float& tlwl, float& tlwr, float& tbll,
    float& tblr);

void calculate_status_vector_fly(
    const std::array<std::array<float, 10>, 4>& k_matrix,
    const std::array<float, 10>& status_vector, float& tlwl, float& tlwr, float& tbll,
    float& tblr, bool left_off_gnd, bool right_off_gnd, float improve0);

/// 气弹簧弹簧力估计（v2 spring_force_estimation）。
[[nodiscard]] float spring_force_estimation(float l0, const Params& params);

/// 多圈检测（v2 Multi_Turn_Detection；round_count/phi_last/first_time 由调用方持有，
/// 状态归估计器，不进接口图）。
[[nodiscard]] float multi_turn_detection(
    float& round_count, float phi, float& phi_last, bool& first_time);

} // namespace hcs_core::controller::balance
