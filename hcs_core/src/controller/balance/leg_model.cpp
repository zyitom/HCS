#include "leg_model.hpp"

#include <cmath>

#include "params.hpp"

namespace hcs_core::controller::balance {

LegKinematics leg_pos(
    float phi1, float phi4, float l1, float l2, float l5, float dphi1, float dphi4) {
    float a0, b0, c0, c0_tmp, xb, xd, yb, yc, yd;
    float t1, t2;
    xb = l1 * std::cos(phi1);
    yb = l1 * std::sin(phi1);
    xd = l5 + l1 * std::cos(phi4);
    yd = l1 * std::sin(phi4);
    c0 = xd - xb;
    a0 = 2.0f * l2 * c0;
    yc = yd - yb;
    b0 = 2.0f * l2 * yc;
    c0_tmp = l2 * l2;
    c0 = ((c0_tmp + c0 * c0) + yc * yc) - c0_tmp;

    float sqrt_val_1 = (a0 * a0 + b0 * b0) - c0 * c0;
    if (sqrt_val_1 < 0.0f) {
        sqrt_val_1 = 0.0f;
    }
    t1 = std::sqrt(sqrt_val_1);

    LegKinematics out;
    out.phi2 = 2.0f * std::atan2(b0 + t1, a0 + c0);
    c0 = xb + l2 * std::cos(out.phi2);
    yc = yb + l2 * std::sin(out.phi2);
    out.phi3 = std::atan2(yc - yd, c0 - xd);
    c0 -= l5 / 2.0f;

    t2 = std::sqrt(c0 * c0 + yc * yc);
    out.l0 = t2;
    out.phi0 = std::atan2(yc, c0);

    float vel_den = std::sin(out.phi2 - out.phi3);
    if (std::fabs(vel_den) < 1e-6f) {
        vel_den = (vel_den >= 0.0f) ? 1e-6f : -1e-6f;
    }

    const float dxc = l1 * std::sin(phi1 - out.phi2) * std::sin(out.phi3) * dphi1 / vel_den
                    + l1 * std::sin(out.phi3 - phi4) * std::sin(out.phi2) * dphi4 / vel_den;
    const float dyc = -l1 * std::sin(phi1 - out.phi2) * std::cos(out.phi3) * dphi1 / vel_den
                    - l1 * std::sin(out.phi3 - phi4) * std::cos(out.phi2) * dphi4 / vel_den;

    out.dl0 = dxc * std::cos(out.phi0) + dyc * std::sin(out.phi0);
    out.dphi0 = -dxc * std::sin(out.phi0) + dyc * std::cos(out.phi0);
    return out;
}

void leg_vmc(
    float phi0, float phi1, float phi2, float phi3, float phi4, float l1, float l0, float f,
    float tp, float& t1, float& t2) {
    const float j1_tmp = std::sin(phi3 - phi2);
    const float j3_tmp = std::sin(phi3 - phi4);
    const float t1_tmp = phi0 - phi3;
    const float t2_tmp = phi0 - phi2;

    const float vmc_j0 = l1 * std::sin(t1_tmp) * std::sin(phi1 - phi2) / j1_tmp;
    const float vmc_j1 = l1 * std::cos(t1_tmp) * std::sin(phi1 - phi2) / (l0 * j1_tmp);
    const float vmc_j2 = l1 * std::sin(t2_tmp) * j3_tmp / j1_tmp;
    const float vmc_j3 = l1 * std::cos(t2_tmp) * j3_tmp / (l0 * j1_tmp);

    t1 = vmc_j0 * f + vmc_j1 * tp;
    t2 = vmc_j2 * f + vmc_j3 * tp;
}

void inverse_contact_force(
    float phi0, float phi1, float phi2, float phi3, float phi4, float l1, float l0, float t1,
    float t2, float& f_estimated, float& tp_estimated, int side) {
    const float j1_tmp = std::sin(phi3 - phi2);
    const float j3_tmp = std::sin(phi3 - phi4);
    const float t1_tmp = phi0 - phi3;
    const float t2_tmp = phi0 - phi2;

    const float vmc_j0 = l1 * std::sin(t1_tmp) * std::sin(phi1 - phi2) / j1_tmp;
    const float vmc_j1 = l1 * std::cos(t1_tmp) * std::sin(phi1 - phi2) / (l0 * j1_tmp);
    const float vmc_j2 = l1 * std::sin(t2_tmp) * j3_tmp / j1_tmp;
    const float vmc_j3 = l1 * std::cos(t2_tmp) * j3_tmp / (l0 * j1_tmp);

    float det = vmc_j0 * vmc_j3 - vmc_j1 * vmc_j2;
    if (std::fabs(det) < 1e-6f)
        det = (det >= 0.0f ? 1e-6f : -1e-6f);
    const float inv_det = 1.0f / det;

    side = (side == 0) ? -1 : 1;
    f_estimated = (t1 * vmc_j3 - t2 * vmc_j1) * inv_det * side;
    tp_estimated = (-t1 * vmc_j2 + t2 * vmc_j0) * inv_det * side;
    tp_estimated = abs_clip(tp_estimated, 800.0f);
    if (std::fabs(tp_estimated) < 0.05f)
        tp_estimated = 0.0f;
}

void lqr_k(float ll, float lr, std::array<std::array<float, 10>, 4>& k_matrix) {
    const float a1 = ll * lr;
    const float a2 = ll * ll;
    const float a3 = lr * lr;
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 10; col++) {
            const auto& coeffs = lqr_tables::k_out[static_cast<std::size_t>(col * 4 + row)];
            k_matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] =
                coeffs[0] + coeffs[1] * ll + coeffs[2] * lr + coeffs[3] * a2 + coeffs[4] * a1
                + coeffs[5] * a3;
        }
    }
}

void calculate_status_vector(
    const std::array<std::array<float, 10>, 4>& k_matrix,
    const std::array<float, 10>& status_vector, float& tlwl, float& tlwr, float& tbll,
    float& tblr) {
    tlwl = tbll = tlwr = tblr = 0.0f;
    for (int col = 0; col < 10; ++col) {
        tlwl += k_matrix[0][static_cast<std::size_t>(col)] * status_vector[static_cast<std::size_t>(col)];
        tlwr += k_matrix[1][static_cast<std::size_t>(col)] * status_vector[static_cast<std::size_t>(col)];
        tbll += k_matrix[2][static_cast<std::size_t>(col)] * status_vector[static_cast<std::size_t>(col)];
        tblr += k_matrix[3][static_cast<std::size_t>(col)] * status_vector[static_cast<std::size_t>(col)];
    }
}

void calculate_status_vector_fly(
    const std::array<std::array<float, 10>, 4>& k_matrix,
    const std::array<float, 10>& status_vector, float& tlwl, float& tlwr, float& tbll,
    float& tblr, bool left_off_gnd, bool right_off_gnd, float improve0) {
    // 与 Helios 逐行对应：左右两组输出独立赋值，一组离地不影响另一组的全通道结果。
    const auto dot = [&](std::size_t row) {
        float sum = 0.0f;
        for (int col = 0; col < 10; ++col)
            sum += k_matrix[row][static_cast<std::size_t>(col)]
                 * status_vector[static_cast<std::size_t>(col)];
        return sum;
    };

    if (left_off_gnd) {
        tlwl = 0;
        tbll = (k_matrix[2][4] * status_vector[4] + k_matrix[2][5] * status_vector[5]) * improve0;
    } else {
        tlwl = dot(0);
        tbll = dot(2);
    }
    if (right_off_gnd) {
        tlwr = 0;
        tblr = (k_matrix[3][6] * status_vector[6] + k_matrix[3][7] * status_vector[7]) * improve0;
    } else {
        tlwr = dot(1);
        tblr = dot(3);
    }
}

float spring_force_estimation(float l0, const Params& params) {
    const float fv = -1.8433e3f * l0 * l0 + 1.1122e3f * l0 - 49.6586f + 24.0f;
    (void)params;
    return fv;
}

float multi_turn_detection(float& round_count, float phi, float& phi_last, bool& first_time) {
    if (first_time) {
        phi_last = phi;
        first_time = false;
    }

    if (phi < phi_last && std::fabs(phi - phi_last) > kPi)
        round_count++;
    else if (phi > phi_last && std::fabs(phi - phi_last) > kPi) {
        round_count--;
    }

    phi_last = phi;

    return round_count * 2 * kPi + phi;
}

} // namespace hcs_core::controller::balance
