#pragma once

#include <cmath>

namespace hcs_core::controller::chassis::balance {

// ============================================================================
// 五杆腿运动学、VMC 雅可比与接触力反解。
//
// 逐行移植自 Helios
// User_Code/module/algorithm/balance_alg/balance_algorithm.cpp 的
// leg_pos() / leg_VMC() / inverse_contact_force()（arm_cos_f32 → std::cos 等，
// float → double），数值行为一致（gtest 与 Helios 原函数对拍，容差 1e-5）。
//
// 五杆机构：两个髋电机（phi1 = front 关节、phi4 = back 关节，l5 = 0 时同轴），
// 两条腿杆 l1、l2，膝部铰接；轮轴在 C 点延长线上。输出腿长 l0、腿角 phi0
//（相对铅垂）及膝关节角 phi2/phi3，和速度雅可比 (dl0, dphi0)。
// ============================================================================

/// 五杆腿正解 + 速度正解。
struct LegPose {
    double length = 0.0;         ///< l0，腿长 [m]
    double angle = 0.0;          ///< phi0，腿摆角 [rad]，0 = 铅垂向下
    double length_velocity = 0.0; ///< dl0 [m/s]
    double angle_velocity = 0.0;  ///< dphi0 [rad/s]
    double knee_front = 0.0;     ///< phi2，前侧膝角 [rad]
    double knee_back = 0.0;      ///< phi3，后侧膝角 [rad]
};

inline LegPose leg_pos(
    double phi1, double phi4, double l1, double l2, double l5, double dphi1, double dphi4) {
    // 变量名跟 Helios 原函数一一对应（xb/yb/xd/yd = 两个髋关节点，c0/yc 中间量被
    // 三次复用：先做余弦定理的 (xd-xb) 与其平方，再做膝点 xc，再减 l5/2 做轮心）。
    const double xb = l1 * std::cos(phi1);
    const double yb = l1 * std::sin(phi1);
    const double xd = l5 + l1 * std::cos(phi4);
    const double yd = l1 * std::sin(phi4);

    double c0 = xd - xb;
    const double a0 = 2.0 * l2 * c0;
    double yc = yd - yb;
    const double b0 = 2.0 * l2 * yc;
    const double l2_squared = l2 * l2;
    c0 = ((l2_squared + c0 * c0) + yc * yc) - l2_squared;

    double discriminant = (a0 * a0 + b0 * b0) - c0 * c0;
    if (discriminant < 0.0)
        discriminant = 0.0;
    const double t1 = std::sqrt(discriminant);

    LegPose pose;
    pose.knee_front = 2.0 * std::atan2(b0 + t1, a0 + c0);
    c0 = xb + l2 * std::cos(pose.knee_front);
    yc = yb + l2 * std::sin(pose.knee_front);
    pose.knee_back = std::atan2(yc - yd, c0 - xd);
    c0 -= l5 / 2.0;

    pose.length = std::sqrt(c0 * c0 + yc * yc);
    pose.angle = std::atan2(yc, c0);

    double velocity_denominator = std::sin(pose.knee_front - pose.knee_back);
    if (std::fabs(velocity_denominator) < 1e-6)
        velocity_denominator = velocity_denominator >= 0.0 ? 1e-6 : -1e-6;

    const double dxc = l1 * std::sin(phi1 - pose.knee_front) * std::sin(pose.knee_back) * dphi1
                           / velocity_denominator
        + l1 * std::sin(pose.knee_back - phi4) * std::sin(pose.knee_front) * dphi4
              / velocity_denominator;
    const double dyc = -l1 * std::sin(phi1 - pose.knee_front) * std::cos(pose.knee_back) * dphi1
                           / velocity_denominator
        - l1 * std::sin(pose.knee_back - phi4) * std::cos(pose.knee_front) * dphi4
              / velocity_denominator;

    pose.length_velocity = dxc * std::cos(pose.angle) + dyc * std::sin(pose.angle);
    pose.angle_velocity = -dxc * std::sin(pose.angle) + dyc * std::cos(pose.angle);
    return pose;
}

/// 腿力 → 关节力矩的 VMC 雅可比。F：沿腿方向的支撑力 [N]，Tp：绕髋的俯仰力矩 [N·m]。
/// 返回两个关节（front、back）的力矩 [N·m]。
inline void leg_vmc(
    double phi0, double phi1, double phi2, double phi3, double phi4, double l1, double l0,
    double force, double hip_torque, double& joint_torque_front, double& joint_torque_back) {
    const double j1 = std::sin(phi3 - phi2);
    const double j3 = std::sin(phi3 - phi4);
    const double jacobian[4] = {
        l1 * std::sin(phi0 - phi3) * std::sin(phi1 - phi2) / j1,
        l1 * std::cos(phi0 - phi3) * std::sin(phi1 - phi2) / (l0 * j1),
        l1 * std::sin(phi0 - phi2) * j3 / j1,
        l1 * std::cos(phi0 - phi2) * j3 / (l0 * j1),
    };
    joint_torque_front = jacobian[0] * force + jacobian[1] * hip_torque;
    joint_torque_back = jacobian[2] * force + jacobian[3] * hip_torque;
}

/// 关节力矩 → 接触力反解。side：左腿 -1、右腿 +1（Helios 的 LEFT/RIGHT 约定）。
/// Tp 限幅 ±800 N·m，|Tp| < 0.05 归零。
inline void inverse_contact_force(
    double phi0, double phi1, double phi2, double phi3, double phi4, double l1, double l0,
    double joint_torque_front, double joint_torque_back, double& force, double& hip_torque,
    int side) {
    const double j1 = std::sin(phi3 - phi2);
    const double j3 = std::sin(phi3 - phi4);
    const double jacobian[4] = {
        l1 * std::sin(phi0 - phi3) * std::sin(phi1 - phi2) / j1,
        l1 * std::cos(phi0 - phi3) * std::sin(phi1 - phi2) / (l0 * j1),
        l1 * std::sin(phi0 - phi2) * j3 / j1,
        l1 * std::cos(phi0 - phi2) * j3 / (l0 * j1),
    };

    double determinant = jacobian[0] * jacobian[3] - jacobian[1] * jacobian[2];
    if (std::fabs(determinant) < 1e-6)
        determinant = determinant >= 0.0 ? 1e-6 : -1e-6; // 奇异性防护
    const double inverse_determinant = 1.0 / determinant;

    const double side_sign = side == 0 ? -1.0 : 1.0;
    force = (joint_torque_front * jacobian[3] - joint_torque_back * jacobian[1])
        * inverse_determinant * side_sign;

    constexpr double kHipTorqueLimit = 800.0;
    hip_torque = (-joint_torque_front * jacobian[2] + joint_torque_back * jacobian[0])
        * inverse_determinant * side_sign;
    hip_torque = std::clamp(hip_torque, -kHipTorqueLimit, kHipTorqueLimit);
    if (std::fabs(hip_torque) < 0.05)
        hip_torque = 0.0;
}

} // namespace hcs_core::controller::chassis::balance
