#include "estimator.hpp"

#include <cmath>

namespace hcs_core::controller::balance {

void Estimator::update_kinematics(const LogicInput& input, float dt) {
    auto& wbc = estimate_.wbc;
    auto& leg = estimate_.leg;
    const auto& imu = input.imu;
    const auto& p = params_;

    estimate_.input_wheel_current = {input.wheel[0].current, input.wheel[1].current};

    // ── 速度 KF（先于姿态：v2:107 的 s1 来自 SpeedEstimation，它读上一拍腿状态） ──
    wbc.s1 = -speed_estimation(dt);
    wbc.s += wbc.s1 * dt;

    // ── 姿态（INF_4 映射，v2:122-137） ─────────────────────────────────────
    wbc.thetab = -imu.roll + 0.0111782672f;
    wbc.thetab1 = -imu.droll;
    wbc.phi = imu.total_yaw;
    wbc.phi1 = imu.dyaw;
    wbc.roll = imu.pitch + 0.00654419884f;
    wbc.roll1 = imu.dpitch;
    wbc.a_x = imu.acc_y * p.G;
    const float r_imu =
        (leg[kLeft].L0 * std::cos(leg[kLeft].phi0) + leg[kRight].L0 * std::cos(leg[kRight].phi0))
        / 2.0f;
    wbc.a_y = (imu.acc_x * p.G + std::sin(wbc.thetab) * p.G) * std::cos(wbc.thetab)
            - (wbc.phi1 * wbc.phi1 * r_imu);
    wbc.a_z = (imu.acc_z * p.G - std::cos(wbc.thetab) * p.G) * std::cos(wbc.thetab);
    wbc.v_x += (wbc.a_x > 0.02f ? wbc.a_x : 0.0f) * dt;
    wbc.v_y += wbc.a_y * dt;
    wbc.v_z += wbc.a_z * dt;

    // ── 电机 → 腿几何（v2:139-150；电机顺序 [bl1, bl0, br1, br0]） ──────────
    const auto& bl1 = input.leg_motor[0];
    const auto& bl0 = input.leg_motor[1];
    const auto& br1 = input.leg_motor[2];
    const auto& br0 = input.leg_motor[3];

    leg[kLeft].wheel_spd = input.wheel[0].speed * p.wheel_radius / p.wheel_reduction;
    leg[kRight].wheel_spd = -input.wheel[1].speed * p.wheel_radius / p.wheel_reduction;

    leg[kLeft].phi[3] = -bl1.angle - p.leg_angle_offset / 2.0f + kPi / 2.0f;
    leg[kLeft].phi[0] = -bl0.angle + p.leg_angle_offset / 2.0f + kPi / 2.0f;
    leg[kLeft].dphi[3] = -bl1.speed;
    leg[kLeft].dphi[0] = -bl0.speed;
    leg[kRight].phi[3] = br1.angle - p.leg_angle_offset / 2.0f + kPi / 2.0f;
    leg[kRight].phi[0] = br0.angle + p.leg_angle_offset / 2.0f + kPi / 2.0f;
    leg[kRight].dphi[3] = br1.speed;
    leg[kRight].dphi[0] = br0.speed;

    // ── 多圈检测（v2:149-150） ─────────────────────────────────────────────
    // 注意：v2 在 leg_pos 更新 phi0 之前做多圈检测，phi0_total 天然滞后一拍——
    // 这是原实现的时序怪癖，行为对拍要求逐拍一致，原样保留。
    leg[kLeft].phi0_total = multi_turn_detection(
        multi_turn_[kLeft].round_count, leg[kLeft].phi0, multi_turn_[kLeft].phi0_last,
        multi_turn_[kLeft].first_time);
    leg[kRight].phi0_total = multi_turn_detection(
        multi_turn_[kRight].round_count, leg[kRight].phi0, multi_turn_[kRight].phi0_last,
        multi_turn_[kRight].first_time);
    leg[kLeft].round_count = multi_turn_[kLeft].round_count;
    leg[kRight].round_count = multi_turn_[kRight].round_count;

    leg[kLeft].spring_force = spring_force_estimation(leg[kLeft].L0, p);
    leg[kRight].spring_force = spring_force_estimation(leg[kRight].L0, p);

    // ── 五连杆正运动学（v2:154-157） ───────────────────────────────────────
    for (const std::size_t side : {kLeft, kRight}) {
        const auto kin = leg_pos(
            leg[side].phi[0], leg[side].phi[3], p.L1, p.L2, p.L5, leg[side].dphi[0],
            leg[side].dphi[3]);
        leg[side].L0 = kin.l0;
        leg[side].phi0 = kin.phi0;
        leg[side].dL0 = kin.dl0;
        leg[side].dphi0 = kin.dphi0;
        leg[side].phi[1] = kin.phi2;
        leg[side].phi[2] = kin.phi3;
    }

    // ── 质心、腿惯量、角加速度（v2:158-173） ───────────────────────────────
    leg[kLeft].CoM_x = -(-0.2208f * leg[kLeft].L0 * leg[kLeft].L0 + 0.0553f * leg[kLeft].L0
                         + 0.0301f);
    leg[kLeft].CoM_y = 0.2716f * leg[kLeft].L0 - 0.001f;
    leg[kRight].CoM_x = -(-0.2208f * leg[kRight].L0 * leg[kRight].L0 + 0.0553f * leg[kRight].L0
                          + 0.0301f);
    leg[kRight].CoM_y = 0.2716f * leg[kRight].L0 - 0.001f;

    const float raw_ddphi0_l = (leg[kLeft].dphi0 - leg[kLeft].dphi0_last) / dt;
    const float raw_ddphi0_r = (leg[kRight].dphi0 - leg[kRight].dphi0_last) / dt;
    leg[kLeft].dphi0_last = leg[kLeft].dphi0;
    leg[kRight].dphi0_last = leg[kRight].dphi0;
    leg[kLeft].ddphi0 = filter_ddphi0_[kLeft](raw_ddphi0_l);
    leg[kRight].ddphi0 = filter_ddphi0_[kRight](raw_ddphi0_r);

    leg[kLeft].I_l = 0.1158f * leg[kLeft].L0 + 0.0143f;
    leg[kRight].I_l = 0.1158f * leg[kRight].L0 + 0.0143f;

    // ── 接触力（v2:174-185） ───────────────────────────────────────────────
    inverse_contact_force(
        leg[kLeft].phi0, leg[kLeft].phi[0], leg[kLeft].phi[1], leg[kLeft].phi[2],
        leg[kLeft].phi[3], p.L1, leg[kLeft].L0, bl0.torque, bl1.torque, leg[kLeft].Fn,
        leg[kLeft].Tp, static_cast<int>(kLeft));
    inverse_contact_force(
        leg[kRight].phi0, leg[kRight].phi[0], leg[kRight].phi[1], leg[kRight].phi[2],
        leg[kRight].phi[3], p.L1, leg[kRight].L0, br0.torque, br1.torque, leg[kRight].Fn,
        leg[kRight].Tp, static_cast<int>(kRight));

    const float tp_comp_l = leg[kLeft].Tp - leg[kLeft].I_l * leg[kLeft].ddphi0;
    const float tp_comp_r = leg[kRight].Tp - leg[kRight].I_l * leg[kRight].ddphi0;
    const float angle_to_vertical_l = leg[kLeft].phi0 - kPi / 2 - wbc.thetab;
    leg[kLeft].P = (leg[kLeft].Fn + leg[kLeft].spring_force) * std::cos(angle_to_vertical_l)
                 + tp_comp_l * std::sin(angle_to_vertical_l);
    if (leg[kLeft].P < 0.0f)
        leg[kLeft].P = 0.0f;
    const float angle_to_vertical_r = leg[kRight].phi0 - kPi / 2 - wbc.thetab;
    leg[kRight].P = (leg[kRight].Fn + leg[kRight].spring_force) * std::cos(angle_to_vertical_r)
                  + tp_comp_r * std::sin(angle_to_vertical_r);
    if (leg[kRight].P < 0.0f)
        leg[kRight].P = 0.0f;

    // ── LQR 腿摆角（v2:186-197） ───────────────────────────────────────────
    wbc.thetall = -leg[kLeft].phi0 + kPi / 2 + wbc.thetab;
    if (wbc.thetall > kPi)
        wbc.thetall -= 2 * kPi;
    else if (wbc.thetall < -kPi)
        wbc.thetall += 2 * kPi;
    wbc.thetalr = -leg[kRight].phi0 + kPi / 2 + wbc.thetab;
    if (wbc.thetalr > kPi)
        wbc.thetalr -= 2 * kPi;
    else if (wbc.thetalr < -kPi)
        wbc.thetalr += 2 * kPi;
    wbc.thetall1 = -leg[kLeft].dphi0 + wbc.thetab1;
    wbc.thetalr1 = -leg[kRight].dphi0 + wbc.thetab1;
}

float Estimator::speed_estimation(float dt) {
    const auto& leg = estimate_.leg;
    const auto& wbc = estimate_.wbc;

    const float v_swing_l = wbc.thetall1 * leg[kLeft].L0 * std::cos(wbc.thetall);
    const float v_ext_l = leg[kLeft].dL0 * std::sin(wbc.thetall);
    const float vl = leg[kLeft].wheel_spd - v_swing_l - v_ext_l;

    const float v_swing_r = wbc.thetalr1 * leg[kRight].L0 * std::cos(wbc.thetalr);
    const float v_ext_r = leg[kRight].dL0 * std::sin(wbc.thetalr);
    const float vr = leg[kRight].wheel_spd - v_swing_r - v_ext_r;

    return speed_kf_.update(
        vl, vr, wbc.phi1, wbc.a_y, leg[kLeft].P, leg[kRight].P, leg[kLeft].P < 20.0f,
        leg[kRight].P < 20.0f, leg[kLeft].dphi0, leg[kRight].dphi0, dt);
}

int Estimator::gnd_off_detect(std::size_t leg) {
    // Helios v2:1047 —— 离地检测当前被一行 return 0 禁用。估计器实现算法本体，
    // 由 params.gnd_detection_enabled 复刻"默认禁用"的现状（PORTING.md §9-3）。
    if (!params_.gnd_detection_enabled)
        return 0;

    auto& g = gnd_;
    auto& wbc = estimate_.wbc;
    auto& leg_state = estimate_.leg[leg];

    const float current_p = filter_fn_[leg](leg_state.P) - params_.m_l_g;

    const bool is_changing_len = std::fabs(wbc.LL_want[leg] - g.ll_want_last[leg]) > 1.0f;
    g.ll_want_last[leg] = wbc.LL_want[leg];

    if (!g.d_p_init[leg]) {
        g.p_last[leg] = current_p;
        g.state[leg] = 0;
        for (int i = 0; i < 4; i++)
            g.d_p_buf[leg][i] = 0.0f;
        g.d_p_init[leg] = true;
    }

    constexpr float kDt = 0.001f; // "@1kHz：滤波系数折进了 dP 数值，暂不参数化"
    const float raw_dp = (current_p - g.p_last[leg]) / kDt;
    g.p_last[leg] = current_p;

    g.d_p_buf[leg][g.d_p_idx[leg]] = raw_dp;
    g.d_p_idx[leg] = (g.d_p_idx[leg] + 1) % 4;

    float filtered_dp = 0.0f;
    for (int i = 0; i < 4; i++)
        filtered_dp += g.d_p_buf[leg][i];
    filtered_dp *= 0.25f;

    if (is_changing_len) {
        g.state[leg] = 0;
        return g.state[leg];
    }

    switch (g.state[leg]) {
    case 0:
        if (current_p < params_.gnd_p_air_thres
            && leg_state.L0 > params_.gnd_l_extended_thres) {
            g.state[leg] = 1;
        }
        break;
    case 1:
        // v2:1104-1117 —— dP 冲击分支被注释，只剩压力恢复。
        if (current_p > params_.gnd_p_ground_recover) {
            g.state[leg] = 0;
        }
        break;
    case 2:
        g.impact_timer[leg]++;
        if (g.impact_timer[leg] > static_cast<uint32_t>(
                params_.gnd_impact_duration_s / 0.001f)) {
            if (current_p > params_.gnd_p_air_thres) {
                g.state[leg] = 0;
            } else {
                g.state[leg] = 1;
            }
        }
        break;
    default: break;
    }
    (void)filtered_dp; // 与 v2 相同：滤波结果保留但当前未参与判定
    return g.state[leg];
}

void Estimator::update_contact(const ContactGate& gate) {
    auto& wbc = estimate_.wbc;

    for (const std::size_t leg : {kLeft, kRight}) {
        const int flag = gnd_off_detect(leg);
        // v2:2492-2497 的门控（TOUCH_DOWN 分支已随死状态删除）。
        const bool if_leg_angle_valid =
            std::fabs(estimate_.leg[kLeft].phi0 - (kPi / 2 - params_.pitch_preset)) < 0.28f
            && std::fabs(estimate_.leg[kRight].phi0 - (kPi / 2 - params_.pitch_preset)) < 0.28f;
        if (gate.normal_init_lock || gate.leg_change_lock || !if_leg_angle_valid)
            estimate_.ground.if_off_gnd[leg] = 0;
        else
            estimate_.ground.if_off_gnd[leg] = flag;
        (void)wbc;
    }
}

} // namespace hcs_core::controller::balance
