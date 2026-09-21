#pragma once

// 对拍场景脚本：为参照实现（Helios 原码）与新实现（ChassisLogic）提供逐拍一致的输入。
//
// 输入在 phi 空间（phi[0]/phi[3] 腿摆角）描述，经与 estimator 相同的符号映射换算成
// 电机 offset_angle；位形由数值求逆（leg_pos 的 2D Newton）给出，两侧看到完全相同的
// 电机反馈。接触力（P）通过腿关节力矩注入。

#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "../chassis_logic.hpp"
#include "helios_ref/robot_def.hpp"

namespace hcs_core::controller::balance::port {

struct RefHarness; // 前置：参照实现的电机/IMU 结构集合

inline constexpr float kLegAngleOffset = 1.163f;
inline constexpr float kL1 = 0.21f;
inline constexpr float kL2 = 0.25f;

/// phi 空间 → 电机 offset_angle（与 estimator 的映射互逆）。
struct MotorAngles {
    float bl1{}, bl0{}, br1{}, br0{};
    float bl1_speed{}, bl0_speed{}, br1_speed{}, br0_speed{};
};

[[nodiscard]] inline MotorAngles phi_to_motor(
    float phi0_l, float phi3_l, float phi0_r, float phi3_r, float dphi0_l = 0.0f,
    float dphi3_l = 0.0f, float dphi0_r = 0.0f, float dphi3_r = 0.0f) {
    MotorAngles m;
    m.bl0 = -phi0_l + kLegAngleOffset / 2.0f + kPi / 2.0f;
    m.bl1 = -phi3_l - kLegAngleOffset / 2.0f + kPi / 2.0f;
    m.br0 = phi0_r - kLegAngleOffset / 2.0f - kPi / 2.0f;
    m.br1 = phi3_r + kLegAngleOffset / 2.0f - kPi / 2.0f;
    m.bl0_speed = -dphi0_l;
    m.bl1_speed = -dphi3_l;
    m.br0_speed = dphi0_r;
    m.br1_speed = dphi3_r;
    return m;
}

/// 二维 Newton：由目标 (phi0, L0) 解对称摆角 (phi0t, d)，phi1=phi0t+d, phi4=phi0t-d。
struct Pose {
    float phi0{};
    float l0{};
};

/// 解析求逆：对称五连杆（phi1=phi0t+d, phi4=phi0t-d）的腿长有闭式解
/// r(d) = L1*cos(d) + sqrt(L2^2 - L1^2*sin^2(d))，对 d 单调递减；
/// 对 d ∈ [0, 1.55] 二分，确定性强、无分支跳变，输入是场景时间线的纯函数。
[[nodiscard]] inline std::pair<float, float> solve_motor_angles(
    const Pose& target, std::pair<float, float> = {1.39f, 0.30f}) {
    const auto r_of = [](float d) {
        return kL1 * std::cos(d) + std::sqrt(kL2 * kL2 - kL1 * kL1 * std::sin(d) * std::sin(d));
    };
    float lo = 0.0f, hi = 1.55f;
    const float l0 = clip(target.l0, r_of(hi) + 1e-4f, r_of(lo) - 1e-4f);
    for (int i = 0; i < 60; ++i) {
        const float mid = 0.5f * (lo + hi);
        if (r_of(mid) > l0)
            lo = mid;
        else
            hi = mid;
    }
    return {target.phi0, 0.5f * (lo + hi)};
}

/// 常用位形。
struct Poses {
    // 起身目标：phi0 ≈ 1.39，收腿 L0 ≈ 0.18
    Pose stand_low{1.39f, 0.18f};
    Pose stand_mid{1.52f, 0.25f};
    Pose stand_high{1.45f, 0.36f};
    // 自救终点：phi0 ≈ PI/4，L0 ≈ 0.20
    Pose heal_done{0.7854f, 0.20f};
    // 折叠（上台阶检测）
    Pose folded{1.10f, 0.30f};
    // 上台阶转动到位：phi0 ≈ 0.3
    Pose up_stair_done{0.30f, 0.19f};
    // 跳跃伸腿
    Pose takeoff{1.30f, 0.35f};
};

/// 单个场景的每拍输入。以 lambda 驱动，便于描述时间线。
struct TickInputs {
    // 操作（data_to_chassis 语义）
    bool if_enable = false;
    float speed_level[3] = {0.0f, 0.0f, 0.0f};
    bool if_free = false;
    bool if_turn = false;
    bool jump_key = false; // 电平（开关语义），边沿由两侧各自实现计算
    bool save_key = false;
    bool leg_key = false;
    float follow_angle_cmd = 0.0f;
    float real_angle = 0.0f;
    // 传感器
    float roll = 0.0112f;  // → thetab ≈ 0
    float pitch = -0.0065f;
    float yaw = 0.0f;
    float droll = 0.0f, dpitch = 0.0f, dyaw = 0.0f;
    float total_yaw = 0.0f;
    float acc_x = 0.0f, acc_y = 0.0f, acc_z = 1.0f; // g
    Pose pose_left{1.39f, 0.18f};
    Pose pose_right{1.39f, 0.18f};
    float leg_torque_l0 = -12.0f; // 关节力矩（决定 P，让"着地"成立）
    float leg_torque_l1 = -12.0f;
    float leg_torque_r0 = -12.0f;
    float leg_torque_r1 = -12.0f;
    float wheel_speed_l = 0.0f; // 转子 rad/s
    float wheel_speed_r = 0.0f;
    float wheel_current_l = 0.0f;
    float wheel_current_r = 0.0f;
    bool gnd_detection_enabled = false;
};

using ScenarioStep = std::function<void(int tick, float t, TickInputs&)>;

struct Scenario {
    const char* name;
    int ticks;
    bool gnd_detection_enabled;
    ScenarioStep step;
};

/// 两个 pose 之间按时间线性插值的辅助器。
struct PoseRamp {
    Pose from{};
    Pose to{};
    float t0 = 0.0f;
    float t1 = 1.0f;

    [[nodiscard]] Pose at(float t) const {
        if (t <= t0)
            return from;
        if (t >= t1)
            return to;
        const float k = (t - t0) / (t1 - t0);
        return Pose{from.phi0 + (to.phi0 - from.phi0) * k, from.l0 + (to.l0 - from.l0) * k};
    }
};

/// 场景库。时间单位：秒（tick = 1 ms）。
[[nodiscard]] inline std::vector<Scenario> make_scenarios() {
    std::vector<Scenario> scenarios;
    const Poses P;

    // ── 1. 上电起身（正常路径） ────────────────────────────────────────────
    scenarios.push_back(Scenario{
        "power_on_standup", 6000, false,
        [&](int /*tick*/, float t, TickInputs& in) {
            in.if_enable = t >= 0.0f;
            in.pose_left = P.stand_low;
            in.pose_right = P.stand_low;
        }});

    // ── 2. 起身超时路径（phi0 迟迟不入窗，0.5 s 超时） ─────────────────────
    scenarios.push_back(Scenario{
        "standup_timeout", 6000, false,
        [&](int /*tick*/, float t, TickInputs& in) {
            in.if_enable = t >= 0.0f;
            const Pose away{1.39f + 0.25f, 0.18f};
            if (t < 0.52f) {
                in.pose_left = away;
                in.pose_right = away;
            } else {
                in.pose_left = P.stand_low;
                in.pose_right = P.stand_low;
            }
        }});

    // ── 3. 起身中途无力 ────────────────────────────────────────────────────
    scenarios.push_back(Scenario{
        "standup_disable_midway", 8000, false,
        [&](int /*tick*/, float t, TickInputs& in) {
            in.if_enable = (t < 0.30f) || t >= 0.50f;
            in.pose_left = P.stand_low;
            in.pose_right = P.stand_low;
        }});

    // ── 4. 正常行驶（含启动助力与驻车刹车） ────────────────────────────────
    scenarios.push_back(Scenario{
        "drive", 9000, false,
        [&](int /*tick*/, float t, TickInputs& in) {
            in.if_enable = t >= 0.0f;
            in.pose_left = P.stand_mid;
            in.pose_right = P.stand_mid;
            if (t >= 1.5f && t < 3.0f) {
                in.speed_level[0] = 1.0f;
                in.wheel_speed_l = 33.0f;
                in.wheel_speed_r = -33.0f;
                in.acc_x = 0.0f;
            } else if (t >= 3.0f && t < 4.5f) {
                in.speed_level[0] = -1.0f;
                in.wheel_speed_l = -33.0f;
                in.wheel_speed_r = 33.0f;
            } else {
                in.speed_level[0] = 0.0f;
                in.wheel_speed_l = 0.0f;
                in.wheel_speed_r = 0.0f;
            }
        }});

    // ── 5. 大跳全程（检测禁用：TakeOff 走超时路径） ────────────────────────
    scenarios.push_back(Scenario{
        "jump_big", 9000, false,
        [&](int /*tick*/, float t, TickInputs& in) {
            in.if_enable = t >= 0.0f;
            in.pose_left = P.stand_mid;
            in.pose_right = P.stand_mid;
            static thread_local bool jump_prev = false;
            if (t >= 1.5f && t < 1.502f)
                in.jump_key = true;
            else if (t >= 1.502f)
                in.jump_key = false;
            (void)jump_prev;
            if (t >= 1.51f && t < 1.70f) {
                in.pose_left = P.takeoff;
                in.pose_right = P.takeoff;
                in.leg_torque_l0 = -30.0f;
                in.leg_torque_l1 = -30.0f;
                in.leg_torque_r0 = -30.0f;
                in.leg_torque_r1 = -30.0f;
            } else if (t >= 1.70f && t < 2.1f) {
                in.pose_left = P.stand_low;
                in.pose_right = P.stand_low;
            }
        }});

    // ── 6. 跳跃中途翻车 → 自动自救 → 自救完成 ─────────────────────────────
    scenarios.push_back(Scenario{
        "jump_mid_crash_heal", 14000, false,
        [&](int /*tick*/, float t, TickInputs& in) {
            in.if_enable = t >= 0.0f;
            in.pose_left = P.stand_mid;
            in.pose_right = P.stand_mid;
            if (t >= 1.5f && t < 1.502f)
                in.jump_key = true;
            if (t >= 1.6f && t < 1.75f) {
                in.pose_left = P.takeoff;
                in.pose_right = P.takeoff;
            }
            if (t >= 1.65f && t < 1.85f) {
                in.roll = 1.30f; // 灾难翻转
            }
            // Fallen 2 s → SelfHeal（t≥3.9 前后）；自救：腿缓慢转到 PI/4 收腿位
            if (t >= 4.0f && t < 6.0f) {
                PoseRamp ramp{P.stand_mid, P.heal_done, 4.0f, 6.0f};
                in.pose_left = ramp.at(t);
                in.pose_right = ramp.at(t);
            } else if (t >= 6.0f) {
                in.pose_left = P.heal_done;
                in.pose_right = P.heal_done;
            }
        }});

    // ── 7. 翻车按键自救 ────────────────────────────────────────────────────
    scenarios.push_back(Scenario{
        "fall_key_heal", 12000, false,
        [&](int /*tick*/, float t, TickInputs& in) {
            in.if_enable = t >= 0.0f;
            in.pose_left = P.stand_mid;
            in.pose_right = P.stand_mid;
            if (t >= 1.5f && t < 1.7f)
                in.roll = 1.30f;
            if (t >= 2.0f && t < 2.01f)
                in.save_key = true; // 按键触发自救（不必等 2 s）
            if (t >= 3.0f && t < 5.0f) {
                PoseRamp ramp{P.stand_mid, P.heal_done, 3.0f, 5.0f};
                in.pose_left = ramp.at(t);
                in.pose_right = ramp.at(t);
            } else if (t >= 5.0f) {
                in.pose_left = P.heal_done;
                in.pose_right = P.heal_done;
            }
        }});

    // ── 8. 自救中途无力 ────────────────────────────────────────────────────
    scenarios.push_back(Scenario{
        "heal_disable_midway", 12000, false,
        [&](int /*tick*/, float t, TickInputs& in) {
            in.if_enable = t >= 0.0f;
            in.pose_left = P.stand_mid;
            in.pose_right = P.stand_mid;
            if (t >= 1.5f && t < 1.7f)
                in.roll = 1.30f;
            if (t >= 4.0f && t < 5.5f)
                in.if_enable = false; // 自救中途关
            if (t >= 5.5f && t < 8.0f) {
                PoseRamp ramp{P.stand_mid, P.heal_done, 5.5f, 8.0f};
                in.pose_left = ramp.at(t);
                in.pose_right = ramp.at(t);
            } else if (t >= 8.0f) {
                in.pose_left = P.heal_done;
                in.pose_right = P.heal_done;
            }
        }});

    // ── 9. 小陀螺进出 ──────────────────────────────────────────────────────
    scenarios.push_back(Scenario{
        "spin", 8000, false,
        [&](int /*tick*/, float t, TickInputs& in) {
            in.if_enable = t >= 0.0f;
            in.pose_left = P.stand_mid;
            in.pose_right = P.stand_mid;
            if (t >= 1.5f && t < 3.0f) {
                in.speed_level[2] = 1.0f; // dial 档 1（xy 为 0，无 0.5 修正）
                in.dyaw = 6.0f;
                in.yaw = 6.0f * (t - 1.5f);
                in.total_yaw = in.yaw;
            } else if (t >= 3.0f) {
                in.speed_level[2] = 0.0f;
                in.dyaw = 0.0f;
            }
        }});

    // ── 10. 上台阶（HIGH 档折叠 → UpStair → SlowStart） ────────────────────
    scenarios.push_back(Scenario{
        "up_stair", 12000, false,
        [&](int /*tick*/, float t, TickInputs& in) {
            in.if_enable = t >= 0.0f;
            in.pose_left = P.stand_high;
            in.pose_right = P.stand_high;
            if (t >= 1.0f && t < 1.001f)
                in.leg_key = true; // LOW→MID
            if (t >= 1.2f && t < 1.201f)
                in.leg_key = true; // MID→HIGH
            if (t >= 2.0f && t < 3.0f) {
                in.pose_left = P.folded;
                in.pose_right = P.folded;
            }
            if (t >= 3.2f && t < 4.0f) {
                PoseRamp ramp{P.folded, P.up_stair_done, 3.2f, 4.0f};
                in.pose_left = ramp.at(t);
                in.pose_right = ramp.at(t);
            } else if (t >= 4.0f) {
                in.pose_left = P.up_stair_done;
                in.pose_right = P.up_stair_done;
            }
        }});

    // ── 11. 飞坡循环（启用离地检测：零力 → 离地 → FLY 超时 → NORMAL 循环） ──
    scenarios.push_back(Scenario{
        "fly_cycle", 12000, true,
        [&](int /*tick*/, float t, TickInputs& in) {
            in.if_enable = t >= 0.0f;
            in.pose_left = P.stand_mid;
            in.pose_right = P.stand_mid;
            in.leg_torque_l0 = 0.0f; // P=0 → 双腿"离地"
            in.leg_torque_l1 = 0.0f;
            in.leg_torque_r0 = 0.0f;
            in.leg_torque_r1 = 0.0f;
        }});

    // ── 12. 跳跃蹬伸→腾空（启用检测：TakeOff→Flying 走真实离地路径） ────────
    scenarios.push_back(Scenario{
        "jump_airborne", 9000, true,
        [&](int /*tick*/, float t, TickInputs& in) {
            in.if_enable = t >= 0.0f;
            in.pose_left = P.stand_mid;
            in.pose_right = P.stand_mid;
            in.leg_torque_l0 = 0.0f;
            in.leg_torque_l1 = 0.0f;
            in.leg_torque_r0 = 0.0f;
            in.leg_torque_r1 = 0.0f;
            if (t >= 1.5f && t < 1.502f)
                in.jump_key = true;
            if (t >= 1.55f && t < 1.75f) {
                in.pose_left = P.takeoff;
                in.pose_right = P.takeoff;
            }
        }});

    return scenarios;
}

} // namespace hcs_core::controller::balance::port
