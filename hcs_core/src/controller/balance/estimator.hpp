#pragma once

// 估计器层：腿运动学、接触力、弹簧力、多圈、速度 KF、姿态映射、离地判定。
// 所有去抖计数器住在这里（PORTING.md §5）。
//
// 与 Helios 的执行顺序差异（有意保留，行为一致）：
//   Helios 在 Chassis_Data_Update 里做运动学/力/姿态/KF，在 Chassis_Controller_Update
//   开头（FSM 之后）才调 Gnd_Off_Detect 并写回 if_off_gnd —— 因此 FSM 读到的接触
//   状态是上一拍的。本层拆成 update_kinematics()（拍首）与 update_contact()（阶段机
//   之后、控制器之前）两个入口，正是为了逐拍复刻这条数据流。

#include <array>

#include "filters.hpp"
#include "leg_model.hpp"
#include "math.hpp"
#include "params.hpp"
#include "speed_kf.hpp"
#include "types.hpp"

namespace hcs_core::controller::balance {

class Estimator {
public:
    explicit Estimator(const Params& params)
        : params_(params) {}

    /// 拍首：IMU → 姿态/加速度，电机 → 腿运动学/接触力/多圈/速度 KF。
    /// 对应 v2 Chassis_Data_Update（104-198）+ SpeedEstimation（201-215）。
    void update_kinematics(const LogicInput& input, float dt);

    /// 阶段机之后：Gnd_Off_Detect + 门控写回（v2:545-546 与 2492-2497）。
    /// gating 输入：TOUCH_DOWN 已删（死状态），normal_init_lock 由 Standing 计时给出，
    /// leg_change_lock 由阶段机给出，腿角有效性与站立预设有关（v2:2494）。
    struct ContactGate {
        bool normal_init_lock{};
        bool leg_change_lock{};
    };
    void update_contact(const ContactGate& gate);

    [[nodiscard]] const Estimate& estimate() const {
        return estimate_;
    }

    /// 供安全检查/阶段机用的 wbc 可写视图（LL_want/rotate_angle 由阶段机经斜坡写入）。
    WbcState& wbc() {
        return estimate_.wbc;
    }

    /// 速度 KF 复位（v2 module::reset_kf 的调用点语义：阶段/模式切换处调用）。
    void reset_kf() {
        speed_kf_.reset();
    }

    void reset_kf_stall() {
        speed_kf_.reset_stall();
    }

    [[nodiscard]] bool wheel_stall(std::size_t i) const {
        return i == kLeft ? speed_kf_.wheel_stall_left() : speed_kf_.wheel_stall_right();
    }

private:
    [[nodiscard]] int gnd_off_detect(std::size_t leg);
    [[nodiscard]] float speed_estimation(float dt);

    const Params& params_;
    Estimate estimate_{};

    // ── 滤波器（v2 文件级 static → 成员） ──────────────────────────────────
    FirstOrderFilter filter_fn_[2]{{0.69f, 0.31f}, {0.69f, 0.31f}};
    FirstOrderFilter filter_ddphi0_[2]{{0.15f, 0.85f}, {0.15f, 0.85f}};

    // ── 多圈检测私有量（v2 leg_state 内的 round_count/phi0_last/first_time） ─
    struct MultiTurn {
        float round_count{};
        float phi0_last{};
        bool first_time = true;
    };
    std::array<MultiTurn, 2> multi_turn_{};

    // ── Gnd_Off_Detect 私有量（v2 函数内 static → 成员；当前整体被禁用） ────
    struct GndDetect {
        int state[2]{};
        uint32_t impact_timer[2]{};
        float p_last[2]{};
        bool d_p_init[2]{};
        float d_p_buf[2][4]{};
        int d_p_idx[2]{};
        float ll_want_last[2]{};
    };
    GndDetect gnd_{};

    SpeedKf speed_kf_;
};

} // namespace hcs_core::controller::balance
