#pragma once

// 安全检查层（Cheetah SafetyChecker 式）：持有跨拍去抖计数器，输出"已确认"的判定，
// 供阶段机在固定位置消费。判定阈值与 Helios v2 的 disaster/fatal/crash/impact 一致。
//
// 注意：这些计数器的判定依赖当前阶段（is_checking_mode / is_in_active_mode），
// 所以 update() 由阶段机在 Helios 的对应位置调用，传入阶段上下文。

#include <cmath>

#include "leg_model.hpp"
#include "math.hpp"
#include "params.hpp"
#include "types.hpp"

namespace hcs_core::controller::balance {

enum class PhaseKind : uint8_t {
    Disabled,
    SlowStart,
    Standing,
    UpStair,
    Jump,
    Fly,
    Fallen,
    SelfHeal,
};

struct SafetyView {
    bool impact_lock_active{};  // |a_x| 阈值锁存期内（FLY 进入门控）
    bool reliable_touch_down{}; // 双腿 L0 < 阈值持续超时（FLY 退出通道 1）
    bool disaster{};            // 瞬时翻转（roll/pitch > 1.2）
    bool fatal_accumulated{};   // 累积翻车（>0.90/0.82 计 50ms，衰减制）
    bool crash{};               // 摔跤（pitch 或劈叉，计 100ms）→ 回 SlowStart
};

class SafetyChecker {
public:
    explicit SafetyChecker(const Params& params)
        : params_(params) {}

    /// 每拍开头调用（v2:1397-1408）：|a_x| 冲击锁存。
    void update_impact_lock(float a_x, float dt) {
        if (std::fabs(a_x) > params_.impact_ax_thres) {
            impact_lock_s_ = params_.impact_lock_s;
        } else if (impact_lock_s_ > 0.0f) {
            impact_lock_s_ -= dt;
        }
    }

    /// 每拍调用（v2:1421-1431）：双腿短腿计数（FLY 退出通道）。
    void update_touch_down(const Estimate& est, float dt) {
        const bool both_legs_short = est.leg[kLeft].L0 < params_.fly_exit_ll_thres
                                   && est.leg[kRight].L0 < params_.fly_exit_ll_thres;
        if (both_legs_short) {
            fly_exit_short_leg_s_ += dt;
        } else {
            fly_exit_short_leg_s_ = 0.0f;
        }
    }

    /// v2:1494-1531：灾难瞬时检测 + 累积翻车。phase_kind / is_self_heal 由阶段机给。
    void update_fatal(
        const Estimate& est, PhaseKind phase_kind, const std::array<float, 2>& phi0_total,
        bool in_active_mode, float dt) {
        const auto& wbc = est.wbc;

        disaster_ = (std::fabs(wbc.roll) > params_.disaster_roll_threshold
                     || std::fabs(wbc.thetab) > params_.disaster_pitch_threshold)
                 && phase_kind != PhaseKind::SelfHeal && phase_kind != PhaseKind::Disabled;

        const bool is_fatal_roll = std::fabs(wbc.roll) > params_.fatal_roll_threshold;
        const bool is_fatal_pitch = std::fabs(wbc.thetab) > params_.fatal_pitch_threshold;
        // v2:1508-1509 —— FLY 属于检查模式（与 DISABLED/SELF_HEAL 相对）。
        const bool is_checking_mode = phase_kind != PhaseKind::Disabled
                                   && phase_kind != PhaseKind::SelfHeal;

        if (is_checking_mode && (is_fatal_roll || is_fatal_pitch)) {
            fatal_err_cnt_s_ += dt;
        } else {
            fatal_err_cnt_s_ = fatal_err_cnt_s_ > params_.fatal_decay_per_tick_s
                                 ? fatal_err_cnt_s_ - params_.fatal_decay_per_tick_s
                                 : 0.0f;
        }
        fatal_ = fatal_err_cnt_s_ > params_.fatal_accum_s && is_checking_mode;
        if (fatal_) {
            fatal_err_cnt_s_ = 0.0f;
        }

        // v2:1574-1607：摔跤（Path C）→ SlowStart，不算 fatal。
        const float crash_pitch_threshold =
            phase_kind == PhaseKind::UpStair ? params_.crash_pitch_threshold_up_stair
                                             : params_.crash_pitch_threshold;
        const bool is_pitch_crashed = std::fabs(wbc.thetab) > crash_pitch_threshold;
        const bool is_touching_ground = est.ground.if_off_gnd[kLeft] != 1
                                     || est.ground.if_off_gnd[kRight] != 1;

        float phi0_diff = phi0_total[kLeft] - phi0_total[kRight];
        while (phi0_diff > kPi) phi0_diff -= 2.0f * kPi;
        while (phi0_diff < -kPi) phi0_diff += 2.0f * kPi;
        const bool is_fatal_split = std::fabs(phi0_diff) > params_.fatal_split_threshold;

        if (((is_pitch_crashed && is_touching_ground) || is_fatal_split) && in_active_mode
            && !fatal_) {
            crash_cnt_s_ += dt;
        } else {
            crash_cnt_s_ = 0.0f;
        }
        crash_ = crash_cnt_s_ > params_.crash_s;
        if (crash_) {
            crash_cnt_s_ = 0.0f;
        }
    }

    [[nodiscard]] SafetyView view() const {
        return SafetyView{
            .impact_lock_active = impact_lock_s_ > 0.0f,
            .reliable_touch_down = fly_exit_short_leg_s_ > params_.fly_exit_short_s,
            .disaster = disaster_,
            .fatal_accumulated = fatal_,
            .crash = crash_,
        };
    }

private:
    const Params& params_;

    float impact_lock_s_ = 0.0f;
    float fly_exit_short_leg_s_ = 0.0f;
    float fatal_err_cnt_s_ = 0.0f;
    float crash_cnt_s_ = 0.0f;

    bool disaster_ = false;
    bool fatal_ = false;
    bool crash_ = false;
};

} // namespace hcs_core::controller::balance
