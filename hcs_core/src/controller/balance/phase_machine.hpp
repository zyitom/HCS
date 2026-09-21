#pragma once

// 阶段状态机（任务书架构决策 B）。
//
// 阶段用 std::variant 表达，全组件只有本文件的 update() 决定阶段转移；全局规则
// （无力、翻车）写在最前。转移条件、执行顺序与 Helios v2 FSM()（1232-2295）逐段
// 对应——包括"同一拍内后写的覆盖先写的"这一原始优先级语义，行为对拍以此为基线。
// 与 v2 的显式差异（死状态 TOUCH_DOWN/TANK、NO_HEAD 路径、IBC 节流、DISABLED 双义
// 拆分为 Disabled/Fallen）见 PORTING.md §9。
//
// v2 藏在枚举外的状态全部并入阶段自带数据或本类上下文（PORTING.md §5）：
//   is_healing_active/init_flag → Fallen/SelfHeal 阶段本身；
//   slow_start_phase/cnt/fail_wait_cnt → phase::SlowStart；
//   jump_* → phase::Jump；heal_* → phase::SelfHeal；
//   normal_init_lock_cnt → phase::Standing::elapsed_s（3 s 站立锁，见 §9-2 注）；
//   fatal_error_timer → phase::Fallen::elapsed_s；
//   fly_init_swing_single → phase::Fly/Jump::locked_swing。

#include <variant>

#include "estimator.hpp"
#include "filters.hpp"
#include "math.hpp"
#include "params.hpp"
#include "safety.hpp"
#include "types.hpp"

namespace hcs_core::controller::balance {

namespace phase {

struct Disabled {};

struct SlowStart {
    enum class Step : uint8_t { Retract, Rotate };
    Step step = Step::Retract;
    float stable_s = 0.0f;    // 转腿阶段姿态稳定计时（v2 cnt，>0.2 s 完成）
    float elapsed_s = 0.0f;   // v2 fail_wait_cnt（>0.5 s 超时完成）
    float pose_target = 1.39f;
};

struct Standing {
    float elapsed_s = 0.0f; // 站立计时，同时承担 3 s 离地锁（v2 normal_init_lock_cnt）
};

/// 自动检测进入：NORMAL + HIGH 档 + 双腿折叠趋势（v2 up_stair_cnt）。
struct UpStair {
    float stable_s = 0.0f; // v2 up_stair_timer（摆角到位 0.1 s → 回 SlowStart）
};

struct Jump {
    enum class Step : uint8_t { Press, TakeOff, Flying, Landing };
    Step step = Step::Press;
    float step_elapsed_s = 0.0f; // v2 phase_timeout_cnt
    float confirm_s = 0.0f;      // v2 jump_leg_cnt/fly_touch_cnt/land_short_cnt
    int level = 2;               // v2 jump_level_latched（当前构建恒 2）
    float locked_swing = 0.0f;   // 起跳瞬间锁定的腿摆角（v2 fly_init_swing_single）
};

struct Fly {
    float elapsed_s = 0.0f;
    float locked_swing = 0.0f; // 起飞瞬间锁定的腿摆角，只存在于 Fly（任务书骨架）
};

struct Fallen {
    float elapsed_s = 0.0f; // v2 fatal_error_timer（2 s → SelfHeal）
};

struct SelfHeal {
    float elapsed_s = 0.0f;
    int dir = 1;                  // v2 heal_dir
    float stall_s = 0.0f;         // v2 heal_stall_cnt
    float cooldown_s = 0.0f;      // v2 heal_flip_cooldown
    float phi0_total_last[2]{};   // v2 phi0_total_last
    bool healing = false;         // v2 state_flag==HEALING（双腿展开后）
    bool done = false;            // 自救完成（v2 清 fatal/heal 标志），停留到下一拍
};

} // namespace phase

using Phase = std::variant<
    phase::Disabled, phase::SlowStart, phase::Standing, phase::UpStair, phase::Jump,
    phase::Fly, phase::Fallen, phase::SelfHeal>;

/// 阶段 → 控制器结构。std::visit 穷举，漏写一个阶段编译失败（任务书骨架）。
/// （UpStair 相位 2 与 Fallen 的瞬时 Disabled 由阶段机的 override 表达。）
ProcessFlag controller_for(const Phase& phase);

PhaseKind kind_of(const Phase& phase);

class PhaseMachine {
public:
    PhaseMachine(const Params& params, Estimator& estimator, SafetyChecker& safety)
        : params_(params)
        , estimator_(estimator)
        , safety_(safety) {}

    /// 一拍推进。返回本拍机器视图（控制器消费）；阶段与上下文在内部演化。
    MachineView update(const OperatorCommand& command, float dt);

    [[nodiscard]] const Phase& phase() const {
        return current_;
    }
    [[nodiscard]] const Phase& phase_past() const {
        return past_;
    }

    /// 估计器 update_contact 的门控输入（站立 3 s 离地锁）。
    [[nodiscard]] bool standing_lock_active() const {
        return standing_lock_active_;
    }
    [[nodiscard]] bool leg_change_lock_active() const {
        return leg_change_lock_s_ > 0.0f;
    }

    /// 阶段机写出的斜坡目标（供底盘控制器外的遥测/发布用）。
    [[nodiscard]] const std::array<float, 2>& ll_want_target() const {
        return ll_want_target_;
    }
    [[nodiscard]] const std::array<float, 2>& rotate_want_target() const {
        return rotate_want_target_;
    }

private:
    [[nodiscard]] float rotate_handle(std::size_t leg, float round_count, float single_angle);
    [[nodiscard]] bool pose_stable_for_jump() const;

    const Params& params_;
    Estimator& estimator_;
    SafetyChecker& safety_;

    Phase current_{phase::Disabled{}};
    Phase past_{phase::Disabled{}};

    // ── 机器上下文（v2 FSM 的非 static 成员与贯穿状态） ─────────────────────
    std::array<RampFunction, 2> ramp_leg_len_{};
    std::array<RampFunction, 2> ramp_leg_rotate_{};
    std::array<float, 2> ll_want_target_{};
    std::array<float, 2> rotate_want_target_{};

    ProcessFlag process_flag_ = ProcessFlag::Disabled;
    ProcessFlag process_flag_past_ = ProcessFlag::Disabled;
    bool flag_disabled_override_ = false; // UpStair 相位 2 的一拍 DISABLED

    int ll_state_ = 0; // LL_Discribe_e：0 LOW 1 MID 2 HIGH
    int stand_last_ll_state_ = 0;
    int spin_last_ll_state_ = 0;

    float rotate_speed_ = 1.0f; // v2 rotate_speed（NORMAL 继承前值的怪癖，原样保留）
    float l0_speed_ = 0.15f;
    float v_yaw_ = 0.0f;        // v2 robot_state.v[2]（只在 SPIN/Disabled 更新的怪癖）
    float v_target_ = 0.0f;
    float v_raw_target_ = 0.0f;
    float acc_fwd_ = 0.0f;
    float dial_level_ = 0.0f;
    float follow_angle_ = 0.0f;
    bool if_follow_ = false;
    bool is_inversed_ = false;
    bool spin_active_ = false;
    bool spin_active_past_ = false;
    bool jumping_ = false;      // 本拍是否处于 Jump（jump_phase_cnt != COMPLETE 的等价）

    float up_stair_cnt_ = 0.0f;
    float leg_change_lock_s_ = 0.0f;
    float fly_enter_accum_ = 0.0f;
    bool standing_lock_active_ = false;
    bool self_heal_healing_past_ = false;

    Td td_input_{};   // v2 td_input[0]：速度指令 TD
    Td td_spin_{};    // v2 td_input[1]：小陀螺转速 TD
};

} // namespace hcs_core::controller::balance
