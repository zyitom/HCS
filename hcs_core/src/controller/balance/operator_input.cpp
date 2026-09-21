#include "operator_input.hpp"

#include <cmath>

namespace hcs_core::controller::balance {

int BalanceOperatorInput::stick_level(float value) const {
    if (value > params_.stick_high_ratio)
        return 2;
    if (value > params_.stick_low_ratio)
        return 1;
    if (value < -params_.stick_high_ratio)
        return -2;
    if (value < -params_.stick_low_ratio)
        return -1;
    return 0;
}

OperatorCommand BalanceOperatorInput::update(const RemoteSnapshot& remote, float /*dt*/) {
    OperatorCommand out{};

    // ── 按钮 → 开关（按一下翻转一次，fsm.cpp 键盘开关语义） ─────────────────
    if (remote.left_button && !left_button_last_)
        jump_toggle_ = !jump_toggle_;
    left_button_last_ = remote.left_button;

    if (remote.pause_button && !pause_button_last_)
        save_toggle_ = !save_toggle_;
    pause_button_last_ = remote.pause_button;

    if (remote.right_button && !right_button_last_)
        leg_toggle_ = !leg_toggle_;
    right_button_last_ = remote.right_button;

    if (remote.trigger && !trigger_last_)
        turn_toggle_ = !turn_toggle_;
    trigger_last_ = remote.trigger;

    // ── 摇杆 → 档位（fsm.cpp setSpeed + dial 0.5 修正） ─────────────────────
    out.speed_level[0] = static_cast<float>(stick_level(remote.stick_left_x));
    out.speed_level[1] = static_cast<float>(stick_level(remote.stick_left_y));
    float w_level = static_cast<float>(stick_level(remote.dial));
    if (w_level != 0.0f && (out.speed_level[0] != 0.0f || out.speed_level[1] != 0.0f)) {
        w_level -= 0.5f * w_level / std::fabs(w_level);
    }
    out.speed_level[2] = w_level;

    // if_free 与小陀螺联动（fsm.cpp:148-151）
    out.if_free = w_level != 0.0f;

    // ── 使能（默认映射：中/运动挡使能，电影挡与未知失能；§10-7） ────────────
    out.if_enable = remote.valid && (remote.mode_switch == 2 || remote.mode_switch == 3);

    // ── 开关电平与边沿（v2 FSM 内联边沿检测的等价迁移） ─────────────────────
    out.jump_rising_edge = jump_toggle_ && !jump_level_last_;
    out.save_changed = save_toggle_ != save_level_last_;
    out.leg_changed = leg_toggle_ != leg_level_last_;
    out.save_level = save_toggle_;
    out.if_turn = turn_toggle_;
    jump_level_last_ = jump_toggle_;
    save_level_last_ = save_toggle_;
    leg_level_last_ = leg_toggle_;

    return out;
}

} // namespace hcs_core::controller::balance
