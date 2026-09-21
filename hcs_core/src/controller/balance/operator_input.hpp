#pragma once

// 操作输入层（架构决策 A）：遥控器 → 操作意图。
//
// 替代 Helios 云台板 module::FSM（遥控翻译 + 开关）+ IBC 通道。与原实现的对应关系：
//   - setSpeed 摇杆阈值（±660 的 100/400）→ 满幅比例阈值（params.stick_*_ratio）；
//   - dial 的 0.5 修正（xy 运动时取消小陀螺）原样保留；
//   - save/leg/jump 在原实现里是键盘开关（电平翻转），这里由遥控自定义按钮翻转，
//     边沿检测在本层完成（原实现分散在底盘 FSM 的 static 里）；
//   - VT13 没有云台三挡开关的语义，mode_switch 的映射是默认设计（§10-7，可改）。

#include "filters.hpp"
#include "params.hpp"
#include "types.hpp"

namespace hcs_core::controller::balance {

/// 硬件组件从遥控接收机（VT13）归一化出的快照。
struct RemoteSnapshot {
    float stick_left_x{}, stick_left_y{}; // -1..1
    float dial{};                          // -1..1
    int mode_switch{};                     // hcs_msgs::Switch 值域：0 未知 1 上 2 下 3 中
    bool pause_button{};
    bool left_button{};
    bool right_button{};
    bool trigger{};
    bool valid{};
};

class BalanceOperatorInput {
public:
    explicit BalanceOperatorInput(const Params& params)
        : params_(params) {}

    OperatorCommand update(const RemoteSnapshot& remote, float dt);

private:
    [[nodiscard]] int stick_level(float value) const;

    const Params& params_;

    // 按钮开关状态（fsm.cpp 的键盘开关语义：按一下翻转一次）
    bool jump_toggle_ = false;
    bool save_toggle_ = false;
    bool leg_toggle_ = false;
    bool turn_toggle_ = false;

    bool left_button_last_ = false;
    bool pause_button_last_ = false;
    bool right_button_last_ = false;
    bool trigger_last_ = false;

    bool jump_level_last_ = false;
    bool save_level_last_ = false;
    bool leg_level_last_ = false;
};

} // namespace hcs_core::controller::balance
