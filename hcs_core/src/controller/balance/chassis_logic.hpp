#pragma once

// 平衡底盘逻辑层组合：估计器 →（安全检查）→ 阶段机 → 接触判定 → 控制器。
//
// 每拍顺序与 Helios ChassisTask 逐段对应（PORTING.md §7.2）：
//   1. update_kinematics      —— v2 Chassis_Data_Update
//   2. machine.update         —— v2 FSM()（含安全判定的消费位置）
//   3. update_contact         —— v2 Gnd_Off_Detect + 写回（FSM 读上一拍，保持原数据流）
//   4. controller.update      —— v2 Chassis_Controller(_Update) + Motor_Send
// 本类不依赖 rclcpp / libhcs，对拍 harness 与组件壳共用同一实现。

#include "controller.hpp"
#include "estimator.hpp"
#include "operator_input.hpp"
#include "params.hpp"
#include "phase_machine.hpp"
#include "safety.hpp"
#include "types.hpp"

namespace hcs_core::controller::balance {

class ChassisLogic {
public:
    explicit ChassisLogic(const Params& params)
        : params_(params)
        , estimator_(params)
        , safety_(params)
        , machine_(params, estimator_, safety_)
        , controller_(params, estimator_) {}

    void update(const LogicInput& input, float dt) {
        estimator_.update_kinematics(input, dt);
        view_ = machine_.update(input.command, dt);
        estimator_.update_contact(
            {.normal_init_lock = machine_.standing_lock_active(),
             .leg_change_lock = machine_.leg_change_lock_active()});

        const bool jump_step_changed = jump_step_of(prev_phase_) != jump_step_of(machine_.phase());
        controller_.update(
            estimator_.estimate(), view_, machine_.phase(), jump_step_changed, dt, tick_++);
        prev_phase_ = machine_.phase();
    }

    [[nodiscard]] const Estimate& estimate() const {
        return estimator_.estimate();
    }
    [[nodiscard]] const MachineView& view() const {
        return view_;
    }
    [[nodiscard]] const Phase& phase() const {
        return machine_.phase();
    }
    [[nodiscard]] const MotorCommands& commands() const {
        return controller_.commands();
    }
    [[nodiscard]] const BalanceController::Telemetry& telemetry() const {
        return controller_.telemetry();
    }
    [[nodiscard]] const BalanceNLMPC::Diagnostics& nlmpc_diagnostics() const {
        return controller_.nlmpc_diagnostics();
    }

    static int jump_step_of(const Phase& phase) {
        const auto* jump = std::get_if<phase::Jump>(&phase);
        return jump ? static_cast<int>(jump->step) : -1;
    }

private:
    const Params& params_;
    Estimator estimator_;
    SafetyChecker safety_;
    PhaseMachine machine_;
    BalanceController controller_;

    MachineView view_{};
    Phase prev_phase_{phase::Disabled{}};
    uint64_t tick_ = 0;
};

} // namespace hcs_core::controller::balance
