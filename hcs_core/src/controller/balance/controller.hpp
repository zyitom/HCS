#pragma once

// 控制器层：LQR 状态向量装配、NLMPC/fallback、roll MPC/PID、腿长与摆角 PID、
// 柔顺权重、VMC、力矩→电机命令映射（v2 Chassis_Controller_Update / Chassis_Controller /
// Motor_Send 的逐语义移植；LESO 与功率 QP/RLS 是 Helios 的死代码，不移植，见 §9-4/5）。

#include <array>

#include "filters.hpp"
#include "leg_model.hpp"
#include "math.hpp"
#include "nlmpc.hpp"
#include "params.hpp"
#include "pid.hpp"
#include "phase_machine.hpp"
#include "types.hpp"

namespace hcs_core::controller::balance {

class BalanceController {
public:
    BalanceController(const Params& params, Estimator& estimator)
        : params_(params)
        , estimator_(estimator)
        , pid_ll_(params.leg_l_pid)
        , pid_lr_(params.leg_l_pid)
        , pid_leg_rotate_l_(params.leg_rotate_pid)
        , pid_leg_rotate_r_(params.leg_rotate_pid)
        , nlmpc_()
        , roll_mpc_() {}

    /// 一拍控制。jump_step_changed 由底盘逻辑层用上一拍阶段给出。
    void update(
        const Estimate& est, const MachineView& view, const Phase& phase, bool jump_step_changed,
        float dt, uint64_t tick);

    [[nodiscard]] const MotorCommands& commands() const {
        return commands_;
    }

    /// 遥测：最终轮力矩与关节力矩（限幅后、映射前），供上层发布。
    struct Telemetry {
        float final_tw[2]{};
        float final_tl0[2]{};
        float final_tl1[2]{};
        float f_want[2]{};
    };
    [[nodiscard]] const Telemetry& telemetry() const {
        return telemetry_;
    }

    [[nodiscard]] const BalanceNLMPC::Diagnostics& nlmpc_diagnostics() const {
        return nlmpc_.diagnostics();
    }

    void reset_wheel_pid_i() {
        pid_wheel_current_l_.clear_i();
        pid_wheel_current_r_.clear_i();
    }

    void reset_kf() {
        estimator_.reset_kf();
    }

private:
    /// v2 Chassis_Controller_Update 的状态向量装配与 process_flag 分发。
    void controller_update(
        const Estimate& est, const MachineView& view, const Phase& phase, float dt,
        uint64_t tick);
    /// v2 Chassis_Controller 的模式分发 + VMC。
    void controller_dispatch(
        const Estimate& est, const MachineView& view, const Phase& phase,
        bool jump_step_changed, float dt, uint64_t tick);
    /// v2 Motor_Send。
    void motor_send(const Estimate& est, PhaseKind mode, float dt, uint64_t tick);

    static float smooth_saturation(float x, float width, float smoothness) {
        if (std::fabs(x) < width)
            return 1.0f;
        return std::exp(-smoothness * std::pow(std::fabs(x) - width, 2));
    }

    [[nodiscard]] float torque_to_current(float torque) const;

    const Params& params_;
    Estimator& estimator_;

    Pid pid_ll_;
    Pid pid_lr_;
    Pid pid_leg_rotate_l_;
    Pid pid_leg_rotate_r_;
    Pid pid_wheel_current_l_{params_.wheel_current_pid};
    Pid pid_wheel_current_r_{params_.wheel_current_pid};

    BalanceNLMPC nlmpc_;
    BalanceRollMPC roll_mpc_;

    // 滤波器与 TAKE_OFF 状态（v2 文件级/函数级 static → 成员）
    FirstOrderFilter filter_roll_{0.5672f, 0.4328f};
    FirstOrderFilter filter_roll1_{0.5672f, 0.4328f};
    FirstOrderFilter filter_takeoff_acc_err_d_{0.4700f, 0.5300f};
    float takeoff_acc_err_last_ = 0.0f;

    std::array<std::array<float, 10>, 4> k_matrix_{};
    std::array<float, 10> status_vector_{};

    bool if_jump_controller_enable_ = false;
    float compliance_weight_[2] = {1.0f, 1.0f};
    float f_roll_add_ = 0.0f;
    float stab_roll_ = 0.0f;
    float yaw_rate_cmd_limited_ = 0.0f;
    float smoothed_yaw_error_ = 0.0f;
    float landing_recovery_factor_ = 1.0f;
    float adapt_leg_angle_preset_latched_ = 0.0f;
    bool adapt_leg_angle_preset_latched_valid_ = false;

    // 驻车刹车与启动助力（v2 park_brake_*）
    enum class ParkBrake : uint8_t { Off, Waiting, Engaged };
    ParkBrake park_brake_state_ = ParkBrake::Off;
    float park_s_locked_ = 0.0f;
    float pos_filter_state_ = 0.0f;

    struct LegOutput {
        float tw{}, tl{}, f_want{}, final_tw{}, final_tl0{}, final_tl1{};
    };
    std::array<LegOutput, 2> output_{};
    std::array<LegOutput, 2> output_last_{};

    MotorCommands commands_;
    Telemetry telemetry_{};
};

} // namespace hcs_core::controller::balance
