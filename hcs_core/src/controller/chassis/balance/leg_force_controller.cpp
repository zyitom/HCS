#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "controller/chassis/balance/balance_types.hpp"
#include "controller/chassis/balance/nlmpc_solver.hpp"

namespace hcs_core::controller::chassis::balance {
using hcs_core::controller::pid::PidCalculator;

// ============================================================================
// 腿力控制器（对应 Helios Chassis_Controller 里 F_want 的全部组成）：
//   - 腿长 PID（leg_L_pid：Kp 2800 / Ki 0 / Kd 60 / 限幅 ±600），测量 = 当前腿长；
//   - 重力前馈 F_ff = −M_t·G/2（M_t = 机体 + 两条腿的完整质量）；
//   - 滚转镇定：滚转 MPC（主）或 stab_roll 积分 + 变 Kd（回退），差动力
//     一腿加一腿减，回退部分同时偏移腿长目标；
//   - 起跳加速度 PD（JUMP/TAKE_OFF：az_ref = 20，kp 5.6 / kd 0.4，
//     误差先经 47%/53% 一阶滤波）；
//   - 柔顺权重（仅 FLY 模式按离地状态降权，跳腾空段不乘）；
//   - 弹簧力补偿：VMC 的 F 输入 = F_want + 弹簧力（均在权重过滤后相加）。
// 输出 control_force（VMC 的 F 输入）与 hip_torque_compensation（起跳俯仰
// 补偿，叠加到 LQR 的髋力矩上）。
// ============================================================================

class LegForceController
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    LegForceController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        register_input("/chassis/balance/mode", mode_);
        register_input("/chassis/balance/jump_phase", jump_phase_);
        register_input("/chassis/left_leg/control_length", left_control_length_);
        register_input("/chassis/right_leg/control_length", right_control_length_);
        register_input("/chassis/left_leg/length", left_length_);
        register_input("/chassis/right_leg/length", right_length_);
        register_input("/chassis/left_leg/spring_force", left_spring_force_);
        register_input("/chassis/right_leg/spring_force", right_spring_force_);
        register_input("/chassis/left_leg/grounded", left_grounded_);
        register_input("/chassis/right_leg/grounded", right_grounded_);
        register_input("/chassis/balance/roll", body_roll_);
        register_input("/chassis/balance/roll_rate", body_roll_rate_);
        register_input("/chassis/balance/yaw_rate", body_yaw_rate_);
        register_input("/chassis/balance/acceleration_vertical", acceleration_vertical_);

        register_output("/chassis/left_leg/control_force", left_control_force_, nan_);
        register_output("/chassis/right_leg/control_force", right_control_force_, nan_);
        register_output(
            "/chassis/left_leg/hip_torque_compensation", left_hip_compensation_, 0.0);
        register_output(
            "/chassis/right_leg/hip_torque_compensation", right_hip_compensation_, 0.0);

        read_parameters();
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        const double dt = tick.dt_seconds();
        const auto mode = static_cast<Mode>(*mode_);
        const auto jump_phase = static_cast<JumpPhase>(*jump_phase_);

        // 失能/摔倒：支撑力必须清零（Helios 里靠 Motor_Send 的 DISABLED 分支
        // 兜底把关节命令清零；组件化之后力从这里出口，没人再兜底，必须自己清），
        // 同时清腿长 PID 积分，对齐 Helios DISABLED 分支的 PIDClearI。
        if (mode == Mode::kDisabled || mode == Mode::kFallen) {
            left_length_pid_.reset();
            right_length_pid_.reset();
            *left_control_force_ = 0.0;
            *right_control_force_ = 0.0;
            *left_hip_compensation_ = 0.0;
            *right_hip_compensation_ = 0.0;
            return;
        }

        const double total_mass = body_mass_ + 2.0 * (wheel_mass_ + leg_mass_);
        const double gravity_feedforward = -total_mass * kGravity / 2.0;

        // 跳跃分支的 F_want（Helios Chassis_Controller 262-341）。
        bool jump_force_controller = false;
        double left_force = 0.0;
        double right_force = 0.0;

        switch (mode) {
        case Mode::kJump:
            switch (jump_phase) {
            case JumpPhase::kTakeOff: {
                jump_force_controller = true;
                const double acceleration_error = takeoff_acceleration_ - *acceleration_vertical_;
                if (!takeoff_error_initialized_) {
                    takeoff_error_initialized_ = true;
                    takeoff_error_last_ = acceleration_error;
                }
                const double error_derivative_raw =
                    (acceleration_error - takeoff_error_last_) / dt;
                takeoff_error_filter_ += 0.47 * (error_derivative_raw - takeoff_error_filter_);
                takeoff_error_last_ = acceleration_error;

                double add_force_total = total_mass
                    * (takeoff_kp_ * acceleration_error + takeoff_kd_ * takeoff_error_filter_);
                add_force_total = std::clamp(add_force_total, 0.0, total_mass * kGravity);
                const double add_force_leg = 0.5 * add_force_total;
                left_force = gravity_feedforward - add_force_leg;
                right_force = gravity_feedforward - add_force_leg;

                // 起跳俯仰补偿（Tl_comp = F_total·CoM_x·0.85）。
                const double left_total = left_force + *left_spring_force_;
                const double right_total = right_force + *right_spring_force_;
                *left_hip_compensation_ = left_total * com_x(*left_length_) * 0.85;
                *right_hip_compensation_ = right_total * com_x(*right_length_) * 0.85;
                break;
            }
            case JumpPhase::kFlying:
                jump_force_controller = true;
                left_force = 0.0;
                right_force = 0.0;
                *left_hip_compensation_ = 0.0;
                *right_hip_compensation_ = 0.0;
                break;
            default:
                jump_force_controller = false;
                *left_hip_compensation_ = 0.0;
                *right_hip_compensation_ = 0.0;
                break;
            }
            break;
        case Mode::kFly:
            jump_force_controller = true;
            left_force = 0.0;
            right_force = 0.0;
            break;
        case Mode::kSelfHeal:
            jump_force_controller = true;
            break;
        default:
            *left_hip_compensation_ = 0.0;
            *right_hip_compensation_ = 0.0;
            break;
        }

        // 滚转镇定（NORMAL/SPIN/跳跃压腿/落地 + 双腿着地）。
        update_roll_stabilization(mode, dt);

        // F_want = F_ff − 腿长PID（截到 [0.18, 0.40] 的目标 + 滚转偏移）。
        if (!jump_force_controller) {
            left_force = gravity_feedforward
                - left_length_pid_.update(
                    std::clamp(*left_control_length_ + roll_length_offset_, 0.18, 0.40)
                    - *left_length_);
            right_force = gravity_feedforward
                - right_length_pid_.update(
                    std::clamp(*right_control_length_ - roll_length_offset_, 0.18, 0.40)
                    - *right_length_);
        } else {
            left_force = -left_length_pid_.update(
                *left_control_length_ + roll_length_offset_ - *left_length_);
            right_force = -right_length_pid_.update(
                *right_control_length_ - roll_length_offset_ - *right_length_);
        }
        if (roll_mpc_active_) {
            left_force += roll_differential_force_;
            right_force -= roll_differential_force_;
        }

        // 柔顺权重：仅 FLY 模式按离地状态降权；跳跃腾空段不乘。
        const bool jump_flying = mode == Mode::kJump && jump_phase == JumpPhase::kFlying;
        const double left_weight = compliance_weight(mode, *left_grounded_);
        const double right_weight = compliance_weight(mode, *right_grounded_);
        compliance_weight_left_ += 0.1 * (left_weight - compliance_weight_left_);
        compliance_weight_right_ += 0.1 * (right_weight - compliance_weight_right_);

        if (!jump_flying) {
            left_force *= compliance_weight_left_;
            right_force *= compliance_weight_right_;
        }
        double left_spring = *left_spring_force_ * compliance_weight_left_;
        double right_spring = *right_spring_force_ * compliance_weight_right_;

        *left_control_force_ = left_force + left_spring;
        *right_control_force_ = right_force + right_spring;
    }

private:
    static constexpr double kGravity = 9.8;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    [[nodiscard]] static double com_x(double length) {
        // Helios leg_state.CoM_x = −(−0.2208·L0² + 0.0553·L0 + 0.0301)。
        return -(-0.2208 * length * length + 0.0553 * length + 0.0301);
    }

    [[nodiscard]] static double compliance_weight(Mode mode, std::uint8_t grounded) {
        if (mode != Mode::kFly)
            return 1.0;
        if (grounded == 1) // 腾空
            return 0.3;
        if (grounded == 2) // 冲击
            return 0.5;
        return 1.0;
    }

    void update_roll_stabilization(Mode mode, double dt) {
        const bool roll_active = (mode == Mode::kNormal || mode == Mode::kSpin
                                  || (mode == Mode::kJump
                                      && (static_cast<JumpPhase>(*jump_phase_)
                                              == JumpPhase::kPress
                                          || static_cast<JumpPhase>(*jump_phase_)
                                              == JumpPhase::kLanding)))
            && *left_grounded_ == 0 && *right_grounded_ == 0;
        if (!roll_active) {
            roll_mpc_active_ = false;
            roll_integral_ = 0.0;
            roll_length_offset_ = 0.0;
            roll_mpc_.reset();
            roll_differential_force_ = 0.0;
            return;
        }

        const double roll_error = *body_roll_;
        const double roll_error_rate = *body_roll_rate_;
        roll_mpc_active_ = roll_mpc_.solve(
            roll_error, roll_error_rate, roll_force_arm_, roll_inertia_, roll_force_limit_,
            roll_differential_force_);

        double speed_factor = 0.0;
        if (mode == Mode::kSpin)
            speed_factor = std::clamp((std::fabs(*body_yaw_rate_) - 3.0) / 5.0, 0.0, 1.0);

        double integral_gain = 0.0005 - speed_factor * (0.0005 - 0.0003);
        integral_gain += std::clamp((std::fabs(roll_error) - 0.05) * 0.01, 0.0, 0.002);
        roll_integral_ = std::clamp(roll_integral_ + integral_gain * (0.0 - roll_error) * 1000.0 * dt,
            -0.15, 0.15);

        const double kd_base = 0.02 + 0.002 * std::exp(-std::fabs(roll_error) * 20.0);
        const double kd_actual = kd_base - speed_factor * (kd_base - 0.00003);
        const double derivative_term =
            std::fabs(roll_error_rate) > 0.01 ? kd_actual * (0.0 - roll_error_rate) : 0.0;
        roll_length_offset_ = std::clamp(roll_integral_ + derivative_term, -0.15, 0.15);
    }

    void read_parameters() {
        const double kp = get_parameter("length_kp").as_double();
        const double ki = get_parameter("length_ki").as_double();
        const double kd = get_parameter("length_kd").as_double();
        const double output_max = get_parameter("length_output_max").as_double();
        left_length_pid_ = PidCalculator{kp, ki, kd};
        right_length_pid_ = PidCalculator{kp, ki, kd};
        left_length_pid_.output_min = -output_max;
        left_length_pid_.output_max = output_max;
        right_length_pid_.output_min = -output_max;
        right_length_pid_.output_max = output_max;

        body_mass_ = get_parameter("body_mass").as_double();
        leg_mass_ = get_parameter("leg_mass").as_double();
        wheel_mass_ = get_parameter("wheel_mass").as_double();
        takeoff_acceleration_ = get_parameter("takeoff_acceleration").as_double();
        takeoff_kp_ = get_parameter("takeoff_kp").as_double();
        takeoff_kd_ = get_parameter("takeoff_kd").as_double();
        roll_inertia_ = get_parameter("roll_inertia").as_double();
        roll_force_arm_ = get_parameter("roll_force_arm").as_double();
        roll_force_limit_ = get_parameter("roll_force_limit").as_double();
    }

    // ── 输入 ─────────────────────────────────────────────────────────────
    InputInterface<std::uint8_t> mode_;
    InputInterface<std::uint8_t> jump_phase_;
    InputInterface<double> left_control_length_;
    InputInterface<double> right_control_length_;
    InputInterface<double> left_length_;
    InputInterface<double> right_length_;
    InputInterface<double> left_spring_force_;
    InputInterface<double> right_spring_force_;
    InputInterface<std::uint8_t> left_grounded_;
    InputInterface<std::uint8_t> right_grounded_;
    InputInterface<double> body_roll_;
    InputInterface<double> body_roll_rate_;
    InputInterface<double> body_yaw_rate_;
    InputInterface<double> acceleration_vertical_;

    // ── 输出 ─────────────────────────────────────────────────────────────
    OutputInterface<double> left_control_force_;
    OutputInterface<double> right_control_force_;
    OutputInterface<double> left_hip_compensation_;
    OutputInterface<double> right_hip_compensation_;

    // ── 参数 ─────────────────────────────────────────────────────────────
    PidCalculator left_length_pid_{2800.0, 0.0, 60.0};
    PidCalculator right_length_pid_{2800.0, 0.0, 60.0};
    BalanceRollMpc roll_mpc_;
    double body_mass_ = 18.0;
    double leg_mass_ = 2.6;
    double wheel_mass_ = 0.25867;
    double takeoff_acceleration_ = 20.0;
    double takeoff_kp_ = 5.6;
    double takeoff_kd_ = 0.4;
    double roll_inertia_ = 0.35;
    double roll_force_arm_ = 0.20;
    double roll_force_limit_ = 80.0;

    // ── 状态 ─────────────────────────────────────────────────────────────
    bool roll_mpc_active_ = false;
    double roll_differential_force_ = 0.0;
    double roll_length_offset_ = 0.0;
    double roll_integral_ = 0.0;
    double compliance_weight_left_ = 1.0;
    double compliance_weight_right_ = 1.0;
    bool takeoff_error_initialized_ = false;
    double takeoff_error_last_ = 0.0;
    double takeoff_error_filter_ = 0.0;
};

} // namespace hcs_core::controller::chassis::balance

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    hcs_core::controller::chassis::balance::LegForceController, hcs_executor::Component)
