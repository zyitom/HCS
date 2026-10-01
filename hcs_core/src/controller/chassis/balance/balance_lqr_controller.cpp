#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>

#include "controller/pid/pid_calculator.hpp"

#include "controller/chassis/balance/balance_types.hpp"
#include "controller/chassis/balance/control_math.hpp"
#include "controller/chassis/balance/lqr_gain_table.hpp"
#include "controller/chassis/balance/nlmpc_solver.hpp"

namespace hcs_core::controller::chassis::balance {
using hcs_core::controller::pid::PidCalculator;

// ============================================================================
// 平衡 LQR/NLMPC 控制器（对应 Helios Chassis_Controller_Update 的活分支）。
//
// 输入估计量与模式，输出四个通道：左右轮力矩、左右髋力矩。NaN 表示不控制。
//
//   - 状态向量装配照 Helios（驻车/启动助力、yaw 误差 tanh + 卡死惩罚、
//     着陆恢复因子、跳跃俯仰目标窗）；pitch_preset 与 adapt 项在 Helios 里
//     被强制清零（"Temporarily disable ... while validating NLMPC"），按现状
//     搬，参数保留。
//   - 主控制器 NLMPC（nlmpc_solver.hpp），失败回退 LQR 拟合表
//    （lqr_torques，improve0 = 0.1）。NLMPC_CYCLE_LIMIT 是 MCU 的 DWT 周期
//     预算，主机侧没有对应物，不搬。
//   - PID_ONLY 模式（起身/自救/上台阶/跳跃腾空）切摆动腿角度 PID。
// ============================================================================

class BalanceLqrController
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    BalanceLqrController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        register_input("/chassis/control_velocity", control_velocity_);
        register_input("/chassis/balance/mode", mode_);
        register_input("/chassis/balance/process", process_);
        register_input("/chassis/balance/jump_phase", jump_phase_);
        register_input("/chassis/balance/inverted", inverted_);
        register_input("/chassis/balance/pitch", body_pitch_);
        register_input("/chassis/balance/pitch_rate", body_pitch_rate_);
        register_input("/chassis/balance/yaw", body_yaw_);
        register_input("/chassis/balance/yaw_rate", body_yaw_rate_);
        register_input("/chassis/balance/velocity", body_velocity_);
        register_input("/chassis/balance/distance", body_distance_);
        register_input("/chassis/left_leg/length", left_length_);
        register_input("/chassis/right_leg/length", right_length_);
        register_input("/chassis/left_leg/angle", left_angle_);
        register_input("/chassis/right_leg/angle", right_angle_);
        register_input("/chassis/left_leg/angle_total", left_angle_total_);
        register_input("/chassis/right_leg/angle_total", right_angle_total_);
        register_input("/chassis/left_leg/angle_velocity", left_angle_velocity_);
        register_input("/chassis/right_leg/angle_velocity", right_angle_velocity_);
        register_input("/chassis/left_leg/grounded", left_grounded_);
        register_input("/chassis/right_leg/grounded", right_grounded_);
        register_input("/chassis/left_leg/control_angle", left_control_angle_);
        register_input("/chassis/right_leg/control_angle", right_control_angle_);
        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);

        register_output("/chassis/left_wheel/control_torque", wheel_torque_left_, nan_);
        register_output("/chassis/right_wheel/control_torque", wheel_torque_right_, nan_);
        register_output("/chassis/left_leg/control_hip_torque", hip_torque_left_, nan_);
        register_output("/chassis/right_leg/control_hip_torque", hip_torque_right_, nan_);

        read_parameters();
    }

    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        const auto process = static_cast<Process>(*process_);
        const auto mode = static_cast<Mode>(*mode_);
        const auto jump_phase = static_cast<JumpPhase>(*jump_phase_);

        if (process != process_previous_) {
            nlmpc_.reset();
            roll_mpc_.reset();
            relative_distance_offset_ = *body_distance_;
        }
        process_previous_ = process;

        lqr_k(*left_length_, *right_length_, kKOut, k_matrix_);

        const double theta_left = leg_angle_to_vertical(*left_angle_, *body_pitch_);
        const double theta_right = leg_angle_to_vertical(*right_angle_, *body_pitch_);
        relative_distance_ = *body_distance_ - relative_distance_offset_;

        switch (process) {
        case Process::kLqrOn: update_lqr_on(mode, jump_phase, theta_left, theta_right); break;
        case Process::kPidOnly: update_pid_only(mode, jump_phase); break;
        case Process::kHealing: update_healing(); break;
        case Process::kDisabled: set_outputs(0.0, 0.0, 0.0, 0.0); break;
        }
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kPi = std::numbers::pi;

    [[nodiscard]] static double leg_angle_to_vertical(double leg_angle, double body_pitch) {
        double theta = -leg_angle + kPi / 2.0 + body_pitch;
        if (theta > kPi)
            theta -= 2.0 * kPi;
        else if (theta < -kPi)
            theta += 2.0 * kPi;
        return theta;
    }

    void read_parameters() {
        max_wheel_torque_ = get_parameter("max_wheel_torque").as_double();
        max_leg_torque_ = get_parameter("max_leg_torque").as_double();
        lqr_fallback_improve_ = get_parameter("lqr_fallback_improve").as_double();
        gimbal_yaw_angle_sign_ = get_parameter("gimbal_yaw_angle_sign").as_double();
        swing_pid_.kp = get_parameter("swing_kp").as_double();
        swing_pid_.ki = get_parameter("swing_ki").as_double();
        swing_pid_.kd = get_parameter("swing_kd").as_double();
        swing_pid_.output_max = get_parameter("swing_output_max").as_double();
        swing_pid_.output_min = -get_parameter("swing_output_max").as_double();
    }

    void update_lqr_on(Mode mode, JumpPhase jump_phase, double theta_left, double theta_right) {
        update_state_vector(mode, jump_phase, theta_left, theta_right);

        double output[kControlSize] = {0.0, 0.0, 0.0, 0.0};
        const bool nlmpc_ok = nlmpc_.solve(
            *left_length_, *right_length_, status_vector_, theta_left, theta_right, output);

        if (!nlmpc_ok) {
            lqr_torques(
                k_matrix_, status_vector_, output[0], output[1], output[2], output[3],
                *left_grounded_ != 0, *right_grounded_ != 0, lqr_fallback_improve_);
        }

        double wheel_left = std::clamp(output[0], -max_wheel_torque_, max_wheel_torque_);
        double wheel_right = std::clamp(output[1], -max_wheel_torque_, max_wheel_torque_);
        // 离地腿轮矩清零（检测禁用时 grounded 恒 0，此条件恒假，同 Helios 现状）。
        if (*left_grounded_ != 0)
            wheel_left = 0.0;
        if (*right_grounded_ != 0)
            wheel_right = 0.0;
        const double hip_left = std::clamp(output[2], -max_leg_torque_, max_leg_torque_);
        const double hip_right = std::clamp(output[3], -max_leg_torque_, max_leg_torque_);
        set_outputs(wheel_left, wheel_right, hip_left, hip_right);
    }

    /// 状态向量装配（Helios 518-860 的活分支；驻车/启动助力、yaw 处理）。
    void update_state_vector(Mode mode, JumpPhase jump_phase, double theta_left, double theta_right) {
        // [0] 位置参考：驻车锁存 + 启动助力的位移偏置，60 Hz 一阶低通。
        const double command_velocity = (*control_velocity_)[0];
        double raw_position = 0.0;
        if (mode == Mode::kNormal) {
            if (std::fabs(command_velocity) >= park_command_threshold_)
                park_state_ = ParkState::kOff;
            else
                switch (park_state_) {
                case ParkState::kOff:
                    if (std::fabs(command_velocity) < park_command_threshold_)
                        park_state_ = ParkState::kWaiting;
                    break;
                case ParkState::kWaiting:
                    if (std::fabs(*body_velocity_) < park_speed_threshold_) {
                        park_state_ = ParkState::kEngaged;
                        park_locked_distance_ = relative_distance_;
                    }
                    break;
                case ParkState::kEngaged: {
                    double position_error = park_locked_distance_ - relative_distance_;
                    if (std::fabs(position_error) < park_error_deadband_)
                        position_error = 0.0;
                    position_error = std::clamp(
                        position_error, -park_max_position_error_, park_max_position_error_);
                    raw_position = park_position_gain_ * position_error;
                    break;
                }
                }
        } else {
            park_state_ = ParkState::kOff;
        }
        position_filter_ += 0.314 * (raw_position - position_filter_);
        status_vector_[0] = position_filter_;

        // [1] 速度误差。
        double velocity_error = std::clamp(command_velocity - *body_velocity_, -2.5, 2.5);
        if (std::fabs(velocity_error) < 0.005)
            velocity_error = 0.0;
        status_vector_[1] = velocity_error;

        // [2] yaw 误差：跟随角 = 机体 yaw + 云台 yaw（方向参数）+ 掉头偏置。
        if (mode == Mode::kNormal || mode == Mode::kSpin) {
            double follow_angle = *body_yaw_;
            if (gimbal_yaw_angle_.has_provider())
                follow_angle += gimbal_yaw_angle_sign_ * *gimbal_yaw_angle_;
            if (*inverted_)
                follow_angle += kPi;
            follow_angle = wrap_angle(follow_angle);

            double yaw_error = wrap_angle(follow_angle - *body_yaw_);
            const double absolute_yaw_error = std::fabs(yaw_error);
            const bool stuck = std::fabs(*body_velocity_) < 0.2
                && (std::fabs(last_wheel_torque_left_) + std::fabs(last_wheel_torque_right_))
                    > max_wheel_torque_ * 1.2;
            if (absolute_yaw_error > 0.8 && stuck)
                yaw_error *= std::exp(-1.8 * (absolute_yaw_error - 0.8));
            smoothed_yaw_error_ += std::clamp(yaw_error - smoothed_yaw_error_, -0.5, 0.5);
            status_vector_[2] = 0.85 * std::tanh(smoothed_yaw_error_ / 0.6);
        } else {
            smoothed_yaw_error_ = 0.0;
            status_vector_[2] = 0.0;
        }

        // [3] yaw 速率指令：小陀螺直接给、跟随锁相、否则按速度误差平滑放开。
        double yaw_rate_command_raw = 0.0;
        if (mode == Mode::kSpin) {
            yaw_rate_command_raw = (*control_velocity_)[2] - *body_yaw_rate_;
        } else if (mode == Mode::kNormal) {
            yaw_rate_command_raw = -*body_yaw_rate_;
        } else {
            yaw_rate_command_raw = ((*control_velocity_)[2] - *body_yaw_rate_)
                * smooth_saturation(
                    std::fabs((*control_velocity_)[0] - *body_velocity_), 0.05, 4.0);
        }

        // 着陆恢复：空中/落地时 yaw 权限收紧到 15%，落地后每拍 +0.005 恢复。
        const bool airborne_or_landing = mode == Mode::kFly || *left_grounded_ != 0
            || *right_grounded_ != 0;
        landing_recovery_factor_ = airborne_or_landing
            ? 0.15
            : std::min(1.0, landing_recovery_factor_ + 0.005);
        const double yaw_limit = 0.5 + (5.0 - 0.5) * landing_recovery_factor_;
        const double yaw_slew = 0.08 + (0.4 - 0.08) * landing_recovery_factor_;
        const double yaw_rate_target = std::clamp(yaw_rate_command_raw, -yaw_limit, yaw_limit);
        yaw_rate_command_limited_ +=
            std::clamp(yaw_rate_target - yaw_rate_command_limited_, -yaw_slew, yaw_slew);
        if (std::fabs(yaw_rate_command_limited_) < 0.001)
            yaw_rate_command_limited_ = 0.0;
        status_vector_[3] =
            mode == Mode::kSpin || mode == Mode::kFly ? yaw_rate_command_raw
                                                      : yaw_rate_command_limited_;
        if (mode == Mode::kFly)
            status_vector_[3] = 0.0;

        // [4..7] 腿角/角速率（pitch_preset 与 adapt 项按 Helios 现状清零）。
        status_vector_[4] = -theta_left;
        status_vector_[5] = 0.0 - (-*left_angle_velocity_ + *body_pitch_rate_);
        status_vector_[6] = -theta_right;
        status_vector_[7] = 0.0 - (-*right_angle_velocity_ + *body_pitch_rate_);

        // [8][9] 机体俯仰。
        const bool in_jump_window = mode == Mode::kJump
            && (jump_phase == JumpPhase::kPress || jump_phase == JumpPhase::kTakeOff);
        const double target_pitch = in_jump_window ? 0.1 : 0.0;
        status_vector_[8] = std::clamp(target_pitch - *body_pitch_, -0.52, 0.52);
        status_vector_[9] = -*body_pitch_rate_;

        last_wheel_torque_left_ = *wheel_torque_left_;
        last_wheel_torque_right_ = *wheel_torque_right_;
    }

    static double smooth_saturation(double x, double width, double smoothness) {
        if (std::fabs(x) < width)
            return 1.0;
        return std::exp(-smoothness * std::pow(std::fabs(x) - width, 2));
    }

    /// PID_ONLY：起身/自救/上台阶用摆动角 PID；跳跃腾空/飞坡清轮矩并带俯仰 PD。
    void update_pid_only(Mode mode, JumpPhase jump_phase) {
        // 摆动 PID：误差 = 模式机斜坡后的目标（多圈绝对角）− 当前多圈腿角。
        if (mode == Mode::kSlowStart || mode == Mode::kSelfHeal || mode == Mode::kUpStair) {
            const double hip_left = -swing_pid_.update(swing_error_left());
            const double hip_right = -swing_pid_.update(swing_error_right());
            set_outputs(0.0, 0.0, hip_left, hip_right);
            return;
        }
        if (mode == Mode::kJump && jump_phase >= JumpPhase::kTakeOff) {
            // 腾空段加俯仰 PD 前馈（Helios 的 FLYING 分支）。
            const double pitch_pd = std::clamp(
                0.35 * *body_pitch_ + 0.10 * *body_pitch_rate_, -0.30, 0.30);
            const double hip_left = -swing_pid_.update(swing_error_left() + pitch_pd);
            const double hip_right = -swing_pid_.update(swing_error_right() + pitch_pd);
            set_outputs(0.0, 0.0, hip_left, hip_right);
            return;
        }
        set_outputs(0.0, 0.0, 0.0, 0.0);
    }

    void update_healing() {
        const double hip_left = -swing_pid_.update(swing_error_left());
        const double hip_right = -swing_pid_.update(swing_error_right());
        set_outputs(0.0, 0.0, hip_left, hip_right);
    }

    [[nodiscard]] double swing_error_left() const {
        return *left_control_angle_ - *left_angle_total_;
    }
    [[nodiscard]] double swing_error_right() const {
        return *right_control_angle_ - *right_angle_total_;
    }

    void set_outputs(double wheel_left, double wheel_right, double hip_left, double hip_right) {
        *wheel_torque_left_ = wheel_left;
        *wheel_torque_right_ = wheel_right;
        *hip_torque_left_ = hip_left;
        *hip_torque_right_ = hip_right;
    }

    // ── 输入 ─────────────────────────────────────────────────────────────
    InputInterface<Eigen::Vector3d> control_velocity_;
    InputInterface<std::uint8_t> mode_;
    InputInterface<std::uint8_t> process_;
    InputInterface<std::uint8_t> jump_phase_;
    InputInterface<bool> inverted_;
    InputInterface<double> body_pitch_;
    InputInterface<double> body_pitch_rate_;
    InputInterface<double> body_yaw_;
    InputInterface<double> body_yaw_rate_;
    InputInterface<double> body_velocity_;
    InputInterface<double> body_distance_;
    InputInterface<double> left_length_;
    InputInterface<double> right_length_;
    InputInterface<double> left_angle_;
    InputInterface<double> right_angle_;
    InputInterface<double> left_angle_velocity_;
    InputInterface<double> right_angle_velocity_;
    InputInterface<std::uint8_t> left_grounded_;
    InputInterface<std::uint8_t> right_grounded_;
    InputInterface<double> left_control_angle_;
    InputInterface<double> right_control_angle_;
    InputInterface<double> gimbal_yaw_angle_;

    // 摆动 PID 的测量输入（多圈绝对腿角）。
    InputInterface<double> left_angle_total_;
    InputInterface<double> right_angle_total_;

    // ── 输出 ─────────────────────────────────────────────────────────────
    OutputInterface<double> wheel_torque_left_;
    OutputInterface<double> wheel_torque_right_;
    OutputInterface<double> hip_torque_left_;
    OutputInterface<double> hip_torque_right_;

    // ── 参数 ─────────────────────────────────────────────────────────────
    double max_wheel_torque_ = 5.0;
    double max_leg_torque_ = 40.0;
    double lqr_fallback_improve_ = 0.1;
    double gimbal_yaw_angle_sign_ = -1.0;
    PidCalculator swing_pid_{250.0, 0.0, 7.0};

    // ── 状态 ─────────────────────────────────────────────────────────────
    enum class ParkState { kOff, kWaiting, kEngaged };
    ParkState park_state_ = ParkState::kOff;
    double park_locked_distance_ = 0.0;
    double park_command_threshold_ = 0.05;
    double park_speed_threshold_ = 0.20;
    double park_position_gain_ = 5.0;
    double park_max_position_error_ = 0.30;
    double park_error_deadband_ = 0.005;

    double position_filter_ = 0.0;
    double relative_distance_ = 0.0;
    double relative_distance_offset_ = 0.0;
    double smoothed_yaw_error_ = 0.0;
    double yaw_rate_command_limited_ = 0.0;
    double landing_recovery_factor_ = 1.0;
    double last_wheel_torque_left_ = 0.0;
    double last_wheel_torque_right_ = 0.0;
    Process process_previous_ = Process::kDisabled;

    double status_vector_[10]{};
    double k_matrix_[4][10]{};
    BalanceNlmpc nlmpc_;
    BalanceRollMpc roll_mpc_;
};

} // namespace hcs_core::controller::chassis::balance

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    hcs_core::controller::chassis::balance::BalanceLqrController, hcs_executor::Component)
