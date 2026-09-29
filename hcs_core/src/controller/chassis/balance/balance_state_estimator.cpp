#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <string>

#include <eigen3/Eigen/Dense>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_sync/tick.hpp>
#include <hcs_utility/rt_attributes.hpp>

#include "controller/chassis/balance/balance_types.hpp"
#include "controller/chassis/balance/control_math.hpp"
#include "controller/chassis/balance/leg_kinematics.hpp"
#include "controller/chassis/balance/speed_kf.hpp"

namespace hcs_core::controller::chassis::balance {

// ============================================================================
// 平衡底盘状态估计（对应 Helios 的 Chassis_Data_Update + SpeedEstimation +
// Multi_Turn_Detection + Gnd_Off_Detect）。
//
// 输入是驱动层原始物理量（腿关节角 = 电机输出角减零点，轮速已含方向），
// 输出是运动学/动力学坐标系下的估计：
//   - 关节角 → 五杆运动学角：phi = side_sign·angle + π/2 ± leg_angle_offset/2
//    （side_sign 左 −1 / 右 +1；front 关节取 +offset/2、back 取 −offset/2，
//     与 Helios Chassis_Data_Update 逐项对应）；
//   - 机体姿态：CH040 欧拉角按安装折算（Helios INF_4：thetab = −roll + 偏置、
//     roll = pitch + 偏置、a_x = acc_y·G……），折算规则全部参数化；
//   - 前向速度：单标量 KF（speed_kf.hpp），腿摆动/伸缩补偿照 Helios；
//   - 离地检测：三态（P_AIR=80 / P_RECOVER=60 N；dP 冲击迁移在 Helios 里
//     已被注释），yaml 开关默认关——Helios 整个函数被 return 0 禁用，
//     默认行为一致（恒着地）。
// ============================================================================

class BalanceStateEstimator
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    BalanceStateEstimator()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        register_leg("left", left_);
        register_leg("right", right_);
        register_input("/chassis/left_wheel/velocity", wheel_velocity_left_);
        register_input("/chassis/right_wheel/velocity", wheel_velocity_right_);
        register_input("/chassis/imu/pitch", imu_pitch_);
        register_input("/chassis/imu/roll", imu_roll_);
        register_input("/chassis/imu/yaw", imu_yaw_);
        register_input("/chassis/imu/pitch_rate", imu_pitch_rate_);
        register_input("/chassis/imu/roll_rate", imu_roll_rate_);
        register_input("/chassis/imu/yaw_rate", imu_yaw_rate_);
        register_input("/chassis/imu/acceleration", imu_acceleration_);
        register_output("/chassis/balance/pitch", body_pitch_, nan_);
        register_output("/chassis/balance/pitch_rate", body_pitch_rate_, nan_);
        register_output("/chassis/balance/roll", body_roll_, nan_);
        register_output("/chassis/balance/roll_rate", body_roll_rate_, nan_);
        register_output("/chassis/balance/yaw", body_yaw_, nan_);
        register_output("/chassis/balance/yaw_rate", body_yaw_rate_, nan_);
        register_output("/chassis/balance/velocity", body_velocity_, nan_);
        register_output("/chassis/balance/distance", body_distance_, 0.0);
        register_output("/chassis/balance/acceleration_forward", body_acceleration_forward_, nan_);
        register_output("/chassis/balance/acceleration_lateral", body_acceleration_lateral_, nan_);
        register_output("/chassis/balance/acceleration_vertical", body_acceleration_vertical_, nan_);

        read_parameters();
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        const double dt = tick.dt_seconds();

        update_body_attitude();
        update_leg(left_, side_sign_left_, dt);
        update_leg(right_, side_sign_right_, dt);
        update_forward_acceleration();
        update_velocity(dt);
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kGravity = 9.8;

    struct LegEstimate {
        // 关节输入（驱动原始量，右腿驱动层不反转）
        InputInterface<double> front_angle, front_velocity, front_torque;
        InputInterface<double> back_angle, back_velocity, back_torque;
        // 输出（运动学坐标系）
        OutputInterface<double> length, length_velocity, angle, angle_velocity;
        OutputInterface<double> angle_total, spring_force, support_force;
        OutputInterface<std::uint8_t> grounded;
        OutputInterface<double> hip_front_angle, hip_back_angle, knee_front_angle, knee_back_angle;

        // 内部状态
        double round_count = 0.0;
        double angle_last = 0.0;
        bool angle_total_initialized = false;
        double dphi0_last = 0.0;
        double dphi0_filtered = 0.0;
        // 离地检测状态
        int ground_state = 0;
        double pressure_last = 0.0;
        bool pressure_initialized = false;
    };

    void register_leg(const std::string& name, LegEstimate& leg) {
        const std::string leg_prefix = "/chassis/" + name + "_leg";
        register_output(leg_prefix + "/length", leg.length, nan_);
        register_output(leg_prefix + "/length_velocity", leg.length_velocity, nan_);
        register_output(leg_prefix + "/angle", leg.angle, nan_);
        register_output(leg_prefix + "/angle_velocity", leg.angle_velocity, nan_);
        register_output(leg_prefix + "/angle_total", leg.angle_total, nan_);
        register_output(leg_prefix + "/spring_force", leg.spring_force, nan_);
        register_output(leg_prefix + "/support_force", leg.support_force, nan_);
        register_output(
            leg_prefix + "/grounded", leg.grounded,
            static_cast<std::uint8_t>(GroundState::kGrounded));
        register_output(leg_prefix + "/hip_front_angle", leg.hip_front_angle, nan_);
        register_output(leg_prefix + "/hip_back_angle", leg.hip_back_angle, nan_);
        register_output(leg_prefix + "/knee_front_angle", leg.knee_front_angle, nan_);
        register_output(leg_prefix + "/knee_back_angle", leg.knee_back_angle, nan_);

        const std::string front = "/chassis/" + name + "_front_joint";
        const std::string back = "/chassis/" + name + "_back_joint";
        register_input(front + "/angle", leg.front_angle);
        register_input(front + "/velocity", leg.front_velocity);
        register_input(front + "/torque", leg.front_torque);
        register_input(back + "/angle", leg.back_angle);
        register_input(back + "/velocity", leg.back_velocity);
        register_input(back + "/torque", leg.back_torque);
    }

    void read_parameters() {
        geometry_.l1 = get_parameter("leg_link1").as_double();
        geometry_.l2 = get_parameter("leg_link2").as_double();
        geometry_.l5 = get_parameter("leg_link5").as_double();
        leg_angle_offset_ = get_parameter("leg_angle_offset").as_double();
        wheel_radius_ = get_parameter("wheel_radius").as_double();
        wheel_half_track_ = get_parameter("wheel_half_track").as_double();
        side_sign_left_ = get_parameter("left_angle_sign").as_double();
        side_sign_right_ = get_parameter("right_angle_sign").as_double();
        velocity_sign_ = get_parameter("velocity_sign").as_double();

        euler_swap_ = get_parameter("imu_euler_swap").as_bool();
        pitch_sign_ = get_parameter("imu_pitch_sign").as_double();
        roll_sign_ = get_parameter("imu_roll_sign").as_double();
        pitch_offset_ = get_parameter("imu_pitch_offset").as_double();
        roll_offset_ = get_parameter("imu_roll_offset").as_double();
        acceleration_swap_ = get_parameter("imu_acceleration_swap").as_bool();
        acceleration_x_sign_ = get_parameter("imu_acceleration_x_sign").as_double();
        acceleration_y_sign_ = get_parameter("imu_acceleration_y_sign").as_double();
        acceleration_z_sign_ = get_parameter("imu_acceleration_z_sign").as_double();

        velocity_filter_.parameters.q = get_parameter("velocity_kf_q").as_double();
        velocity_filter_.parameters.r_base = get_parameter("velocity_kf_r_base").as_double();
        velocity_filter_.parameters.r_max = get_parameter("velocity_kf_r_max").as_double();

        ground_detection_enabled_ = get_parameter("ground_detection_enabled").as_bool();
        ground_pressure_air_ = get_parameter("ground_pressure_air_threshold").as_double();
        ground_pressure_recover_ =
            get_parameter("ground_pressure_recover_threshold").as_double();
        ground_length_threshold_ = get_parameter("ground_length_threshold").as_double();
        ddphi0_filter_alpha_ = get_parameter("ddphi0_filter_alpha").as_double();
    }

    void update_body_attitude() {
        // 欧拉角折算：swap = 安装旋转 90°（Helios INF_4 用传感器的 roll 当机体
        // pitch 用），符号与零偏是安装基准，全部来自参数。
        // 欧拉角折算。ROS 标准（REP-103）正装：机体 pitch = 传感器 pitch，
        // 恒等映射；Helios 的 CH100 是驱动里软件重映射的轴（pitch 从传感器
        // roll 来、还带镜像），那套 swap/符号只在复刻它的安装时才需要。
        const double a = euler_swap_ ? *imu_roll_ : *imu_pitch_;
        const double b = euler_swap_ ? *imu_pitch_ : *imu_roll_;
        *body_pitch_ = pitch_sign_ * a + pitch_offset_;
        *body_roll_ = roll_sign_ * b + roll_offset_;
        *body_yaw_ = wrap_angle(*imu_yaw_);
        *body_pitch_rate_ = pitch_sign_ * (euler_swap_ ? *imu_roll_rate_ : *imu_pitch_rate_);
        *body_roll_rate_ = roll_sign_ * (euler_swap_ ? *imu_pitch_rate_ : *imu_roll_rate_);
        *body_yaw_rate_ = *imu_yaw_rate_;
    }

    void update_leg(LegEstimate& leg, double side_sign, double dt) {
        // 关节角 → 五杆运动学角（与 Helios Chassis_Data_Update 逐项一致）。
        const double phi_front =
            side_sign * *leg.front_angle + std::numbers::pi / 2.0 + leg_angle_offset_ / 2.0;
        const double phi_back =
            side_sign * *leg.back_angle + std::numbers::pi / 2.0 - leg_angle_offset_ / 2.0;
        const double dphi_front = side_sign * *leg.front_velocity;
        const double dphi_back = side_sign * *leg.back_velocity;

        const LegPose pose = leg_pos(
            phi_front, phi_back, geometry_.l1, geometry_.l2, geometry_.l5, dphi_front,
            dphi_back);

        if (!leg.angle_total_initialized) {
            leg.angle_total_initialized = true;
            leg.angle_last = pose.angle;
        }
        const double angle_total = multi_turn_angle(
            leg.round_count, pose.angle, leg.angle_last);

        *leg.length = pose.length;
        *leg.length_velocity = pose.length_velocity;
        *leg.angle = pose.angle;
        *leg.angle_velocity = pose.angle_velocity;
        *leg.angle_total = angle_total;
        *leg.hip_front_angle = phi_front;
        *leg.hip_back_angle = phi_back;
        *leg.knee_front_angle = pose.knee_front;
        *leg.knee_back_angle = pose.knee_back;

        // 弹簧力多项式（spring_force_estimation）。
        *leg.spring_force =
            -1843.3 * pose.length * pose.length + 1112.2 * pose.length - 49.6586 + 24.0;

        // 关节实测力矩 → 接触力/髋力矩反解（inverse_contact_force，左腿 side=-1）。
        const double leg_inertia = 0.1158 * pose.length + 0.0143;
        const double dphi0_raw = (pose.angle_velocity - leg.dphi0_last) / dt;
        leg.dphi0_last = pose.angle_velocity;
        leg.dphi0_filtered += ddphi0_filter_alpha_ * (dphi0_raw - leg.dphi0_filtered);

        double contact_force = 0.0;
        double contact_hip_torque = 0.0;
        inverse_contact_force(
            pose.angle, phi_front, pose.knee_front, pose.knee_back, phi_back, geometry_.l1,
            pose.length, *leg.front_torque, *leg.back_torque, contact_force,
            contact_hip_torque, side_sign > 0 ? 1 : 0);

        const double hip_torque_compensated =
            contact_hip_torque - leg_inertia * leg.dphi0_filtered;
        const double angle_to_vertical = pose.angle - std::numbers::pi / 2.0 - *body_pitch_;
        double pressure = (contact_force + *leg.spring_force) * std::cos(angle_to_vertical)
            + hip_torque_compensated * std::sin(angle_to_vertical);
        *leg.support_force = std::max(pressure, 0.0);

        *leg.grounded = detect_ground(leg, pose.length, *leg.support_force, dt);
    }

    /// 离地检测（Gnd_Off_Detect 函数体；dP 冲击迁移在 Helios 里已被注释禁用，
    /// 这里只保留 P 滞回两态。整个功能默认禁用 = 恒着地，行为同 Helios）。
    std::uint8_t detect_ground(LegEstimate& leg, double length, double pressure, double dt) {
        (void)dt;
        if (!ground_detection_enabled_)
            return static_cast<std::uint8_t>(GroundState::kGrounded);

        if (!leg.pressure_initialized) {
            leg.pressure_last = pressure;
            leg.pressure_initialized = true;
        }
        (void)leg.pressure_last;
        leg.pressure_last = pressure;

        switch (leg.ground_state) {
        case 0:
            if (pressure < ground_pressure_air_ && length > ground_length_threshold_)
                leg.ground_state = 1;
            break;
        case 1:
            if (pressure > ground_pressure_recover_)
                leg.ground_state = 0;
            break;
        default: leg.ground_state = 0; break;
        }
        return static_cast<std::uint8_t>(leg.ground_state);
    }

    static double multi_turn_angle(double& round_count, double angle, double& angle_last) {
        if (angle < angle_last && std::fabs(angle - angle_last) > std::numbers::pi)
            round_count += 1.0;
        else if (angle > angle_last && std::fabs(angle - angle_last) > std::numbers::pi)
            round_count -= 1.0;
        angle_last = angle;
        return round_count * 2.0 * std::numbers::pi + angle;
    }

    /// 前向/侧向/垂向加速度（Helios 的 wbc_state.a_y/a_x/a_z：重力分量 +
    /// 离心项补偿）。输入先按安装参数折算成机体 FLU 轴（ROS 标准正装 =
    /// 恒等映射），机体 x 即前向。
    void update_forward_acceleration() {
        const double bx = acceleration_x_sign_
            * (acceleration_swap_ ? imu_acceleration_->y() : imu_acceleration_->x()) * kGravity;
        const double by = acceleration_y_sign_
            * (acceleration_swap_ ? imu_acceleration_->x() : imu_acceleration_->y()) * kGravity;
        const double bz = acceleration_z_sign_ * imu_acceleration_->z() * kGravity;

        const double r_imu = (*left_.length * std::cos(*left_.angle)
                                 + *right_.length * std::cos(*right_.angle))
            / 2.0;
        forward_acceleration_ = (bx + std::sin(*body_pitch_) * kGravity) * std::cos(*body_pitch_)
            - *body_yaw_rate_ * *body_yaw_rate_ * r_imu;
        lateral_acceleration_ = by;
        vertical_acceleration_ = (bz - std::cos(*body_pitch_) * kGravity)
            * std::cos(*body_pitch_);
        *body_acceleration_forward_ = forward_acceleration_;
        *body_acceleration_lateral_ = lateral_acceleration_;
        *body_acceleration_vertical_ = vertical_acceleration_;
    }

    void update_velocity(double dt) {
        // 腿摆动/伸缩补偿（SpeedEstimation）：thetall = −phi0 + π/2 + thetab，
        // thetall1 = −dphi0 + thetab1；vl = 轮速 − θ̇·L0·cosθ − Ḷ0·sinθ。
        const double theta_left = std::numbers::pi / 2.0 - *left_.angle + *body_pitch_;
        const double theta_right = std::numbers::pi / 2.0 - *right_.angle + *body_pitch_;
        const double swing_left = (-*left_.angle_velocity + *body_pitch_rate_)
                * *left_.length * std::cos(theta_left)
            + *left_.length_velocity * std::sin(theta_left);
        const double swing_right = (-*right_.angle_velocity + *body_pitch_rate_)
                * *right_.length * std::cos(theta_right)
            + *right_.length_velocity * std::sin(theta_right);
        const double measured_left = *wheel_velocity_left_ - swing_left;
        const double measured_right = *wheel_velocity_right_ - swing_right;

        const double leg_pressures[2] = {*left_.support_force, *right_.support_force};
        const bool legs_off_ground[2] = {
            *left_.support_force < 20.0, *right_.support_force < 20.0};
        const double leg_angle_velocities[2] = {*left_.angle_velocity, *right_.angle_velocity};

        const double velocity = velocity_filter_.update(
            measured_left, measured_right, *body_yaw_rate_, forward_acceleration_,
            leg_pressures, legs_off_ground, leg_angle_velocities, dt);
        *body_velocity_ = velocity_sign_ * velocity;
        distance_ += *body_velocity_ * dt;
        *body_distance_ = distance_;
        (void)lateral_acceleration_;
        (void)vertical_acceleration_;
        (void)wheel_radius_;
        (void)wheel_half_track_;
    }

    // ── 输入 ─────────────────────────────────────────────────────────────
    LegEstimate left_;
    LegEstimate right_;
    InputInterface<double> wheel_velocity_left_;
    InputInterface<double> wheel_velocity_right_;
    InputInterface<double> imu_pitch_;
    InputInterface<double> imu_roll_;
    InputInterface<double> imu_yaw_;
    InputInterface<double> imu_pitch_rate_;
    InputInterface<double> imu_roll_rate_;
    InputInterface<double> imu_yaw_rate_;
    InputInterface<Eigen::Vector3d> imu_acceleration_;

    // ── 输出 ─────────────────────────────────────────────────────────────
    OutputInterface<double> body_pitch_;
    OutputInterface<double> body_pitch_rate_;
    OutputInterface<double> body_roll_;
    OutputInterface<double> body_roll_rate_;
    OutputInterface<double> body_yaw_;
    OutputInterface<double> body_yaw_rate_;
    OutputInterface<double> body_velocity_;
    OutputInterface<double> body_distance_;
    OutputInterface<double> body_acceleration_forward_;
    OutputInterface<double> body_acceleration_lateral_;
    OutputInterface<double> body_acceleration_vertical_;

    // ── 参数 ─────────────────────────────────────────────────────────────
    struct Geometry {
        double l1 = 0.21;
        double l2 = 0.25;
        double l5 = 0.0;
    };
    Geometry geometry_;
    double leg_angle_offset_ = 1.163;
    double wheel_radius_ = 0.06;
    double wheel_half_track_ = 0.2296;
    double side_sign_left_ = -1.0;
    double side_sign_right_ = 1.0;
    double velocity_sign_ = -1.0; // Helios 的 s1 = −SpeedEstimation()

    bool euler_swap_ = false;          ///< ROS 标准正装 = false；复刻 Helios CH100 安装才开
    double pitch_sign_ = 1.0;
    double roll_sign_ = 1.0;
    double pitch_offset_ = 0.0;        ///< 水平校准偏置，上车实测后填
    double roll_offset_ = 0.0;
    bool acceleration_swap_ = false;   ///< ROS 标准正装 = false
    double acceleration_x_sign_ = 1.0;
    double acceleration_y_sign_ = 1.0;
    double acceleration_z_sign_ = 1.0;

    SpeedKalmanFilter velocity_filter_;
    double forward_acceleration_ = 0.0;
    double lateral_acceleration_ = 0.0;
    double vertical_acceleration_ = 0.0;

    bool ground_detection_enabled_ = false;
    double ground_pressure_air_ = 80.0;
    double ground_pressure_recover_ = 60.0;
    double ground_length_threshold_ = 0.20;
    double ddphi0_filter_alpha_ = 0.1;

    double distance_ = 0.0;
};

} // namespace hcs_core::controller::chassis::balance

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    hcs_core::controller::chassis::balance::BalanceStateEstimator, hcs_executor::Component)
