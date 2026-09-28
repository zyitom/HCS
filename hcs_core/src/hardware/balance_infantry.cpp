#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <numbers>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <stdexcept>
#include <string>

#include <libhcs/board/hcs_can_port.hpp>
#include <libhcs/board/hpm5321.hpp>
#include <libhcs/data/datas.hpp>
#include <libhcs/protocol/handler.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/timer.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_sync/tick.hpp>
#include <hcs_utility/machine_guard.hpp>
#include <hcs_utility/rt_attributes.hpp>
#include <hcs_utility/thread_config.hpp>

#include <hcs_description/tf_description.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/hipnuc.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/vt13.hpp"
#include "hardware/util/board_transmitter.hpp"

namespace hcs_core::hardware {

// ============================================================================
// 平衡步兵整车硬件组件（模板 = RMCS 的 omni_infantry.cpp）。
//
// 三块 libhcs::board::Hpm5321，设备清单与接线（§4.4；上车前必须由用户核对）：
//
//   gimbal 板
//     CAN1: LK yaw  （Helios LK_motor(5, 5010)，指令 0x145 / 反馈 0x145）
//           LK 拨盘 （Helios LK_motor(4, 5010)，指令 0x144 / 反馈 0x144）
//     CAN2: 3508 摩擦轮 id1/id2（0x201/0x202，4 合 1 发送帧 0x200）
//           DM pitch（Helios DM_MOTOR(0x03, 0x04, ...)：反馈 0x03 / 指令 0x04）
//     UART0: 云台 CH040（HiPNUC HI91，921600）
//   chassis 板
//     CAN1: 左腿 DM x2（反馈 0x05/0x06，指令 0x01/0x02）
//     CAN2: 右腿 DM x2（反馈 0x07/0x08，指令 0x03/0x04）
//     UART0: 底盘 CH040（921600）
//   aux 板
//     CAN1: 3508 轮毂 左 id2 / 右 id1（0x202/0x201，发送帧 0x200）
//     UART0: VT13 遥控接收机
//
// 腿关节命名 = /chassis/{left,right}_{front,back}_joint。Helios 的
// motor_leg[0..3] = bl1、bl0、br1、br0，对应关系与 front/back 约定：
//
//   Helios   HCS                     反馈/指令 id   org_pos (rad)
//   bl0   →  left_front_joint        0x06 / 0x02    0.717242718
//   bl1   →  left_back_joint         0x05 / 0x01    1.938694
//   br0   →  right_front_joint       0x08 / 0x04    1.39220476
//   br1   →  right_back_joint        0x07 / 0x03    -1.37379646
//
// Helios 的五杆腿 l5 = 0，两条髋轴同轴，front/back 纯属命名约定（phi1 = bl0、
// phi4 = bl1，见 balance_algorithm.leg_pos），上车前按实际安装核对。
//
// 方向约定（与 Helios 对齐，待上台架核对）：
//   腿关节驱动层一律不反转：左腿角度取负在估计器（运动学安装方向），
//   右腿力矩取负在 LegJointController（Helios Motor_Send 的 -final_Tl）；
//   yaw / pitch 反转（Helios dir_yaw = dir_pitch = -1，命令与反馈一起反）；
//   右轮反转（Helios 里右轮速度取负、电流取负，一次反转等价）。
//
// 与 RMCS 不同的只有发送与安全（§4.4）：
//   指令侧在 Command 伙伴里打包成定长批次，经 Snapshot 交给三板共用的
//   发送线程（board_transmitter.hpp）；DM 关节 torque 为 NaN 时发 0xFD 失能帧，
//   LK 遇 NaN 自动失能，DJI 遇 NaN 发 0 电流；组件被隔离后发送线程只补发
//   一次全失能批次，然后静默。
// ============================================================================

class BalanceInfantry
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    BalanceInfantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_(
              create_partner_component<BalanceInfantryCommand>(
                  get_component_name() + "_command", *this))
        , yaw_lk_(*this, *command_, "/gimbal/yaw")
        , dial_lk_(*this, *command_, "/gimbal/dial")
        , pitch_dm_(*this, *command_, "/gimbal/pitch")
        , friction_left_wheel_(*this, *command_, "/gimbal/left_friction_wheel")
        , friction_right_wheel_(*this, *command_, "/gimbal/right_friction_wheel")
        , left_front_joint_(*this, *command_, "/chassis/left_front_joint")
        , left_back_joint_(*this, *command_, "/chassis/left_back_joint")
        , right_front_joint_(*this, *command_, "/chassis/right_front_joint")
        , right_back_joint_(*this, *command_, "/chassis/right_back_joint")
        , left_wheel_(*this, *command_, "/chassis/left_wheel")
        , right_wheel_(*this, *command_, "/chassis/right_wheel")
        , gimbal_imu_(*this, "/gimbal/imu")
        , chassis_imu_(*this, "/chassis/imu")
        , remote_control_(*this)
        , gimbal_callback_(*this, kGimbal)
        , chassis_callback_(*this, kChassis)
        , aux_callback_(*this, kAux) {

        remote_control_.register_vt13(&vt13_);
        configure_devices();

        register_output("/tf", tf_);
        register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_, nan_);
        register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_, nan_);
        register_output("/chassis/imu/pitch", chassis_imu_pitch_, nan_);
        register_output("/chassis/imu/roll", chassis_imu_roll_, nan_);
        register_output("/chassis/imu/yaw", chassis_imu_yaw_, nan_);
        register_output("/chassis/imu/pitch_rate", chassis_imu_pitch_rate_, nan_);
        register_output("/chassis/imu/roll_rate", chassis_imu_roll_rate_, nan_);
        register_output("/chassis/imu/yaw_rate", chassis_imu_yaw_rate_, nan_);
        // /chassis/imu/{quaternion,angular_velocity,acceleration,online} 由
        // Hipnuc 设备自己注册，这里不重复；欧拉角与角速度分量是桥接输出。

        // 板卡在构造期就可能开始回调，放在所有它会碰的数据都就位之后（§4.4）。
        create_board("gimbal", gimbal_callback_, gimbal_board_);
        create_board("chassis", chassis_callback_, chassis_board_);
        create_board("aux", aux_callback_, aux_board_);
        check_classic_can();

        transmitter_ = std::make_unique<util::BoardTransmitter>(
            std::array<libhcs::board::Hpm5321*, 3>{
                gimbal_board_.get(), chassis_board_.get(), aux_board_.get()},
            hcs_utility::ThreadConfig{
                string_parameter("tx_thread_config", ""), get_component_name() + "-tx"},
            std::chrono::milliseconds{int_parameter("tx_stale_after_ms", 10)});
        transmitter_->set_safe_batch(build_safe_batch());
        report_bus_load();

        static std::once_flag machine_guard_once;
        std::call_once(machine_guard_once, [this]() {
            for (const auto& finding : hcs_utility::MachineGuard::run()) {
                const char* text = finding.text.c_str();
                switch (finding.level) {
                case hcs_utility::MachineGuard::Level::kCritical:
                    RCLCPP_ERROR(get_logger(), "[machine] %s", text);
                    break;
                case hcs_utility::MachineGuard::Level::kWarn:
                    RCLCPP_WARN(get_logger(), "[machine] %s", text);
                    break;
                default: RCLCPP_INFO(get_logger(), "[machine] %s", text); break;
                }
            }
        });

        report_timer_ = create_wall_timer(std::chrono::seconds{1}, [this] { report(); });
    }

    hcs_utility::Doorbell* tick_end_doorbell() override {
        return transmitter_ ? &transmitter_->doorbell() : nullptr;
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        // 反馈解包：回调线程只存了原始帧，这里解成物理量写进输出。
        yaw_lk_.update_status();
        dial_lk_.update_status();
        pitch_dm_.update_status();
        friction_left_wheel_.update_status();
        friction_right_wheel_.update_status();
        left_front_joint_.update_status();
        left_back_joint_.update_status();
        right_front_joint_.update_status();
        right_back_joint_.update_status();
        left_wheel_.update_status();
        right_wheel_.update_status();
        gimbal_imu_.update_status();
        chassis_imu_.update_status();
        vt13_.update_status(tick.scheduled);

        remote_control_.update();
        update_imu();
    }

private:
    // ── 板卡下标与 CAN id（见文件头注释的接线表）───────────────────────────
    static constexpr std::size_t kGimbal = 0;
    static constexpr std::size_t kChassis = 1;
    static constexpr std::size_t kAux = 2;

    static constexpr std::uint32_t kYawCanId = 0x145;   // LK: id + 0x140
    static constexpr std::uint32_t kDialCanId = 0x144;
    static constexpr std::uint32_t kPitchSendId = 0x04; // DM esc id（指令）
    static constexpr std::uint32_t kPitchRecvId = 0x03; // DM master id（反馈）
    static constexpr std::uint32_t kDjiSendId = 0x200;  // 3508 四合一帧

    class BalanceInfantryCommand : public hcs_executor::Component {
    public:
        explicit BalanceInfantryCommand(BalanceInfantry& owner)
            : owner_(owner) {}

        void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
            owner_.command_update(tick);
        }

    private:
        BalanceInfantry& owner_;
    };

    class BoardCallback final : public libhcs::board::Hpm5321::Callback {
    public:
        BoardCallback(BalanceInfantry& owner, std::size_t board)
            : owner_(owner)
            , board_(board) {}

        void can_receive(
            libhcs::board::hcs::CanPort port,
            const libhcs::data::CanDataView& data) override {
            owner_.on_can_receive(board_, port, data);
        }

        void uart0_receive_callback(const libhcs::data::UartDataView& data) override {
            owner_.on_uart0_receive(board_, data);
        }

    private:
        BalanceInfantry& owner_;
        std::size_t board_;
    };

    // ── 反馈分发（libhcs IO 线程，事件域）──────────────────────────────────
    void on_can_receive(
        std::size_t board, libhcs::board::hcs::CanPort port,
        const libhcs::data::CanDataView& data) noexcept {
        if (data.is_extended_can_id || data.is_remote_transmission)
            return;
        const auto bytes = data.can_data;

        switch (board) {
        case kGimbal:
            if (port == libhcs::board::hcs::CanPort::kCan1) {
                if (data.can_id == kYawCanId)
                    yaw_lk_.store_status(bytes);
                else if (data.can_id == kDialCanId)
                    dial_lk_.store_status(bytes);
            } else if (port == libhcs::board::hcs::CanPort::kCan2) {
                if (data.can_id == 0x201)
                    friction_left_wheel_.store_status(bytes);
                else if (data.can_id == 0x202)
                    friction_right_wheel_.store_status(bytes);
                else if (data.can_id == kPitchRecvId)
                    pitch_dm_.store_status(bytes);
            }
            break;
        case kChassis:
            // 左腿在 CAN1、右腿在 CAN2；master id 见文件头注释。
            if (port == libhcs::board::hcs::CanPort::kCan1) {
                if (data.can_id == 0x05)
                    left_back_joint_.store_status(bytes);
                else if (data.can_id == 0x06)
                    left_front_joint_.store_status(bytes);
            } else if (port == libhcs::board::hcs::CanPort::kCan2) {
                if (data.can_id == 0x07)
                    right_back_joint_.store_status(bytes);
                else if (data.can_id == 0x08)
                    right_front_joint_.store_status(bytes);
            }
            break;
        case kAux:
            if (port == libhcs::board::hcs::CanPort::kCan1) {
                if (data.can_id == 0x201)
                    right_wheel_.store_status(bytes);
                else if (data.can_id == 0x202)
                    left_wheel_.store_status(bytes);
            }
            break;
        default: break;
        }
    }

    void on_uart0_receive(std::size_t board, const libhcs::data::UartDataView& data) noexcept {
        switch (board) {
        case kGimbal: gimbal_imu_.store_status(data.uart_data); break;
        case kChassis: chassis_imu_.store_status(data.uart_data); break;
        case kAux: vt13_.store_status(data.uart_data); break;
        default: break;
        }
    }

    // ── 指令侧（Command 伙伴，周期域）────────────────────────────────────
    void command_update(const hcs_sync::Tick& tick) HCS_NONBLOCKING {
        util::TransmitBatch batch;
        batch.sequence = ++batch_sequence_;

        if (gimbal_board_) {
            // LK 驱动对 NaN 自动发失能帧（yaw 只接了 control_torque）。
            util::push_frame(batch, kGimbal, libhcs::board::hcs::CanPort::kCan1, kYawCanId,
                       yaw_lk_.generate_command());
            util::push_frame(batch, kGimbal, libhcs::board::hcs::CanPort::kCan1, kDialCanId,
                       dial_lk_.generate_command());
            // 摩擦轮这期不接控制器：NaN → 0 电流。
            device::CanPacket8 friction_packet{
                device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{},
                device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{}};
            friction_packet << friction_left_wheel_;
            friction_packet << friction_right_wheel_;
            util::push_frame(batch, kGimbal, libhcs::board::hcs::CanPort::kCan2, kDjiSendId,
                       friction_packet);
            util::push_frame(batch, kGimbal, libhcs::board::hcs::CanPort::kCan2, kPitchSendId,
                       dm_joint_frame(pitch_dm_));
        }

        if (chassis_board_) {
            util::push_frame(batch, kChassis, libhcs::board::hcs::CanPort::kCan1, 0x01,
                       dm_joint_frame(left_back_joint_));
            util::push_frame(batch, kChassis, libhcs::board::hcs::CanPort::kCan1, 0x02,
                       dm_joint_frame(left_front_joint_));
            util::push_frame(batch, kChassis, libhcs::board::hcs::CanPort::kCan2, 0x03,
                       dm_joint_frame(right_back_joint_));
            util::push_frame(batch, kChassis, libhcs::board::hcs::CanPort::kCan2, 0x04,
                       dm_joint_frame(right_front_joint_));
        }

        if (aux_board_) {
            device::CanPacket8 wheel_packet{
                device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{},
                device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{}};
            wheel_packet << right_wheel_;
            wheel_packet << left_wheel_;
            util::push_frame(batch, kAux, libhcs::board::hcs::CanPort::kCan1, kDjiSendId,
                       wheel_packet);
        }

        transmitter_->publish(batch, tick.scheduled);
    }

    /// DM 关节的"这一拍该发的帧"：torque 为 NaN（失能/摔倒/隔离复位）时发 0xFD，
    /// 否则交给驱动（它自己负责使能和清错）。
    static device::CanPacket8 dm_joint_frame(device::DmMotor& motor) noexcept {
        if (std::isnan(motor.control_torque()))
            return device::DmMotor::generate_disable_command();
        return motor.generate_command();
    }

    // ── IMU 桥接 ──────────────────────────────────────────────────────────
    void update_imu() HCS_NONBLOCKING {
        // 云台 CH040：机体角速度直接喂云台速度环（REP-103 FLU：yaw 取 z、
        // pitch 取 y，与 RMCS 的 BMI088 用法一致，符号上台架核对）；
        // 四元数写进 /tf（照 RMCS 用 BMI088 的写法，换成 Hipnuc 的 /quaternion 共轭）。
        *gimbal_yaw_velocity_imu_ = gimbal_imu_.angular_velocity().z();
        *gimbal_pitch_velocity_imu_ = gimbal_imu_.angular_velocity().y();
        tf_->set_state<hcs_description::GimbalCenterLink, hcs_description::YawLink>(
            yaw_lk_.angle());
        tf_->set_state<hcs_description::YawLink, hcs_description::PitchLink>(pitch_dm_.angle());
        // ROS 2 / REP-103 约定：sensor_msgs 的四元数是"机体 → ENU 世界系"的
        // 旋转（R_world←sensor）；fast_tf 的 OdomImu 关节变换是父系→子系
        //（= R_level←body），方向正好一致，直接用，不取共轭。
        //（RMCS 的 BMI088 EKF 输出的是反方向四元数，那边才需要 conjugate。）
        // 前提：云台 CH040 与 pitch 链同轴正装（x 前、y 左、z 上）。
        tf_->set_transform<hcs_description::PitchLink, hcs_description::OdomImu>(
            gimbal_imu_.quaternion());

        // 底盘 CH040：原样发布，安装角与轴序的折算（Helios 里混在
        // Chassis_Data_Update 的符号约定中）全部留给估计器按参数处理。
        const auto& euler = chassis_imu_.euler_angles(); // roll, pitch, yaw
        *chassis_imu_roll_ = euler.x();
        *chassis_imu_pitch_ = euler.y();
        *chassis_imu_yaw_ = euler.z();
        const auto& gyro = chassis_imu_.angular_velocity();
        *chassis_imu_roll_rate_ = gyro.x();
        *chassis_imu_pitch_rate_ = gyro.y();
        *chassis_imu_yaw_rate_ = gyro.z();
        // 加速度（传感器系，m/s^2）由 Hipnuc 直接输出 /chassis/imu/acceleration。
    }

    // ── 构造期配置 ────────────────────────────────────────────────────────
    struct LegJointSpec {
        device::DmMotor* motor;
        std::uint32_t esc_id;   // 指令 id
        std::uint32_t master_id; // 反馈 id
        const char* zero_param;
        bool reversed;
    };

    void configure_devices() {
        const double leg_position_max = double_parameter("leg_position_max", 6.283185);
        const double leg_velocity_max = double_parameter("leg_velocity_max", 45.0);
        const double leg_torque_max = double_parameter("leg_torque_max", 40.0);

        // motor_leg[0..3] = bl1、bl0、br1、br0（balance_tasks.cpp INF_4 分支）。
        // reversed 一律 false：Helios 对腿电机的取负都不在驱动层——
        // 左腿是运动学角度取负（估计器按参数折算），右腿是发送力矩取负
        // （LegJointController 的符号参数），这样反馈与 Helios 的 motordata 完全一致。
        const LegJointSpec leg_joints[] = {
            {&left_front_joint_, 0x02, 0x06, "left_front_joint_zero", false},
            {&left_back_joint_, 0x01, 0x05, "left_back_joint_zero", false},
            {&right_front_joint_, 0x04, 0x08, "right_front_joint_zero", false},
            {&right_back_joint_, 0x03, 0x07, "right_back_joint_zero", false},
        };
        for (const auto& spec : leg_joints) {
            device::DmMotor::Config config{device::DmMotor::Type::kJ4310, spec.esc_id,
                                           spec.master_id};
            config.set_position_max(leg_position_max)
                .set_velocity_max(leg_velocity_max)
                .set_torque_max(leg_torque_max)
                .set_encoder_zero_point(
                    dm_raw_zero(double_parameter(spec.zero_param, 0.0), leg_position_max));
            // 多圈：自救/上台阶时髋关节相对机体可转过整圈。PMAX≈2π 使单圈
            // 解码的运动学近似不变，但会积累 3e-7 rad/圈 的映射漂移，开多圈
            // 与 Helios 的 MTurnProc 行为一致。
            config.enable_multi_turn_angle();
            if (spec.reversed)
                config.set_reversed();
            spec.motor->configure(config);
        }

        // Helios DM_MOTOR(0x03, 0x04, ..., 12.566, 30, 1, 1, 40, 0.0285701752)。
        device::DmMotor::Config pitch_config{
            device::DmMotor::Type::kJ4310, kPitchSendId, kPitchRecvId};
        pitch_config.set_position_max(double_parameter("pitch_position_max", 12.566))
            .set_velocity_max(double_parameter("pitch_velocity_max", 30.0))
            .set_torque_max(double_parameter("pitch_torque_max", 40.0))
            .set_encoder_zero_point(
                dm_raw_zero(double_parameter("pitch_zero", 0.0), 12.566));
        pitch_config.set_reversed(); // dir_pitch = -1
        pitch_dm_.configure(pitch_config);

        // Helios LK_motor(5, 5010, ..., 0.867753744)；Mid_Angle 是弧度，换算成 raw 计数。
        device::LkMotor::Config yaw_config{device::LkMotor::Type::kMG5010Ei10};
        yaw_config.set_encoder_zero_point(lk_raw_zero(double_parameter("yaw_zero", 0.0)));
        yaw_config.set_reversed(); // dir_yaw = -1
        yaw_lk_.configure(yaw_config);
        dial_lk_.configure(device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei10});

        const double wheel_reduction = double_parameter("wheel_reduction_ratio", 13.94);
        device::DjiMotor::Config left_wheel_config{device::DjiMotor::Type::kM3508, 2};
        left_wheel_config.set_reduction_ratio(wheel_reduction);
        left_wheel_.configure(left_wheel_config);
        device::DjiMotor::Config right_wheel_config{device::DjiMotor::Type::kM3508, 1};
        right_wheel_config.set_reduction_ratio(wheel_reduction).set_reversed();
        right_wheel_.configure(right_wheel_config);

        device::DjiMotor::Config friction_left_config{device::DjiMotor::Type::kM3508, 1};
        friction_left_wheel_.configure(friction_left_config);
        device::DjiMotor::Config friction_right_config{device::DjiMotor::Type::kM3508, 2};
        friction_right_wheel_.configure(friction_right_config);
    }

    /// Helios 的 org_pos 是弧度（decoded − org_pos），HCS 的 zero_point 是 raw 计数。
    /// DM 解码 raw=32768 对应 0 rad：zero = 32768 + org·65535/(2·PMAX)。
    static int dm_raw_zero(double zero_rad, double position_max) {
        const auto zero =
            std::lround(32768.0 + zero_rad * 65535.0 / (2.0 * position_max));
        return static_cast<int>(std::clamp<long>(zero, 0, 65535));
    }

    /// LK 的 raw 计数域是 [0, 65536) ↔ [0, 2π)。
    static int lk_raw_zero(double zero_rad) {
        constexpr double two_pi = 2.0 * std::numbers::pi;
        const auto zero = std::lround(zero_rad * 65536.0 / two_pi);
        return static_cast<int>(((zero % 65536) + 65536) % 65536);
    }

    void create_board(
        const char* name, libhcs::board::Hpm5321::Callback& callback,
        std::unique_ptr<libhcs::board::Hpm5321>& board) {
        const std::string prefix = std::string(name) + "_";
        if (!bool_parameter(prefix + "enabled", false)) {
            RCLCPP_WARN(
                get_logger(), "[%s] board %s disabled by parameter; its devices stay offline",
                get_component_name().c_str(), name);
            return;
        }

        auto options = libhcs::board::AdvancedOptions{};
        options.dangerously_skip_version_checks =
            bool_parameter("dangerously_skip_version_checks", false);
        const auto io_cpu = int_parameter(prefix + "io_thread_cpu", -1);
        if (io_cpu >= 0)
            options.set_io_thread_affinity(
                static_cast<int>(io_cpu),
                static_cast<int>(int_parameter(prefix + "io_thread_rt_priority", 0)));

        board = std::make_unique<libhcs::board::Hpm5321>(
            callback, string_parameter(prefix + "serial_filter", ""), options);

        using LinkState = libhcs::host::protocol::Handler::LinkState;
        if (board->link_state() == LinkState::kFaulted)
            throw std::runtime_error(
                std::format("{}: board {} faulted at construction", get_component_name(), name));

        const auto baudrate =
            static_cast<std::uint32_t>(int_parameter(prefix + "uart0_baudrate", 921600));
        board->configure_uart0(baudrate);

        RCLCPP_INFO(
            get_logger(), "[%s] board %s: serial_filter='%s' io_cpu=%lld io_prio=%lld uart0=%u",
            get_component_name().c_str(), name,
            string_parameter(prefix + "serial_filter", "").c_str(),
            static_cast<long long>(io_cpu),
            static_cast<long long>(int_parameter(prefix + "io_thread_rt_priority", 0)),
            baudrate);
    }

    /// 总线上的电机都是经典 CAN 2.0，固件刷成 FD 直接拒绝（§4.4）。
    void check_classic_can() {
        const std::pair<const std::unique_ptr<libhcs::board::Hpm5321>&, const char*> boards[] = {
            {gimbal_board_, "gimbal"}, {chassis_board_, "chassis"}, {aux_board_, "aux"}};
        for (const auto& [board, name] : boards) {
            if (!board)
                continue;
            if (board->can1_is_fd() || board->can2_is_fd())
                throw std::runtime_error(
                    std::format(
                        "{}: board {} reports CAN FD but all motors are classic CAN 2.0; "
                        "reflash the firmware",
                        get_component_name(), name));
        }
    }

    /// 全失能批次：DM 0xFD、LK 失能、DJI 0 电流。隔离后由发送线程补发一次。
    /// 批次按整车接线全量构建：禁用的板在发送线程里被跳过（板指针为空），
    /// 这样总线负载估算也能在台架形态（板全部禁用）下给出规划值。
    util::TransmitBatch build_safe_batch() const {
        util::TransmitBatch batch;
        // gimbal 板：yaw、拨盘、摩擦轮、pitch
        {
            util::push_frame(
                batch, kGimbal, libhcs::board::hcs::CanPort::kCan1, kYawCanId,
                device::LkMotor::generate_disable_command());
            util::push_frame(
                batch, kGimbal, libhcs::board::hcs::CanPort::kCan1, kDialCanId,
                device::LkMotor::generate_disable_command());
            util::push_frame(
                batch, kGimbal, libhcs::board::hcs::CanPort::kCan2, kDjiSendId,
                zero_packet());
            util::push_frame(
                batch, kGimbal, libhcs::board::hcs::CanPort::kCan2, kPitchSendId,
                device::DmMotor::generate_disable_command());
        }
        // chassis 板：四个腿关节
        {
            util::push_frame(
                batch, kChassis, libhcs::board::hcs::CanPort::kCan1, 0x01,
                device::DmMotor::generate_disable_command());
            util::push_frame(
                batch, kChassis, libhcs::board::hcs::CanPort::kCan1, 0x02,
                device::DmMotor::generate_disable_command());
            util::push_frame(
                batch, kChassis, libhcs::board::hcs::CanPort::kCan2, 0x03,
                device::DmMotor::generate_disable_command());
            util::push_frame(
                batch, kChassis, libhcs::board::hcs::CanPort::kCan2, 0x04,
                device::DmMotor::generate_disable_command());
        }
        // aux 板：两个轮毂
        util::push_frame(
            batch, kAux, libhcs::board::hcs::CanPort::kCan1, kDjiSendId, zero_packet());
        return batch;
    }

    static device::CanPacket8 zero_packet() {
        return device::CanPacket8{
            device::CanPacket8::Quarter{0}, device::CanPacket8::Quarter{0},
            device::CanPacket8::Quarter{0}, device::CanPacket8::Quarter{0}};
    }

    /// 每路总线的负载估算：帧数/拍 × 拍率 × 130 µs（经典 CAN 1 Mbit/s、8 字节帧
    /// 含填充与仲裁的上界）。超过 70% 打 WARN——那已经会挤掉反馈帧了。
    void report_bus_load() const {
        struct BusLoad {
            std::uint32_t gimbal[2]{};
            std::uint32_t chassis[2]{};
            std::uint32_t aux[2]{};
        };
        BusLoad load{};
        const auto batch = build_safe_batch();
        for (std::uint32_t index = 0; index < batch.frame_count; ++index) {
            const auto& frame = batch.frames[index];
            const auto port_index = frame.port == 2 ? 1 : 0; // kCan1=1, kCan2=2
            switch (frame.board) {
            case kGimbal: load.gimbal[port_index]++; break;
            case kChassis: load.chassis[port_index]++; break;
            case kAux: load.aux[port_index]++; break;
            default: break;
            }
        }

        const double update_rate = double_parameter("expected_update_rate", 1000.0);
        constexpr double kFrameSeconds = 130e-6;
        const auto percent = [&](std::uint32_t frames) {
            return frames * update_rate * kFrameSeconds * 100.0;
        };

        RCLCPP_INFO(
            get_logger(),
            "[%s] bus load estimate at %.0f Hz (classic CAN, 130 us/frame): "
            "gimbal can1=%.0f%% can2=%.0f%% | chassis can1=%.0f%% can2=%.0f%% | "
            "aux can1=%.0f%%",
            get_component_name().c_str(), update_rate, percent(load.gimbal[0]),
            percent(load.gimbal[1]), percent(load.chassis[0]), percent(load.chassis[1]),
            percent(load.aux[0]));

        const auto warn_if_overloaded = [&](const char* bus, double load_percent) {
            if (load_percent > 70.0)
                RCLCPP_WARN(
                    get_logger(), "[%s] bus %s at %.0f%% estimated load (>70%%)",
                    get_component_name().c_str(), bus, load_percent);
        };
        warn_if_overloaded("gimbal/can1", percent(load.gimbal[0]));
        warn_if_overloaded("gimbal/can2", percent(load.gimbal[1]));
        warn_if_overloaded("chassis/can1", percent(load.chassis[0]));
        warn_if_overloaded("chassis/can2", percent(load.chassis[1]));
        warn_if_overloaded("aux/can1", percent(load.aux[0]));
    }

    // ── 尽力域：板卡故障监控 ──────────────────────────────────────────────
    void report() {
        using LinkState = libhcs::host::protocol::Handler::LinkState;
        const std::pair<const std::unique_ptr<libhcs::board::Hpm5321>&, const char*> boards[] = {
            {gimbal_board_, "gimbal"}, {chassis_board_, "chassis"}, {aux_board_, "aux"}};
        for (const auto& [board, name] : boards) {
            if (!board || board->link_state() != LinkState::kFaulted)
                continue;
            if (bool_parameter("exit_on_board_fault", true)) {
                RCLCPP_FATAL(
                    get_logger(), "[%s] board %s faulted (link lost); shutting down",
                    get_component_name().c_str(), name);
                rclcpp::shutdown();
                return;
            }
            RCLCPP_ERROR(
                get_logger(), "[%s] board %s faulted (link lost); its devices are offline",
                get_component_name().c_str(), name);
        }
    }

    // ── 参数 helper（照 hcs_link_probe.cpp）───────────────────────────────
    std::string string_parameter(const std::string& name, const std::string& fallback) const {
        std::string value = fallback;
        get_parameter_or(name, value, fallback);
        return value;
    }

    std::int64_t int_parameter(const std::string& name, std::int64_t fallback) const {
        std::int64_t value = fallback;
        get_parameter_or(name, value, fallback);
        return value;
    }

    double double_parameter(const std::string& name, double fallback) const {
        double value = fallback;
        get_parameter_or(name, value, fallback);
        return value;
    }

    bool bool_parameter(const std::string& name, bool fallback) const {
        bool value = fallback;
        get_parameter_or(name, value, fallback);
        return value;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    std::shared_ptr<BalanceInfantryCommand> command_;
    std::uint32_t batch_sequence_ = 0; ///< 指令侧批次序号（周期域）

    // 设备（指令接口注册在 Command 伙伴上，反馈输出注册在本组件上）。
    device::LkMotor yaw_lk_;
    device::LkMotor dial_lk_;
    device::DmMotor pitch_dm_;
    device::DjiMotor friction_left_wheel_;
    device::DjiMotor friction_right_wheel_;
    device::DmMotor left_front_joint_;
    device::DmMotor left_back_joint_;
    device::DmMotor right_front_joint_;
    device::DmMotor right_back_joint_;
    device::DjiMotor left_wheel_;
    device::DjiMotor right_wheel_;
    device::Hipnuc gimbal_imu_;
    device::Hipnuc chassis_imu_;
    device::Vt13 vt13_;
    device::RemoteControl remote_control_;

    // 板卡回调：构造期就可能进回调，引用的都是上面的设备。
    BoardCallback gimbal_callback_;
    BoardCallback chassis_callback_;
    BoardCallback aux_callback_;

    std::unique_ptr<libhcs::board::Hpm5321> gimbal_board_;
    std::unique_ptr<libhcs::board::Hpm5321> chassis_board_;
    std::unique_ptr<libhcs::board::Hpm5321> aux_board_;

    // 析构逆序：发送线程（先停）→ 板卡（停 IO 线程）→ 回调 → 设备。
    std::unique_ptr<util::BoardTransmitter> transmitter_;

    hcs_executor::Component::OutputInterface<hcs_description::Tf> tf_;
    hcs_executor::Component::OutputInterface<double> gimbal_yaw_velocity_imu_;
    hcs_executor::Component::OutputInterface<double> gimbal_pitch_velocity_imu_;
    hcs_executor::Component::OutputInterface<double> chassis_imu_pitch_;
    hcs_executor::Component::OutputInterface<double> chassis_imu_roll_;
    hcs_executor::Component::OutputInterface<double> chassis_imu_yaw_;
    hcs_executor::Component::OutputInterface<double> chassis_imu_pitch_rate_;
    hcs_executor::Component::OutputInterface<double> chassis_imu_roll_rate_;
    hcs_executor::Component::OutputInterface<double> chassis_imu_yaw_rate_;

    rclcpp::TimerBase::SharedPtr report_timer_;
};

} // namespace hcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::hardware::BalanceInfantry, hcs_executor::Component)
