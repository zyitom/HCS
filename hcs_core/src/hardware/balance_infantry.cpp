#include <libhcs/board/hpm5321.hpp>

#include "hardware/board/board.hpp"
#include "hardware/board/devices.hpp"

namespace hcs_core::hardware {
namespace {

using namespace board; // NOLINT(google-build-using-namespace): 接线表只用 board 的词汇

// ============================================================================
// 平衡步兵接线表：三块 HPM5321，一块板一个组件，成员就是它的口和口上的设备。
// 板型与端口直接用 libhcs 的（libhcs::board::Hpm5321、Spec::kCans / Spec::kUarts）；
// 换成 mc02 就是 Board<libhcs::board::Mc02> 加它自己的描述符（Spec::kUarts.kDbus 等）。
// 上车前由用户核对：全部 id 与零点、DM 电机里实际存的 PMAX/VMAX/TMAX、各串口设备的波特率。
// 部署项（板卡使能、serial_filter、线程）在 hcs_bringup/config/balance-infantry.yaml；
// 哪些设备坏了要整车失能在那里的 safety_latch.critical_devices。
//
// 本车全部电机都是力矩模式：所有闭环都在上位机。
// ============================================================================

using libhcs::board::Hpm5321;
using DmMode = device::DmMotor::ControlMode;
using LkMode = device::LkMotor::ControlMode;
using LkType = device::LkMotor::Type;
using DjiType = device::DjiMotor::Type;
using ImuFrame = device::Hipnuc::Config::ModuleFrame;

// 四条腿刷成同一组 MIT 映射范围（DM 寄存器 PMAX/VMAX/TMAX，见 DmMotor::MitRange）。
constexpr device::DmMotor::MitRange kLegRange{6.283185, 45.0, 40.0};

// 云台板：CAN1 LK yaw / 拨盘，CAN2 DM pitch + 3508 摩擦轮，UART0 云台 CH040
struct Gimbal : Board<Hpm5321> {
    CanBus can1{*this, Spec::kCans.kCan1, kClassic1M};
    CanBus can2{*this, Spec::kCans.kCan2, kClassic1M};
    SerialPort uart0{*this, Spec::kUarts.kUart0};

    LkMotor yaw{can1, "/gimbal/yaw", {
        .motor_type = LkType::kMG5010Ei10, .control_mode = LkMode::kTorque, .can_id = 0x145,
        .encoder_zero_point = 9051, // org_pos 0.867753744
        .reversed = true,           // dir_yaw = -1
    }};
    LkMotor dial{can1, "/gimbal/dial", {
        .motor_type = LkType::kMG5010Ei10, .control_mode = LkMode::kTorque, .can_id = 0x144,
    }};

    // 指令 0x04 / 反馈 0x03，寄存器被刷过
    DmMotor pitch{can2, "/gimbal/pitch", {
        .control_mode = DmMode::kTorque, .esc_id = 0x04, .master_id = 0x03,
        .mit_range = {12.566, 30.0, 40.0},
        .zero_angle = 0.0285701752, // Helios org_pos
        .reversed = true,           // dir_pitch = -1
    }};
    DjiMotor friction_left{can2, "/gimbal/left_friction_wheel", {.motor_type = DjiType::kM3508, .id = 1}};
    DjiMotor friction_right{can2, "/gimbal/right_friction_wheel", {.motor_type = DjiType::kM3508, .id = 2}};

    // 模块 flash 坐标系 NWU：机体系已是 FLU，直通
    Hipnuc imu{uart0, "/gimbal/imu", {.baudrate = 921600, .module_frame = ImuFrame::kNwu}};
};

// 底盘板：CAN1 左腿、CAN2 右腿（Helios bl0/br0 = front，bl1/br1 = back），UART0 底盘 CH040。
// 腿关节驱动层不反转（左腿在估计器取负、右腿在 LegJointController 取负），多圈对齐 Helios MTTurnProc。
struct Chassis : Board<Hpm5321> {
    CanBus can1{*this, Spec::kCans.kCan1, kClassic1M};
    CanBus can2{*this, Spec::kCans.kCan2, kClassic1M};
    SerialPort uart0{*this, Spec::kUarts.kUart0};

    DmMotor left_front_joint{can1, "/chassis/left_front_joint", {
        .control_mode = DmMode::kTorque, .esc_id = 0x02, .master_id = 0x06, .mit_range = kLegRange,
        .zero_angle = 0.717242718, .multi_turn_angle_enabled = true,
    }};
    DmMotor left_back_joint{can1, "/chassis/left_back_joint", {
        .control_mode = DmMode::kTorque, .esc_id = 0x01, .master_id = 0x05, .mit_range = kLegRange,
        .zero_angle = 1.938694, .multi_turn_angle_enabled = true,
    }};
    DmMotor right_front_joint{can2, "/chassis/right_front_joint", {
        .control_mode = DmMode::kTorque, .esc_id = 0x04, .master_id = 0x08, .mit_range = kLegRange,
        .zero_angle = 1.39220476, .multi_turn_angle_enabled = true,
    }};
    DmMotor right_back_joint{can2, "/chassis/right_back_joint", {
        .control_mode = DmMode::kTorque, .esc_id = 0x03, .master_id = 0x07, .mit_range = kLegRange,
        .zero_angle = -1.37379646, .multi_turn_angle_enabled = true,
    }};

    // 模块 flash 坐标系 ENU：驱动把机体系 RFU 转成 FLU
    Hipnuc imu{uart0, "/chassis/imu", {.baudrate = 921600, .module_frame = ImuFrame::kEnu}};
};

// 辅助板：CAN1 两个 3508 轮毂，UART0 VT13 遥控接收机（波特率待上台架核对）。
// 转子到轮 13.94（Helios REDUCTION_RATIO_WHEEL，替换 3508 默认的 19.2）；
// 右轮速度与电流一起取负 = 驱动层一次反转。
struct Aux : Board<Hpm5321> {
    CanBus can1{*this, Spec::kCans.kCan1, kClassic1M};
    SerialPort uart0{*this, Spec::kUarts.kUart0};

    DjiMotor left_wheel{can1, "/chassis/left_wheel", {
        .motor_type = DjiType::kM3508, .id = 2, .reduction_ratio = 13.94,
    }};
    DjiMotor right_wheel{can1, "/chassis/right_wheel", {
        .motor_type = DjiType::kM3508, .id = 1, .reduction_ratio = 13.94, .reversed = true,
    }};

    Vt13Remote remote{uart0, "/remote", {.baudrate = 115200}};
};

} // namespace

using BalanceGimbalBoard = board::BoardComponent<Gimbal>;
using BalanceChassisBoard = board::BoardComponent<Chassis>;
using BalanceAuxBoard = board::BoardComponent<Aux>;

} // namespace hcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::hardware::BalanceGimbalBoard, hcs_executor::Component)
PLUGINLIB_EXPORT_CLASS(hcs_core::hardware::BalanceChassisBoard, hcs_executor::Component)
PLUGINLIB_EXPORT_CLASS(hcs_core::hardware::BalanceAuxBoard, hcs_executor::Component)
