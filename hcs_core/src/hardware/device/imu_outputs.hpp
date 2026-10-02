#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <eigen3/Eigen/Dense>

#include <hcs_executor/component.hpp>

namespace hcs_core::hardware::device {

/// 一个 IMU 对外的那一组输出，不管它是哪种传感器、接在哪种端口上：
///
///   <名字>/quaternion          机体系（FLU：x 前 y 左 z 上）相对世界系的姿态
///   <名字>/angular_velocity    rad/s，机体系
///   <名字>/acceleration        m/s²，机体系
///   <名字>/euler/{roll,pitch,yaw}            rad
///   <名字>/angular_velocity/{x,y,z}          rad/s，同上面那个向量的三个分量
///
/// 结构化输出（三个向量）给需要整个量的消费者（自瞄吃四元数、估计器吃加速度）；标量给只读一路
/// 的消费者（云台速度环、姿态角）。轴名由**传感器**决定——一个 IMU 就是 roll / pitch / yaw 三轴，
/// 与下游要什么无关，所以不需要别处的适配表去替它挑轴。
///
/// 欧拉角的初值是 NaN 而不是 0：它们是姿态量，没有"零姿态"这种合法默认；NaN 也让组件被隔离后的
/// 复位值天然带着"不可用"的语义。角速度的零是合法的，用 0。
class ImuOutputs {
public:
    struct State {
        Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
        Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
        Eigen::Vector3d euler_angles = Eigen::Vector3d::Zero(); ///< roll, pitch, yaw
    };

    ImuOutputs(hcs_executor::Component& component, const std::string& name) {
        component.register_output(
            name + "/quaternion", quaternion_, Eigen::Quaterniond::Identity());
        component.register_output(
            name + "/angular_velocity", angular_velocity_, Eigen::Vector3d::Zero());
        component.register_output(name + "/acceleration", acceleration_, Eigen::Vector3d::Zero());

        component.register_output(name + "/euler/roll", euler_roll_, kNan);
        component.register_output(name + "/euler/pitch", euler_pitch_, kNan);
        component.register_output(name + "/euler/yaw", euler_yaw_, kNan);
        component.register_output(name + "/angular_velocity/x", angular_velocity_x_, 0.0);
        component.register_output(name + "/angular_velocity/y", angular_velocity_y_, 0.0);
        component.register_output(name + "/angular_velocity/z", angular_velocity_z_, 0.0);
    }

    /// 周期域：有新的解算结果时调。没调的那些拍，输出保持上一次的值。
    void publish(const State& state) noexcept {
        *quaternion_ = state.quaternion;
        *angular_velocity_ = state.angular_velocity;
        *acceleration_ = state.acceleration;

        *euler_roll_ = state.euler_angles[0];
        *euler_pitch_ = state.euler_angles[1];
        *euler_yaw_ = state.euler_angles[2];

        *angular_velocity_x_ = state.angular_velocity[0];
        *angular_velocity_y_ = state.angular_velocity[1];
        *angular_velocity_z_ = state.angular_velocity[2];
    }

    /// 四元数 → roll / pitch / yaw（先绕 z 转 yaw，再绕 y 转 pitch，最后绕 x 转 roll）。
    /// 给自己不报欧拉角的传感器用；自己报的（HiPNUC）直接用它报的。
    [[nodiscard]] static Eigen::Vector3d euler_from(const Eigen::Quaterniond& q) noexcept {
        const double sin_pitch = std::clamp(2.0 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0);
        const double roll = std::atan2(
            2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
        const double yaw = std::atan2(
            2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
        return {roll, std::asin(sin_pitch), yaw};
    }

private:
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    hcs_executor::Component::OutputInterface<Eigen::Quaterniond> quaternion_;
    hcs_executor::Component::OutputInterface<Eigen::Vector3d> angular_velocity_;
    hcs_executor::Component::OutputInterface<Eigen::Vector3d> acceleration_;

    // 逐轴标量都是 double（平凡可拷贝），所以 register_output 会给它们装复位钩子：组件被隔离时
    // 这几个会回到初值，而上面三个向量输出不会（Eigen 类型非平凡）。
    hcs_executor::Component::OutputInterface<double> euler_roll_;
    hcs_executor::Component::OutputInterface<double> euler_pitch_;
    hcs_executor::Component::OutputInterface<double> euler_yaw_;
    hcs_executor::Component::OutputInterface<double> angular_velocity_x_;
    hcs_executor::Component::OutputInterface<double> angular_velocity_y_;
    hcs_executor::Component::OutputInterface<double> angular_velocity_z_;
};

} // namespace hcs_core::hardware::device
