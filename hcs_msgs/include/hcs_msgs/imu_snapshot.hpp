#pragma once

#include <eigen3/Eigen/Geometry>

#include <hcs_msgs/board_clock.hpp>

namespace hcs_msgs {

/// 某一个板上时刻的姿态与角速度。带时间戳是为了和同一块板打了时间戳的别的东西（相机触发）对齐。
struct ImuSnapshot {
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_body = Eigen::Vector3d::Zero(); ///< rad/s，机体系
    BoardClock::time_point timestamp{};
};

} // namespace hcs_msgs
