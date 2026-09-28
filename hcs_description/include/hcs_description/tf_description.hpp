#pragma once

#include <fast_tf/fast_tf.hpp>

#include <fast_tf/impl/link.hpp>

namespace hcs_description {

// 平衡步兵的坐标系树（移植自 rmcs_description，只保留当前用到的链接）：
//
//   BaseLink（底盘根）
//     └─ GimbalCenterLink（云台回转中心，平移默认单位）
//          └─ YawLink（绕 Z，yaw 电机反馈）
//               └─ PitchLink（绕 Y，pitch 电机反馈）
//                    └─ OdomImu（云台 CH040 四元数，每拍由硬件组件写入）
//
// 方向矢量的 cast 不经过平移，云台回转中心的安装高度对云台解算无影响，
// 需要位置级 tf（自瞄、弹道）时再补 MuzzleLink/CameraLink。

struct BaseLink : fast_tf::Link<BaseLink> {
    static constexpr char name[] = "base_link";
};

struct GimbalCenterLink : fast_tf::Link<GimbalCenterLink> {
    static constexpr char name[] = "gimbal_center_link";
};

struct YawLink : fast_tf::Link<YawLink> {
    static constexpr char name[] = "yaw_link";
};

struct PitchLink : fast_tf::Link<PitchLink> {
    static constexpr char name[] = "pitch_link";
};

struct OdomImu : fast_tf::Link<OdomImu> {
    static constexpr char name[] = "odom_imu";
};

} // namespace hcs_description

template <>
struct fast_tf::Joint<hcs_description::GimbalCenterLink> : fast_tf::ModificationTrackable {
    using Parent = hcs_description::BaseLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<hcs_description::YawLink> : fast_tf::ModificationTrackable {
    using Parent = hcs_description::GimbalCenterLink;

    void set_state(double angle) { angle_ = angle; }
    auto get_transform() const { return Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitZ()}; }

private:
    double angle_;
};

template <>
struct fast_tf::Joint<hcs_description::PitchLink> : fast_tf::ModificationTrackable {
    using Parent = hcs_description::YawLink;

    void set_state(double angle) { angle_ = angle; }
    auto get_transform() const { return Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitY()}; }

private:
    double angle_;
};

template <>
struct fast_tf::Joint<hcs_description::OdomImu> : fast_tf::ModificationTrackable {
    using Parent = hcs_description::PitchLink;
    Eigen::Quaterniond transform = Eigen::Quaterniond::Identity();
};

namespace hcs_description {

using Tf = fast_tf::JointCollection<GimbalCenterLink, YawLink, PitchLink, OdomImu>;

} // namespace hcs_description
