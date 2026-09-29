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
// 方向矢量的 cast 不经过平移，云台回转中心的安装高度对云台解算无影响；
// MuzzleLink 留到弹道解算时再补。
//
//   PitchLink
//     └─ CameraLink（相机安装位姿，Isometry3d，自瞄 UI 反投影用）

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

struct CameraLink : fast_tf::Link<CameraLink> {
    static constexpr char name[] = "camera_link";
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

template <>
struct fast_tf::Joint<hcs_description::CameraLink> : fast_tf::ModificationTrackable {
    using Parent = hcs_description::PitchLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
};

namespace hcs_description {

using Tf = fast_tf::JointCollection<GimbalCenterLink, YawLink, PitchLink, OdomImu, CameraLink>;

} // namespace hcs_description
