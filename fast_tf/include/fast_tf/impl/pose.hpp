#pragma once

#include <array>
#include <cstddef>
#include <tuple>

#include <eigen3/Eigen/Geometry>

#include "fast_tf/impl/joint.hpp"
#include "fast_tf/impl/joint_collection.hpp"

namespace fast_tf {

namespace internal {

template <internal::is_transform TransformT>
inline std::tuple<Eigen::Translation3d, Eigen::Quaterniond>
    extract_translation_rotation(const TransformT& transform) {
    return {
        static_cast<Eigen::Translation3d>(transform.translation()),
        static_cast<Eigen::Quaterniond>(transform.linear())};
}

template <internal::is_translation TranslationT>
inline std::tuple<Eigen::Translation3d, Eigen::Quaterniond>
    extract_translation_rotation(const TranslationT& translation) {
    return {static_cast<Eigen::Translation3d>(translation), Eigen::Quaterniond::Identity()};
}

template <internal::is_rotation RotationT>
inline std::tuple<Eigen::Translation3d, Eigen::Quaterniond>
    extract_translation_rotation(const RotationT& rotation) {
    return {Eigen::Translation3d::Identity(), static_cast<Eigen::Quaterniond>(rotation)};
}

} // namespace internal

/// 一个关节相对它父连杆的位姿，就是几个数。
///
/// JointCollection 本身不能靠一次普通拷贝交给另一条线程：它是一个由关节组成的 tuple，关节里
/// 放着 Eigen 对象，不是平凡可拷贝的。这个类型是：它过得了无锁缓冲、原子量、共享内存，
/// 也可以直接 memcpy。
struct JointPose {
    std::array<double, 3> translation; // x y z
    std::array<double, 4> rotation;    // 四元数，x y z w
};

template <internal::is_joint_collection JointCollectionT>
inline constexpr std::size_t joint_count = std::tuple_size_v<typename JointCollectionT::TupleT>;

/// 每个关节一个位姿，顺序同 JointCollectionT::for_each。
template <internal::is_joint_collection JointCollectionT>
using JointPoses = std::array<JointPose, joint_count<JointCollectionT>>;

/// 取每个关节当前的位姿。只有定长的算术：不分配、不加锁、不碰 ROS，
/// 所以在实时循环里调是安全的。
template <internal::is_joint_collection JointCollectionT>
[[nodiscard]] inline JointPoses<JointCollectionT>
    capture(const JointCollectionT& collection) noexcept {
    JointPoses<JointCollectionT> poses{};
    std::size_t index = 0;
    JointCollectionT::for_each([&]<typename From, typename To>() {
        const auto [translation, rotation] =
            internal::extract_translation_rotation(get_transform<From, To>(collection));
        poses[index++] = JointPose{
            .translation = {translation.x(), translation.y(), translation.z()},
            .rotation    = {rotation.x(), rotation.y(), rotation.z(), rotation.w()}};
    });
    return poses;
}

} // namespace fast_tf
