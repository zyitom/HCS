// fast_tf::capture：把关节集合抓成定长的位姿数组（跨线程发布 TF 用的那一份）。
//   - 顺序与 Tf::for_each 一致
//   - 平移 / 四元数 / 轴角 / 完整变换四种关节各自落到正确的字段
//   - 结果平凡可拷贝，能过 Snapshot

#include <cmath>
#include <numbers>
#include <string>
#include <type_traits>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <gtest/gtest.h>

#include <fast_tf/fast_tf.hpp>
#include <hcs_description/tf_description.hpp>

namespace {

using namespace hcs_description;

static_assert(fast_tf::joint_count<Tf> == 5);
static_assert(std::is_trivially_copyable_v<fast_tf::JointPoses<Tf>>);

/// for_each 的顺序：GimbalCenter、Yaw、Pitch、OdomImu、Camera。
enum Index : std::size_t { kGimbalCenter, kYaw, kPitch, kOdomImu, kCamera };

void expect_rotation(const fast_tf::JointPose& pose, const Eigen::Quaterniond& expected) {
    const Eigen::Quaterniond actual{
        pose.rotation[3], pose.rotation[0], pose.rotation[1], pose.rotation[2]};
    // q 与 -q 是同一个旋转，比点积的绝对值。
    EXPECT_NEAR(std::abs(actual.dot(expected)), 1.0, 1e-12);
}

void expect_translation(const fast_tf::JointPose& pose, const Eigen::Vector3d& expected) {
    EXPECT_DOUBLE_EQ(pose.translation[0], expected.x());
    EXPECT_DOUBLE_EQ(pose.translation[1], expected.y());
    EXPECT_DOUBLE_EQ(pose.translation[2], expected.z());
}

} // namespace

TEST(TfCapture, OrderMatchesForEach) {
    std::vector<std::string> children;
    Tf::for_each([&]<typename From, typename To>() { children.emplace_back(To::name); });

    const std::vector<std::string> expected{
        "gimbal_center_link", "yaw_link", "pitch_link", "odom_imu", "camera_link"};
    EXPECT_EQ(children, expected);
}

TEST(TfCapture, EveryJointKindLandsInTheRightFields) {
    Tf tf;
    const Eigen::Quaterniond imu =
        Eigen::Quaterniond{Eigen::AngleAxisd{0.3, Eigen::Vector3d{1.0, 2.0, 3.0}.normalized()}};
    Eigen::Isometry3d camera = Eigen::Isometry3d::Identity();
    camera.translate(Eigen::Vector3d{0.1, -0.2, 0.3});
    camera.rotate(Eigen::AngleAxisd{-0.7, Eigen::Vector3d::UnitX()});

    tf.set_transform<BaseLink, GimbalCenterLink>(Eigen::Translation3d{1.0, 2.0, 3.0});
    tf.set_state<GimbalCenterLink, YawLink>(std::numbers::pi / 2);
    tf.set_state<YawLink, PitchLink>(-0.25);
    tf.set_transform<PitchLink, OdomImu>(imu);
    tf.set_transform<PitchLink, CameraLink>(camera);

    const auto poses = fast_tf::capture(tf);

    // 纯平移：旋转是单位元。
    expect_translation(poses[kGimbalCenter], {1.0, 2.0, 3.0});
    expect_rotation(poses[kGimbalCenter], Eigen::Quaterniond::Identity());

    // 轴角（有状态关节）：平移是零。
    expect_translation(poses[kYaw], Eigen::Vector3d::Zero());
    expect_rotation(
        poses[kYaw],
        Eigen::Quaterniond{Eigen::AngleAxisd{std::numbers::pi / 2, Eigen::Vector3d::UnitZ()}});
    expect_rotation(
        poses[kPitch], Eigen::Quaterniond{Eigen::AngleAxisd{-0.25, Eigen::Vector3d::UnitY()}});

    // 四元数。
    expect_translation(poses[kOdomImu], Eigen::Vector3d::Zero());
    expect_rotation(poses[kOdomImu], imu);

    // 完整变换：平移与旋转都有。
    expect_translation(poses[kCamera], {0.1, -0.2, 0.3});
    expect_rotation(
        poses[kCamera], Eigen::Quaterniond{Eigen::AngleAxisd{-0.7, Eigen::Vector3d::UnitX()}});
}
