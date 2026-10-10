// 自瞄契约的单测：布局钉死；控制侧拒收哪些命令；外推和开火窗的语义。

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>

#include <gtest/gtest.h>

#include "hcs_link/autoaim.hpp"
#include "hcs_link/channel.hpp"

namespace {

using hcs_link::autoaim::AimCommand;
using hcs_link::autoaim::GimbalState;

constexpr std::int64_t kNow = 1'000'000'000'000;
constexpr std::int64_t kMs = 1'000'000;

static_assert(hcs_link::Payload<GimbalState>);
static_assert(hcs_link::Payload<AimCommand>);

// 两个进程各自编译这两个结构体，下面这些偏移就是它们之间的约定。
static_assert(offsetof(GimbalState, tick_ns) == 0);
static_assert(offsetof(GimbalState, quaternion) == 16);
static_assert(offsetof(GimbalState, angular_velocity) == 48);
static_assert(offsetof(GimbalState, imu_online) == 72);
static_assert(offsetof(AimCommand, t_ref_ns) == 0);
static_assert(offsetof(AimCommand, azimuth) == 16);
static_assert(offsetof(AimCommand, fire_from_ns) == 64);
static_assert(offsetof(AimCommand, flags) == 80);

AimCommand steady_target() {
    return AimCommand{
        .t_ref_ns = kNow,
        .frame_id = 1,
        .azimuth = 0.5,
        .elevation = 0.1,
        .azimuth_rate = 2.0,
        .elevation_rate = -1.0,
        .azimuth_acceleration = 0.0,
        .elevation_acceleration = 0.0,
        .fire_from_ns = 0,
        .fire_until_ns = 0,
        .flags = AimCommand::kHasTarget,
        .reserved = 0,
    };
}

TEST(AimCommand, AcceptsASaneCommand) {
    EXPECT_TRUE(hcs_link::autoaim::is_valid(steady_target(), kNow));
}

TEST(AimCommand, RejectsNonFiniteValues) {
    auto command = steady_target();
    command.azimuth = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(hcs_link::autoaim::is_valid(command, kNow));

    command = steady_target();
    command.elevation_rate = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(hcs_link::autoaim::is_valid(command, kNow));
}

TEST(AimCommand, RejectsUnknownFlagsAndReservedBits) {
    auto command = steady_target();
    command.flags |= 1U << 7;
    EXPECT_FALSE(hcs_link::autoaim::is_valid(command, kNow));

    command = steady_target();
    command.reserved = 1;
    EXPECT_FALSE(hcs_link::autoaim::is_valid(command, kNow));
}

TEST(AimCommand, RejectsOutOfRangeMotion) {
    auto command = steady_target();
    command.elevation = 2.0;
    EXPECT_FALSE(hcs_link::autoaim::is_valid(command, kNow));

    command = steady_target();
    command.azimuth_rate = 60.0;
    EXPECT_FALSE(hcs_link::autoaim::is_valid(command, kNow));

    command = steady_target();
    command.elevation_acceleration = -2000.0;
    EXPECT_FALSE(hcs_link::autoaim::is_valid(command, kNow));
}

TEST(AimCommand, RejectsTimestampsOutsideTheWindow) {
    auto command = steady_target();
    command.t_ref_ns = kNow - 201 * kMs;
    EXPECT_FALSE(hcs_link::autoaim::is_valid(command, kNow));

    command.t_ref_ns = kNow + 51 * kMs;
    EXPECT_FALSE(hcs_link::autoaim::is_valid(command, kNow));

    command.t_ref_ns = kNow - 199 * kMs;
    EXPECT_TRUE(hcs_link::autoaim::is_valid(command, kNow));
}

TEST(AimCommand, RejectsBrokenFireWindows) {
    auto command = steady_target();
    command.flags |= AimCommand::kFire;
    command.fire_from_ns = kNow + 10 * kMs;
    command.fire_until_ns = kNow;
    EXPECT_FALSE(hcs_link::autoaim::is_valid(command, kNow));

    command.fire_from_ns = kNow;
    command.fire_until_ns = kNow + 300 * kMs;
    EXPECT_FALSE(hcs_link::autoaim::is_valid(command, kNow));

    command.fire_until_ns = kNow + 20 * kMs;
    EXPECT_TRUE(hcs_link::autoaim::is_valid(command, kNow));
}

TEST(Extrapolate, FollowsRateAndAcceleration) {
    auto command = steady_target();
    command.azimuth_acceleration = 10.0;

    const auto aim = hcs_link::autoaim::extrapolate(command, kNow + 20 * kMs, 60 * kMs);
    EXPECT_NEAR(aim.azimuth, 0.5 + 2.0 * 0.02 + 0.5 * 10.0 * 0.02 * 0.02, 1e-12);
    EXPECT_NEAR(aim.elevation, 0.1 - 1.0 * 0.02, 1e-12);
}

TEST(Extrapolate, ClampsTheHorizon) {
    const auto command = steady_target();

    const auto far = hcs_link::autoaim::extrapolate(command, kNow + 500 * kMs, 60 * kMs);
    EXPECT_NEAR(far.azimuth, 0.5 + 2.0 * 0.06, 1e-12);

    const auto before = hcs_link::autoaim::extrapolate(command, kNow - 5 * kMs, 60 * kMs);
    EXPECT_DOUBLE_EQ(before.azimuth, 0.5);
}

TEST(Extrapolate, KeepsElevationInsideTheHemisphere) {
    auto command = steady_target();
    command.elevation = 1.5;
    command.elevation_rate = 40.0;
    const auto aim = hcs_link::autoaim::extrapolate(command, kNow + 50 * kMs, 60 * kMs);
    EXPECT_DOUBLE_EQ(aim.elevation, std::numbers::pi / 2);
}

TEST(Direction, MatchesTheAxes) {
    using hcs_link::autoaim::Aim;
    const auto forward = hcs_link::autoaim::direction(Aim{.azimuth = 0.0, .elevation = 0.0});
    EXPECT_NEAR(forward[0], 1.0, 1e-12);

    const auto left = hcs_link::autoaim::direction(Aim{.azimuth = std::numbers::pi / 2, .elevation = 0.0});
    EXPECT_NEAR(left[1], 1.0, 1e-12);

    const auto up = hcs_link::autoaim::direction(Aim{.azimuth = 0.3, .elevation = std::numbers::pi / 2});
    EXPECT_NEAR(up[2], 1.0, 1e-12);
    EXPECT_NEAR(std::hypot(up[0], up[1]), 0.0, 1e-12);
}

TEST(Fire, OnlyInsideTheWindow) {
    auto command = steady_target();
    command.flags |= AimCommand::kFire;
    command.fire_from_ns = kNow;
    command.fire_until_ns = kNow + 10 * kMs;

    EXPECT_FALSE(hcs_link::autoaim::fire_allowed(command, kNow - 1));
    EXPECT_TRUE(hcs_link::autoaim::fire_allowed(command, kNow + 5 * kMs));
    EXPECT_FALSE(hcs_link::autoaim::fire_allowed(command, kNow + 11 * kMs));

    command.flags = AimCommand::kHasTarget;
    EXPECT_FALSE(hcs_link::autoaim::fire_allowed(command, kNow + 5 * kMs));
}

} // namespace
