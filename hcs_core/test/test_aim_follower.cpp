// AimFollower 的单测：新命令就跟、陈旧就停、坏命令不跟、心跳不跟、开火只在窗内、换会话清旧命令。
// 通道用进程内的 Writer / Reader——和跨进程走的是同一条 attach 路径。

#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "controller/auto_aim/aim_follower.hpp"

namespace {

using hcs_core::controller::auto_aim::AimFollower;
using Command = AimFollower::Command;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kT0 = 5'000'000'000;
constexpr std::uint32_t kStaleTicks = 5;

hcs_link::Writer<Command> make_writer() {
    auto writer = hcs_link::Writer<Command>::create({.capacity = 16, .lock_memory = false, .name = "test"});
    EXPECT_TRUE(writer.has_value());
    return std::move(*writer);
}

hcs_link::Reader<Command> open(const hcs_link::Writer<Command>& writer) {
    auto reader = writer.open_reader({.lock_memory = false});
    EXPECT_TRUE(reader.has_value());
    return std::move(*reader);
}

Command target(std::int64_t t_ref_ns, std::uint32_t flags = Command::kHasTarget) {
    return Command{
        .t_ref_ns = t_ref_ns,
        .frame_id = 0,
        .azimuth = 0.3,
        .elevation = 0.1,
        .azimuth_rate = 1.0,
        .elevation_rate = 0.0,
        .azimuth_acceleration = 0.0,
        .elevation_acceleration = 0.0,
        .fire_from_ns = t_ref_ns,
        .fire_until_ns = t_ref_ns + 10 * kMs,
        .flags = flags,
        .reserved = 0,
    };
}

struct Bench {
    hcs_link::Writer<Command> writer = make_writer();
    hcs_link::Reader<Command> reader = open(writer);
    AimFollower follower{AimFollower::Config{.stale_ticks = kStaleTicks}};
};

TEST(AimFollower, DoesNothingWithoutVision) {
    AimFollower follower{AimFollower::Config{}};
    EXPECT_FALSE(follower.update(nullptr, kT0).control);
}

TEST(AimFollower, FollowsAFreshCommandExtrapolatedToNow) {
    Bench bench;
    bench.writer.publish(target(kT0));

    const auto decision = bench.follower.update(&bench.reader, kT0 + 20 * kMs);
    ASSERT_TRUE(decision.control);
    const double azimuth = 0.3 + 1.0 * 0.02;
    EXPECT_NEAR(decision.direction[0], std::cos(0.1) * std::cos(azimuth), 1e-12);
    EXPECT_NEAR(decision.direction[1], std::cos(0.1) * std::sin(azimuth), 1e-12);
    EXPECT_NEAR(decision.direction[2], std::sin(0.1), 1e-12);
}

TEST(AimFollower, StopsAfterStaleTicksWithoutNewCommands) {
    Bench bench;
    bench.writer.publish(target(kT0));

    for (std::uint32_t tick = 0; tick < kStaleTicks; ++tick)
        EXPECT_TRUE(bench.follower.update(&bench.reader, kT0 + tick * kMs).control) << tick;
    EXPECT_FALSE(bench.follower.update(&bench.reader, kT0 + kStaleTicks * kMs).control);

    // 视觉又发了一条：立刻恢复
    bench.writer.publish(target(kT0 + kStaleTicks * kMs));
    EXPECT_TRUE(bench.follower.update(&bench.reader, kT0 + (kStaleTicks + 1) * kMs).control);
}

TEST(AimFollower, HeartbeatWithoutTargetDoesNotControl) {
    Bench bench;
    bench.writer.publish(target(kT0, 0));
    EXPECT_FALSE(bench.follower.update(&bench.reader, kT0).control);
    EXPECT_EQ(bench.follower.rejected(), 0U);
}

TEST(AimFollower, RejectsBrokenCommands) {
    Bench bench;
    auto broken = target(kT0);
    broken.azimuth = std::numeric_limits<double>::quiet_NaN();
    bench.writer.publish(broken);
    EXPECT_FALSE(bench.follower.update(&bench.reader, kT0).control);

    bench.writer.publish(target(kT0 - 300 * kMs)); // 太老
    EXPECT_FALSE(bench.follower.update(&bench.reader, kT0).control);
    EXPECT_EQ(bench.follower.rejected(), 2U);

    // 坏命令顶掉了之前的好命令，而不是让好命令继续生效
    bench.writer.publish(target(kT0));
    EXPECT_TRUE(bench.follower.update(&bench.reader, kT0).control);
    bench.writer.publish(broken);
    EXPECT_FALSE(bench.follower.update(&bench.reader, kT0 + kMs).control);
}

TEST(AimFollower, FiresOnlyInsideTheWindow) {
    Bench bench;
    bench.writer.publish(target(kT0, Command::kHasTarget | Command::kFire));

    EXPECT_TRUE(bench.follower.update(&bench.reader, kT0 + 5 * kMs).shoot);
    EXPECT_FALSE(bench.follower.update(&bench.reader, kT0 + 11 * kMs).shoot);
}

TEST(AimFollower, ANewSessionForgetsTheOldCommand) {
    Bench first;
    first.writer.publish(target(kT0));
    ASSERT_TRUE(first.follower.update(&first.reader, kT0).control);

    auto writer = make_writer();
    const auto reader = open(writer);
    EXPECT_FALSE(first.follower.update(&reader, kT0 + kMs).control);

    writer.publish(target(kT0 + kMs));
    EXPECT_TRUE(first.follower.update(&reader, kT0 + 2 * kMs).control);
}

} // namespace
