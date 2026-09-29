// 关键设备失效锁存（util/safety_latch.hpp）的单测。钉住的是"什么时候必须停、什么时候才许再动"：
//   - 从未上线的设备不触发（台架缺板不锁死整车）
//   - 上过线后离线 / 报故障立刻锁存，并记下第一个坏的是谁
//   - 设备自己恢复不解锁：接触不良的线反复掉线又上线，不许腿自己重新出力
//   - 复位必须是 DOWN → 离开 DOWN，且 DOWN 那一刻全部健康；拨到 DOWN 时还坏着，复位作废
//   - 一直停在 DOWN 不解锁（解锁发生在"离开"那一拍，控制器同拍从失能态起来）

#include <gtest/gtest.h>

#include "hardware/util/safety_latch.hpp"

namespace {

using hcs_core::hardware::util::DeviceHealth;
using hcs_core::hardware::util::SafetyLatch;
using hcs_msgs::Switch;

constexpr DeviceHealth kHealthy{.received = true, .online = true, .faulted = false};
constexpr DeviceHealth kNeverSeen{.received = false, .online = false, .faulted = false};
constexpr DeviceHealth kOffline{.received = true, .online = false, .faulted = false};
constexpr DeviceHealth kFaulted{.received = true, .online = true, .faulted = true};

} // namespace

TEST(SafetyLatch, NeverSeenDevicesDoNotTrip) {
    SafetyLatch latch;
    const DeviceHealth devices[] = {kNeverSeen, kHealthy, kNeverSeen};
    for (int i = 0; i < 10; ++i)
        EXPECT_FALSE(latch.update(devices, Switch::MIDDLE));
    EXPECT_EQ(latch.trip_count(), 0u);
}

TEST(SafetyLatch, OfflineAfterSeenTripsAndNamesTheDevice) {
    SafetyLatch latch;
    DeviceHealth devices[] = {kHealthy, kHealthy, kHealthy};
    EXPECT_FALSE(latch.update(devices, Switch::MIDDLE));

    devices[2] = kOffline;
    EXPECT_TRUE(latch.update(devices, Switch::MIDDLE));
    EXPECT_EQ(latch.reason(), SafetyLatch::Reason::kOffline);
    EXPECT_EQ(latch.tripped_device(), 2u);
    EXPECT_EQ(latch.trip_count(), 1u);
}

TEST(SafetyLatch, FaultTripsEvenWhileOnline) {
    SafetyLatch latch;
    const DeviceHealth devices[] = {kHealthy, kFaulted};
    EXPECT_TRUE(latch.update(devices, Switch::MIDDLE));
    EXPECT_EQ(latch.reason(), SafetyLatch::Reason::kFaulted);
    EXPECT_EQ(latch.tripped_device(), 1u);
}

TEST(SafetyLatch, RecoveryAloneDoesNotRelease) {
    SafetyLatch latch;
    DeviceHealth devices[] = {kHealthy, kOffline};
    ASSERT_TRUE(latch.update(devices, Switch::MIDDLE));

    // 线接回来了、设备自己又上线了：仍然锁着
    devices[1] = kHealthy;
    for (int i = 0; i < 100; ++i)
        EXPECT_TRUE(latch.update(devices, Switch::MIDDLE));
    // 第一次的原因不被后来的健康状态改写
    EXPECT_EQ(latch.reason(), SafetyLatch::Reason::kOffline);
    EXPECT_EQ(latch.trip_count(), 1u);
}

TEST(SafetyLatch, DownThenAwayReleases) {
    SafetyLatch latch;
    DeviceHealth devices[] = {kHealthy};
    latch.update(devices, Switch::MIDDLE);
    devices[0] = kOffline;
    ASSERT_TRUE(latch.update(devices, Switch::MIDDLE));
    devices[0] = kHealthy;

    // 停在 DOWN 不解锁
    for (int i = 0; i < 5; ++i)
        EXPECT_TRUE(latch.update(devices, Switch::DOWN));
    // 离开 DOWN 的那一拍解锁
    EXPECT_FALSE(latch.update(devices, Switch::MIDDLE));
    EXPECT_EQ(latch.reason(), SafetyLatch::Reason::kNone);
    EXPECT_FALSE(latch.update(devices, Switch::MIDDLE));
}

TEST(SafetyLatch, SwitchAwayWithoutDownDoesNotRelease) {
    SafetyLatch latch;
    DeviceHealth devices[] = {kFaulted};
    ASSERT_TRUE(latch.update(devices, Switch::MIDDLE));
    devices[0] = kHealthy;
    // MIDDLE ↔ UP 来回拨都不算复位
    EXPECT_TRUE(latch.update(devices, Switch::UP));
    EXPECT_TRUE(latch.update(devices, Switch::MIDDLE));
    EXPECT_TRUE(latch.update(devices, Switch::UP));
    // 遥控失联（UNKNOWN）也不算"离开 DOWN"的复位手势的前半
    EXPECT_TRUE(latch.update(devices, Switch::UNKNOWN));
}

TEST(SafetyLatch, ResetIsVoidWhileStillBroken) {
    SafetyLatch latch;
    DeviceHealth devices[] = {kHealthy};
    latch.update(devices, Switch::MIDDLE);
    devices[0] = kOffline;
    ASSERT_TRUE(latch.update(devices, Switch::MIDDLE));

    // 拨到 DOWN 时还坏着 → 拨回来也不解锁
    EXPECT_TRUE(latch.update(devices, Switch::DOWN));
    EXPECT_TRUE(latch.update(devices, Switch::MIDDLE));

    // 修好之后重新 DOWN → 离开，才解锁
    devices[0] = kHealthy;
    EXPECT_TRUE(latch.update(devices, Switch::DOWN));
    EXPECT_FALSE(latch.update(devices, Switch::MIDDLE));
}

TEST(SafetyLatch, BreakingAgainDuringDownKeepsLatched) {
    SafetyLatch latch;
    DeviceHealth devices[] = {kFaulted};
    ASSERT_TRUE(latch.update(devices, Switch::MIDDLE));

    devices[0] = kHealthy;
    EXPECT_TRUE(latch.update(devices, Switch::DOWN));
    devices[0] = kOffline;
    EXPECT_TRUE(latch.update(devices, Switch::DOWN));
    // 离开 DOWN 那一拍设备仍是坏的 → 不解锁
    EXPECT_TRUE(latch.update(devices, Switch::MIDDLE));
}

TEST(SafetyLatch, RetripsAfterRelease) {
    SafetyLatch latch;
    DeviceHealth devices[] = {kFaulted};
    ASSERT_TRUE(latch.update(devices, Switch::MIDDLE));
    devices[0] = kHealthy;
    latch.update(devices, Switch::DOWN);
    ASSERT_FALSE(latch.update(devices, Switch::MIDDLE));

    devices[0] = kOffline;
    EXPECT_TRUE(latch.update(devices, Switch::MIDDLE));
    EXPECT_EQ(latch.trip_count(), 2u);
    EXPECT_EQ(latch.reason(), SafetyLatch::Reason::kOffline);
}
