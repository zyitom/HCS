// TickTimer 的单测。这个类旧版把"触发后静默"编码在 counter 的奇偶性上
// (1-2 无符号下溢进入永不到 0 的奇数环)——数学上成立、行为上完全不可读。
// 换成显式状态机之后,行为必须一字不差地钉住,防止下一个重构的人凭直觉"修"它。

#include <gtest/gtest.h>

#include <hcs_utility/tick_timer.hpp>

namespace {
using hcs_utility::TickTimer;
} // namespace

TEST(TickTimer, FiresAfterExactlyCooldownTicks) {
    TickTimer timer;
    timer.reset(3);

    EXPECT_FALSE(timer.tick());
    EXPECT_FALSE(timer.tick());
    EXPECT_TRUE(timer.tick()); // 第 cooldown 拍触发,不早不晚
}

TEST(TickTimer, ZeroCooldownFiresOnNextTick) {
    TickTimer timer;
    timer.reset(0);

    EXPECT_TRUE(timer.tick());
}

TEST(TickTimer, FiresOnlyOnceUntilReset) {
    TickTimer timer;
    timer.reset(2);

    EXPECT_FALSE(timer.tick());
    EXPECT_TRUE(timer.tick());

    // 触发后静默:报警刷屏没有意义,安全值只需要设置一次
    for (int i = 0; i < 1000; ++i)
        EXPECT_FALSE(timer.tick());
}

TEST(TickTimer, ResetRestartsTheCooldown) {
    TickTimer timer;
    timer.reset(1);
    EXPECT_TRUE(timer.tick());

    timer.reset(1);
    EXPECT_TRUE(timer.tick());
    EXPECT_FALSE(timer.tick());
}

TEST(TickTimer, ResetBeforeExpiryDefuses) {
    TickTimer timer;
    timer.reset(5);

    // 每拍喂狗:永远不触发
    for (int i = 0; i < 1000; ++i) {
        EXPECT_FALSE(timer.tick());
        timer.reset(5);
    }
}

TEST(TickTimer, NeverResetNeverFires) {
    // 默认构造 = 未武装。看门狗只负责"来过、然后断了"的数据流。
    TickTimer timer;
    for (int i = 0; i < 1000; ++i)
        EXPECT_FALSE(timer.tick());
}
