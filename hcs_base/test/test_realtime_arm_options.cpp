// RealtimeArmOptions::parse 的单测。只测解析，不做真正的武装（mlock / FIFO 要权限）。
//
// 重点是 malloc_arena_max：它是给上板 A/B 实测留的口，默认值必须保持旧行为（1），
// 0 必须表示"不设"而不是"设成 0"，写错单位时必须当场拒绝而不是静默生效。

#include <stdexcept>

#include <gtest/gtest.h>

#include <hcs_base/thread/realtime_arm.hpp>

using hcs_utility::RealtimeArmOptions;

TEST(RealtimeArmOptions, DefaultKeepsSingleArena) {
    const auto options = RealtimeArmOptions::parse("");
    EXPECT_TRUE(options.tune_malloc);
    EXPECT_EQ(options.malloc_arena_max, 1);
}

TEST(RealtimeArmOptions, ArenaMaxZeroMeansLeaveGlibcDefault) {
    const auto options = RealtimeArmOptions::parse("malloc_arena_max=0");
    EXPECT_EQ(options.malloc_arena_max, 0);
}

TEST(RealtimeArmOptions, ArenaMaxIsParsedAlongsideOtherKeys) {
    const auto options =
        RealtimeArmOptions::parse("mlock=on;malloc=on;malloc_arena_max=4;timer_slack=1us");
    EXPECT_TRUE(options.lock_memory);
    EXPECT_EQ(options.malloc_arena_max, 4);
}

TEST(RealtimeArmOptions, ArenaMaxRejectsBadValues) {
    EXPECT_THROW(RealtimeArmOptions::parse("malloc_arena_max=-1"), std::invalid_argument);
    EXPECT_THROW(RealtimeArmOptions::parse("malloc_arena_max=abc"), std::invalid_argument);
    EXPECT_THROW(RealtimeArmOptions::parse("malloc_arena_max=100000"), std::invalid_argument);
    EXPECT_THROW(
        RealtimeArmOptions::parse("malloc_arena_max=1;malloc_arena_max=2"),
        std::invalid_argument);
}
