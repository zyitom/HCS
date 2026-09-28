#include <chrono>
#include <cstdint>

#include <gtest/gtest.h>

#include <libhcs/board/hcs_can_port.hpp>

#include <hcs_sync/snapshot.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/util/board_transmitter.hpp"

namespace {

using hcs_core::hardware::device::CanPacket8;
using hcs_core::hardware::util::push_frame;
using hcs_core::hardware::util::plan_send;
using hcs_core::hardware::util::SenderAction;
using hcs_core::hardware::util::TransmitBatch;

using Reading = hcs_sync::Snapshot<TransmitBatch>::Reading;
using namespace std::chrono_literals;

TEST(board_transmitter, batch_packing) {
    TransmitBatch batch;
    batch.sequence = 7;

    auto packet =
        CanPacket8{CanPacket8::Quarter{0x1111}, CanPacket8::Quarter{0x2222},
                   CanPacket8::PaddingQuarter{}, CanPacket8::Quarter{0x4444}};
    push_frame(batch, 1, libhcs::board::hcs::CanPort::kCan2, 0x201, packet);

    ASSERT_EQ(batch.frame_count, 1u);
    const auto& frame = batch.frames[0];
    EXPECT_EQ(frame.board, 1);
    EXPECT_EQ(frame.port, static_cast<std::uint8_t>(libhcs::board::hcs::CanPort::kCan2));
    EXPECT_EQ(frame.can_id, 0x201u);

    const auto bytes = packet.as_bytes();
    EXPECT_TRUE(std::equal(bytes.begin(), bytes.end(), frame.data.begin()));
}

TEST(board_transmitter, batch_capacity_matches_robot) {
    // 平衡步兵每拍 9 帧（云台 4、底盘 4、aux 1），容量必须装得下且不翻倍浪费。
    TransmitBatch batch;
    for (std::uint32_t index = 0; index < 9; ++index)
        push_frame(
            batch, 0, libhcs::board::hcs::CanPort::kCan1, 0x100 + index, CanPacket8{});
    EXPECT_EQ(batch.frame_count, 9u);
    EXPECT_GE(TransmitBatch::kMaxFrames, batch.frame_count);
}

TEST(board_transmitter, plan_send_decisions) {
    // 从未有过批次：什么都不做。
    Reading invalid{};
    EXPECT_EQ(plan_send(invalid, false, 10ms), SenderAction::kIdle);

    // 新批次：发送，并清掉安全批次已发的记忆。
    Reading fresh{};
    fresh.valid = true;
    fresh.fresh = true;
    EXPECT_EQ(plan_send(fresh, true, 10ms), SenderAction::kSend);

    // 门铃在响但批次还新（刚读过、没有更新的）：等。
    Reading recent{};
    recent.valid = true;
    recent.fresh = false;
    recent.age = 5ms;
    EXPECT_EQ(plan_send(recent, false, 10ms), SenderAction::kIdle);

    // 批次陈旧（组件被隔离）：发一次全失能。
    Reading stale{};
    stale.valid = true;
    stale.fresh = false;
    stale.age = 15ms;
    EXPECT_EQ(plan_send(stale, false, 10ms), SenderAction::kSafeSendOnce);

    // 安全批次发过之后保持静默，绝不重发。
    EXPECT_EQ(plan_send(stale, true, 10ms), SenderAction::kIdle);
}

TEST(board_transmitter, snapshot_fresh_semantics) {
    // 发送线程是唯一读者：publish 后第一次 read fresh，之后不再 fresh。
    hcs_sync::Snapshot<TransmitBatch> snapshot;
    const auto now = hcs_sync::Clock::now();

    const auto first = snapshot.read(now);
    EXPECT_FALSE(first.valid);

    TransmitBatch batch;
    batch.sequence = 1;
    snapshot.publish(batch, now);

    auto reading = snapshot.read(now + 1ms);
    ASSERT_TRUE(reading.valid);
    EXPECT_TRUE(reading.fresh);
    EXPECT_EQ(reading.value.sequence, 1u);
    EXPECT_GT(reading.age, 0ms); // age = reference - sampled_at

    reading = snapshot.read(now + 2ms);
    EXPECT_TRUE(reading.valid);
    EXPECT_FALSE(reading.fresh);
}

} // namespace
