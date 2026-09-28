#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <type_traits>

#include <libhcs/board/hcs_can_port.hpp>
#include <libhcs/board/hpm5321.hpp>
#include <libhcs/data/datas.hpp>

#include <hcs_sync/snapshot.hpp>
#include <hcs_sync/tick.hpp>
#include <hcs_utility/doorbell.hpp>
#include <hcs_utility/doorbell_worker.hpp>

#include "hardware/device/can_packet.hpp"

namespace hcs_core::hardware::util {

// ============================================================================
// 一拍的帧 → 发送线程。
//
// 范例是 hcs_link_probe.cpp，约束来自 libhcs：发送路径（start_transmit → libusb
// submit）里有锁，只能在普通线程里做；而指令必须在拍尾紧跟着交到 USB 手里。
// 于是把一拍拆成两半：
//
//   周期域（Command 伙伴的 update()）
//     调各设备的 generate_*()，把这一拍所有板的帧装进一个定长批次，
//     publish() 进 Snapshot。不碰板卡对象。
//   拍尾门铃
//     executor 每拍最后 ring 一次（这是周期域唯一被允许的额外系统调用位置）。
//   发送线程（DoorbellWorker，三板共用一个）
//     醒来读 Snapshot，fresh 才对每块板依次 start_transmit()。
//     不同板的发送线程在同一个核上本来就是串行的，第二块板要多等约 5 µs，
//     共用一个线程反而省一次唤醒。
//
// 隔离安全（hcs_executor/src/executor.hpp 的拍尾门铃不看 failed_）：
// 组件被隔离后门铃照常响，批次却不再更新。发送线程靠 Snapshot 的 age 识别
// "门铃在响、批次已陈旧"，把预存的全失能批次**发一次**，然后保持静默——
// 绝不重发最后一帧（最后一帧可能是让车冲出去的指令）。
// ============================================================================

/// 一帧：哪块板（板表下标）、哪路总线、什么 id、什么内容。经典 CAN 2.0，8 字节。
struct BoardFrame {
    std::uint8_t board = 0; ///< 板在发送线程板表里的下标
    std::uint8_t port = 0;  ///< libhcs::board::hcs::CanPort
    std::uint32_t can_id = 0;
    std::array<std::byte, 8> data{};
};

/// 一拍的全部待发帧。平凡可拷贝，走 Snapshot 三缓冲。
struct TransmitBatch {
    static constexpr std::size_t kMaxFrames = 16;

    std::uint32_t sequence = 0; ///< 周期域每次 publish 递增，发送线程靠它判 fresh
    std::uint32_t frame_count = 0;
    std::array<BoardFrame, kMaxFrames> frames{};
};

static_assert(std::is_trivially_copyable_v<TransmitBatch>);

/// 装一帧。溢出是实现错误——每拍的帧数是构造期就确定的常数，放不下说明
/// 板卡/设备清单写错了，debug 下断言挡住，release 下丢弃并停在最后一格。
inline void push_frame(
    TransmitBatch& batch, std::uint8_t board, libhcs::board::hcs::CanPort port,
    std::uint32_t can_id, const std::array<std::byte, 8>& data) noexcept {
    if (batch.frame_count >= TransmitBatch::kMaxFrames) [[unlikely]] {
        assert(false && "TransmitBatch overflow: kMaxFrames too small for this robot");
        return;
    }
    auto& frame = batch.frames[batch.frame_count++];
    frame.board  = board;
    frame.port   = static_cast<std::uint8_t>(port);
    frame.can_id = can_id;
    frame.data   = data;
}

/// 同上，直接收设备生成的 CanPacket8。
inline void push_frame(
    TransmitBatch& batch, std::uint8_t board, libhcs::board::hcs::CanPort port,
    std::uint32_t can_id, device::CanPacket8 packet) noexcept {
    const auto bytes = packet.as_bytes();
    std::array<std::byte, 8> data{};
    std::ranges::copy(bytes, data.begin());
    push_frame(batch, board, port, can_id, data);
}

/// 发送线程对一次唤醒该做什么（纯函数，gtest 直接覆盖）。
enum class SenderAction { kIdle, kSend, kSafeSendOnce };

inline SenderAction plan_send(
    const hcs_sync::Snapshot<TransmitBatch>::Reading& reading, bool safety_already_sent,
    hcs_sync::Duration stale_after) noexcept {
    if (!reading.valid)
        return SenderAction::kIdle; ///< 还从未有过批次（首拍之前被叫醒）
    if (reading.fresh)
        return SenderAction::kSend;
    if (!safety_already_sent && reading.age > stale_after)
        return SenderAction::kSafeSendOnce; ///< 门铃在响、批次陈旧：发一次全失能
    return SenderAction::kIdle;
}

class BoardTransmitter {
public:
    using Reading = hcs_sync::Snapshot<TransmitBatch>::Reading;

    /// @param boards 板表，下标即 BoardFrame::board。空指针 = 该板 enabled:false，
    ///               发送时跳过。板表在构造期就绑定（组件先建设备和板卡，最后建本类）。
    /// @param thread_config 发送线程的亲和/优先级（三板共用一个线程）。
    /// @param stale_after 批次陈旧阈值。隔离后批次停在最后一拍，age 超过它才发安全批次。
    BoardTransmitter(
        std::array<libhcs::board::Hpm5321*, 3> boards, hcs_utility::ThreadConfig thread_config,
        hcs_sync::Duration stale_after)
        : boards_(boards)
        , stale_after_(stale_after)
        , worker_(std::move(thread_config), [this] { drain(); }) {}

    ~BoardTransmitter() = default; ///< worker_ 的析构先停线程再 join

    BoardTransmitter(const BoardTransmitter&) = delete;
    BoardTransmitter& operator=(const BoardTransmitter&) = delete;

    /// executor 拍尾振铃的门铃。
    [[nodiscard]] hcs_utility::Doorbell& doorbell() noexcept { return worker_.doorbell(); }

    /// 预存的全失能批次（DM 0xFD、LK 失能、DJI 0 电流），由组件在构造期用设备
    /// 自己的 generate_disable_command() 拼好交进来。
    void set_safe_batch(const TransmitBatch& batch) noexcept { safe_batch_ = batch; }

    /// 周期域：Command 伙伴每拍调一次。wait-free，不分配不加锁。
    void publish(const TransmitBatch& batch, hcs_sync::Timestamp sampled_at) noexcept {
        snapshot_.publish(batch, sampled_at);
    }

    /// 启动后到现在的累计统计（尽力域读）。
    [[nodiscard]] std::uint64_t wakeups() const noexcept { return worker_.wakeups(); }
    [[nodiscard]] std::uint64_t exceptions() const noexcept { return worker_.exceptions(); }

private:
    /// 发送线程体：读 Snapshot，按 plan_send 行动。
    void drain() noexcept {
        const auto reading = snapshot_.read(hcs_sync::Clock::now());
        switch (plan_send(reading, safety_sent_, stale_after_)) {
        case SenderAction::kSend:
            safety_sent_ = false;
            send(reading.value);
            break;
        case SenderAction::kSafeSendOnce:
            safety_sent_ = true;
            send(safe_batch_);
            break;
        case SenderAction::kIdle: break;
        }
    }

    /// 对每块启用的板各 start_transmit() 一次，把属于它的帧全部塞进同一个发送缓冲。
    void send(const TransmitBatch& batch) noexcept {
        for (std::size_t board = 0; board < boards_.size(); ++board) {
            auto* board_ptr = boards_[board];
            if (board_ptr == nullptr)
                continue;
            auto builder = board_ptr->start_transmit();
            for (std::uint32_t index = 0; index < batch.frame_count; ++index) {
                const auto& frame = batch.frames[index];
                if (frame.board != board)
                    continue;
                builder.can_transmit(
                    static_cast<libhcs::board::hcs::CanPort>(frame.port),
                    libhcs::data::CanDataView{
                        .can_id = frame.can_id,
                        .can_data = frame.data,
                    });
            }
        }
    }

    std::array<libhcs::board::Hpm5321*, 3> boards_{};
    hcs_sync::Duration stale_after_{};
    hcs_sync::Snapshot<TransmitBatch> snapshot_;
    TransmitBatch safe_batch_{};
    bool safety_sent_ = false; ///< 只在发送线程碰
    hcs_utility::DoorbellWorker<std::function<void()>> worker_;
};

} // namespace hcs_core::hardware::util
