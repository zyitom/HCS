#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <span>
#include <stdexcept>
#include <type_traits>

#include <libhcs/board/hcs_can_port.hpp>
#include <libhcs/data/datas.hpp>

#include <hcs_base/channel/snapshot.hpp>
#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/doorbell.hpp>
#include <hcs_base/thread/doorbell_worker.hpp>

#include "hardware/device/can_packet.hpp"

namespace hcs_core::hardware::util {

// ============================================================================
// 一块板一拍的帧 → 本板的发送线程。
//
// 约束来自 libhcs：发送路径（start_transmit → 拿发送缓冲池的锁 → libusb_submit_transfer）
// 会阻塞、会进内核，只能在普通线程里做；而指令必须在拍尾紧跟着交到 USB 手里。
// 于是把一拍拆成两半：
//
//   周期域（板组件的 Command 伙伴 update()）
//     调本板各设备的 append_command()，把这一拍的帧装进一个定长批次，
//     publish() 进 Snapshot。不碰板卡对象。
//   拍尾门铃
//     executor 每拍最后 ring 一次（这是周期域唯一被允许的额外系统调用位置）。
//   发送线程（SharedTransmitter，几块板共用一个）
//     醒来逐板读 Snapshot，fresh 才对那块板 start_transmit() 一次把整批帧发出去。
//
// 隔离安全（hcs_executor/src/executor.hpp 的拍尾门铃不看 failed_）：
// 组件被隔离后门铃照常响，批次却不再更新。发送线程靠 Snapshot 的 age 识别
// "门铃在响、批次已陈旧"，把预存的全失能批次**发一次**，然后保持静默——
// 绝不重发最后一帧（最后一帧可能是让车冲出去的指令）。
// ============================================================================

/// 一帧：哪路总线、什么 id、什么内容。经典 CAN 2.0，8 字节。批次属于哪块板由批次本身决定。
struct BoardFrame {
    std::uint8_t port = 0; ///< libhcs::board::hcs::CanPort
    std::uint32_t can_id = 0;
    std::array<std::byte, 8> data{};
};

/// 一块板一拍的全部待发帧。平凡可拷贝，走 Snapshot 三缓冲。
struct TransmitBatch {
    static constexpr std::size_t kMaxFrames = 16;

    std::uint32_t sequence = 0; ///< 周期域每次 publish 递增，发送线程靠它判 fresh
    std::uint32_t frame_count = 0;
    std::array<BoardFrame, kMaxFrames> frames{};
};

static_assert(std::is_trivially_copyable_v<TransmitBatch>);

/// 装一帧。溢出是实现错误——每拍的帧数是构造期就确定的常数，放不下说明
/// 这块板的设备清单写错了，debug 下断言挡住，release 下丢弃并停在最后一格。
inline void push_frame(
    TransmitBatch& batch, libhcs::board::hcs::CanPort port, std::uint32_t can_id,
    const std::array<std::byte, 8>& data) noexcept {
    if (batch.frame_count >= TransmitBatch::kMaxFrames) [[unlikely]] {
        assert(false && "TransmitBatch overflow: kMaxFrames too small for this board");
        return;
    }
    auto& frame = batch.frames[batch.frame_count++];
    frame.port   = static_cast<std::uint8_t>(port);
    frame.can_id = can_id;
    frame.data   = data;
}

/// 同上，直接收设备生成的 CanPacket8。
inline void push_frame(
    TransmitBatch& batch, libhcs::board::hcs::CanPort port, std::uint32_t can_id,
    device::CanPacket8 packet) noexcept {
    const auto bytes = packet.as_bytes();
    std::array<std::byte, 8> data{};
    std::ranges::copy(bytes, data.begin());
    push_frame(batch, port, can_id, data);
}

/// 一拍的帧收集器：电机驱动经它把指令交给自己所在的那路总线，整车组件不必认识任何厂商。
///
/// 两种帧：
///   push()          独占帧，一台电机一帧（DM、LK），立刻进批次；
///   shared_frame()  共享帧，几台电机合一帧（DJI 电调一帧带四台，各写自己的槽位），
///                   同一路总线、同一 can_id 的合进同一帧，flush() 时按首次出现的
///                   顺序进批次。
/// 批次里的帧序因此是：独占帧按电机遍历顺序，共享帧排在最后——与改造前 pack_frames 一致。
/// 周期域使用：不分配、不抛。
class CommandFrames {
public:
    static constexpr std::size_t kMaxSharedFrames = 8;

    explicit CommandFrames(TransmitBatch& batch) noexcept
        : batch_(batch) {}

    /// 某路总线的视图，传给驱动的 append_command()。
    class Bus {
    public:
        void push(std::uint32_t can_id, device::CanPacket8 packet) noexcept {
            push_frame(owner_.batch_, port_, can_id, packet);
        }

        /// 这路总线上 can_id 的共享帧；第一次取时建一个全零帧。
        device::CanPacket8& shared_frame(std::uint32_t can_id) noexcept {
            return owner_.shared_frame(port_, can_id);
        }

    private:
        friend class CommandFrames;
        Bus(CommandFrames& owner, libhcs::board::hcs::CanPort port) noexcept
            : owner_(owner)
            , port_(port) {}

        CommandFrames& owner_;
        libhcs::board::hcs::CanPort port_;
    };

    Bus on(libhcs::board::hcs::CanPort port) noexcept { return Bus{*this, port}; }

    /// 共享帧进批次。每拍打包结束时调一次。
    void flush() noexcept {
        for (const auto& frame : std::span{shared_}.first(shared_count_))
            push_frame(batch_, frame.port, frame.can_id, frame.packet);
        shared_count_ = 0;
    }

private:
    struct Shared {
        libhcs::board::hcs::CanPort port;
        std::uint32_t can_id;
        device::CanPacket8 packet;
    };

    device::CanPacket8& shared_frame(
        libhcs::board::hcs::CanPort port, std::uint32_t can_id) noexcept {
        for (auto& frame : std::span{shared_}.first(shared_count_))
            if (frame.port == port && frame.can_id == can_id)
                return frame.packet;
        if (shared_count_ >= kMaxSharedFrames) [[unlikely]] {
            // 与 push_frame 同一口径：每拍的共享帧数是构造期常数，放不下是清单写错了。
            assert(false && "CommandFrames overflow: kMaxSharedFrames too small for this robot");
            return overflow_;
        }
        auto& frame = shared_[shared_count_++];
        frame = Shared{port, can_id, device::CanPacket8{std::uint64_t{0}}};
        return frame.packet;
    }

    TransmitBatch& batch_;
    std::array<Shared, kMaxSharedFrames> shared_{};
    std::size_t shared_count_ = 0;
    device::CanPacket8 overflow_{std::uint64_t{0}}; ///< 溢出时的丢弃位，不进批次
};

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

/// 发送线程眼里的板卡：把一整批帧发出去。
class BatchSink {
public:
    /// 发送线程调用。抛出的异常由 DoorbellWorker 计数（exceptions()），不终止进程。
    virtual void send(const TransmitBatch& batch) = 0;

protected:
    ~BatchSink() = default;
};

/// 几块板共用的发送线程。每块板一条 Lane：自己的批次、自己的陈旧判断与安全批次，
/// 共用线程每拍被叫醒一次，依次替各 Lane 发。所以一块板被隔离，只有它那条 Lane 变陈旧、
/// 补发一次它自己的全失能批次，别的板照常。
///
/// 为什么共用：发送路径要拿 libhcs 的锁、进内核（libusb_submit_transfer），只能在普通线程里；
/// 几块板的发送在同一个核上本来就是串行的，共用一个线程省掉每拍几次唤醒。
class SharedTransmitter {
public:
    static constexpr std::size_t kMaxLanes = 8;

    class Lane {
    public:
        /// 周期域：板组件的 Command 伙伴每拍调一次。wait-free，不分配不加锁。
        void publish(const TransmitBatch& batch, hcs_sync::Timestamp sampled_at) noexcept {
            snapshot_.publish(batch, sampled_at);
        }

    private:
        friend class SharedTransmitter;

        std::mutex mutex_; ///< 发送线程发送时持有；detach() 拿它等正在进行的那次发完
        BatchSink* board_ = nullptr;
        TransmitBatch safe_batch_{};
        bool safety_sent_ = false; ///< 只在发送线程碰
        hcs_sync::Snapshot<TransmitBatch> snapshot_;
    };

    /// @param thread_config 发送线程的亲和/优先级。
    /// @param stale_after 批次陈旧阈值。隔离后批次停在最后一拍，age 超过它才发安全批次。
    SharedTransmitter(hcs_utility::ThreadConfig thread_config, hcs_sync::Duration stale_after)
        : stale_after_(stale_after)
        , worker_(std::move(thread_config), [this] { drain(); }) {}

    ~SharedTransmitter() = default; ///< worker_ 最后声明、最先析构：先停线程再 join

    SharedTransmitter(const SharedTransmitter&) = delete;
    SharedTransmitter& operator=(const SharedTransmitter&) = delete;

    /// 板卡打开之后登记。@param safe_batch 预存的全失能批次（DM 0xFD、LK 0 电流、
    /// DJI 0 电流），由板组件以 safe = true 走一遍各驱动的 append_command() 拼好交进来。
    Lane& attach(BatchSink& board, const TransmitBatch& safe_batch) {
        const auto index = lane_count_.load(std::memory_order_relaxed);
        if (index >= kMaxLanes)
            throw std::length_error("SharedTransmitter: more boards than kMaxLanes");
        auto& lane = lanes_[index];
        lane.board_ = &board;
        lane.safe_batch_ = safe_batch;
        lane_count_.store(index + 1, std::memory_order_release);
        return lane;
    }

    /// 板卡关闭之前调用：此后发送线程不再碰这块板（正在进行的那次先发完）。
    void detach(Lane& lane) noexcept {
        const std::scoped_lock lock{lane.mutex_};
        lane.board_ = nullptr;
    }

    /// executor 拍尾振铃的门铃。
    [[nodiscard]] hcs_utility::Doorbell& doorbell() noexcept { return worker_.doorbell(); }

    /// 启动后到现在的累计统计（尽力域读）。
    [[nodiscard]] std::uint64_t wakeups() const noexcept { return worker_.wakeups(); }
    [[nodiscard]] std::uint64_t exceptions() const noexcept { return worker_.exceptions(); }

private:
    /// 发送线程体：每条 Lane 读自己的 Snapshot，按 plan_send 行动。
    void drain() {
        const auto now = hcs_sync::Clock::now();
        const auto count = lane_count_.load(std::memory_order_acquire);
        for (auto& lane : std::span{lanes_}.first(count)) {
            const std::scoped_lock lock{lane.mutex_};
            if (lane.board_ == nullptr)
                continue;
            const auto reading = lane.snapshot_.read(now);
            switch (plan_send(reading, lane.safety_sent_, stale_after_)) {
            case SenderAction::kSend:
                lane.safety_sent_ = false;
                lane.board_->send(reading.value);
                break;
            case SenderAction::kSafeSendOnce:
                lane.safety_sent_ = true;
                lane.board_->send(lane.safe_batch_);
                break;
            case SenderAction::kIdle: break;
            }
        }
    }

    hcs_sync::Duration stale_after_{};
    std::array<Lane, kMaxLanes> lanes_;
    std::atomic<std::size_t> lane_count_{0};
    hcs_utility::DoorbellWorker<std::function<void()>> worker_;
};

} // namespace hcs_core::hardware::util
