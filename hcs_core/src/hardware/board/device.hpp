#pragma once

#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_executor/component.hpp>

#include "hardware/device/imu_sample.hpp"
#include "hardware/device/serial_line.hpp"
#include "hardware/util/board_transmitter.hpp"
#include "hardware/util/safety_latch.hpp"

namespace hcs_core::hardware::board {

// ============================================================================
// 端口眼里的设备。具体驱动（device::DmMotor 等）只认识自己的协议，不认识端口、板卡、
// 线程；devices.hpp 里的 Can<Driver> / Serial<Driver> / Imu<Driver> 把驱动接到这组接口上。
//
// 三个执行域：
//   事件域（libhcs IO 线程）  receive()          只把原始帧/字节交过线程，不分配、不加锁、不打日志
//   周期域（控制线程）        update_status()    取出新数据交给驱动解码，推进掉线计数，写健康输出
//                             append_command()   把这一拍的指令帧交给所在总线
//   尽力域（1 Hz 定时器）     take_problem()     有新问题就返回一行文字，由板组件打日志
//
// 所有设备都一样的事在这里只写一遍：掉线计数、<名字>/online、<名字>/health。
// ============================================================================

class Device {
public:
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;
    virtual ~Device() = default;

    /// 话题前缀，也是日志里的名字。
    [[nodiscard]] const std::string& name() const noexcept { return name_; }

    /// 是否收到过至少一帧有效数据。
    [[nodiscard]] bool received() const noexcept HCS_NONBLOCKING { return received_; }
    /// 掉线计数：最近 offline_timeout 拍内有过有效数据。
    [[nodiscard]] bool online() const noexcept HCS_NONBLOCKING { return offline_count_ > 0; }

    virtual void update_status(const hcs_sync::Tick& tick) HCS_NONBLOCKING = 0;

    /// 上线时打一行：驱动按什么参数理解这台设备。
    [[nodiscard]] virtual std::string describe() const = 0;
    /// 尽力域：自上次调用以来的新问题。
    virtual std::optional<std::string> take_problem() { return std::nullopt; }

protected:
    /// 注册 <name>/online 与 <name>/health（给安全锁存这类跨板的消费者读）。
    /// health 的初值（也是组件被隔离时复位到的值）是"上过线、现在不在线"：板组件一旦被隔离，
    /// 读它的锁存立刻看到失效，而不是"从未上线"那种会被放过的状态。
    /// @param offline_timeout 连续多少拍没有有效数据算掉线。按拍计而不是按时间：设备与
    ///        控制回路不同源，零星的空拍是常态，连续空这么多拍才不是。
    Device(hcs_executor::Component& status, std::string name, int offline_timeout)
        : name_(std::move(name))
        , offline_timeout_(offline_timeout) {
        if (offline_timeout_ <= 0)
            throw std::invalid_argument(
                std::format("{}: offline_timeout must be positive, got {}", name_, offline_timeout_));
        status.register_output(name_ + "/online", online_output_, false);
        status.register_output(
            name_ + "/health", health_output_,
            util::DeviceHealth{.received = true, .online = false, .faulted = false});
    }

    /// 周期域每拍一次，在驱动的 on_tick() 之前：推进掉线计数。
    /// @param fresh 这一拍有没有交给驱动有效的新数据
    /// @return 这一拍设备是否在线
    bool watch(bool fresh) noexcept HCS_NONBLOCKING {
        if (fresh) {
            received_ = true;
            offline_count_ = offline_timeout_;
        } else if (offline_count_ > 0)
            --offline_count_;
        return online();
    }

    /// 周期域每拍一次，驱动更新完之后：写 <name>/online 与 <name>/health。
    /// @param faulted 设备自报的、需要人处理的故障（驱动有 faulted() 才有）
    void publish_health(bool faulted) noexcept HCS_NONBLOCKING {
        *online_output_ = online();
        *health_output_ = {.received = received_, .online = online(), .faulted = faulted};
    }

private:
    std::string name_;
    int offline_timeout_;
    int offline_count_ = 0;
    bool received_ = false;
    hcs_executor::Component::OutputInterface<bool> online_output_;
    hcs_executor::Component::OutputInterface<util::DeviceHealth> health_output_;
};

class CanDevice : public Device {
public:
    /// 反馈帧的 CAN id。一路总线上每台设备各不相同（attach 时检查）。
    [[nodiscard]] virtual std::uint32_t feedback_id() const noexcept = 0;
    /// 指令帧的 CAN id。
    [[nodiscard]] virtual std::uint32_t command_id() const noexcept = 0;
    /// 几台合一帧（DJI）时本机写的槽位；独占一帧时为空。
    [[nodiscard]] virtual std::optional<std::size_t> command_slot() const noexcept = 0;
    /// 驱动要求的总线：速率与是否 CAN FD。与总线声明的设置不符，attach 时拒绝。
    [[nodiscard]] virtual std::uint32_t bitrate() const noexcept = 0;
    [[nodiscard]] virtual bool fd() const noexcept = 0;

    /// 事件域：总线已按 feedback_id() 查过表。
    virtual void receive(std::uint32_t can_id, std::span<const std::byte> data) noexcept
        HCS_NONBLOCKING = 0;
    /// 周期域：safe = 这一拍发失能帧。
    virtual void append_command(util::CommandFrames::Bus& bus, bool safe) HCS_NONBLOCKING = 0;

protected:
    using Device::Device;
};

class SerialDevice : public Device {
public:
    /// 设备要求的串口设置（速率、帧格式）：端口据此经 EP0 下发。
    [[nodiscard]] virtual device::SerialLine serial_line() const noexcept = 0;

    /// 事件域：一段收到的字节，帧边界不保证。
    virtual void receive(std::span<const std::byte> data) noexcept HCS_NONBLOCKING = 0;

protected:
    using Device::Device;
};

class ImuDevice : public Device {
public:
    /// 事件域：板载 IMU 的一个原始样本（加速度计或陀螺仪）。
    virtual void receive(const device::ImuSample& sample) noexcept HCS_NONBLOCKING = 0;

protected:
    using Device::Device;
};

} // namespace hcs_core::hardware::board
