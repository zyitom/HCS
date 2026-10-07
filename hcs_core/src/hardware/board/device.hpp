#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include <libhcs/time/sample_time.hpp>

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
// 四个执行域：
//   事件域（libhcs IO 线程）  receive()          只把原始帧/字节交过线程，不分配、不加锁、不打日志
//   周期域（控制线程）        update_status()    取出新数据交给驱动解码，推进掉线计时，写健康输出
//                             append_command()   把这一拍要发的帧/字节交给所在的总线/串口
//   发送域（发送线程）        take_transmit()    取走串口待发的字节（CAN 帧走批次，不经设备）
//   尽力域（1 Hz 定时器）     take_problem()     有新问题就返回一行文字，由板组件打日志
//
// 收和发是对称的：收是 事件域 → 通道 → 周期域的驱动，发是 周期域的驱动 → 通道 → 发送域。
//
// 所有设备都一样的事在这里只写一遍：掉线计时、<名字>/online、<名字>/health。
// 带板上时刻的设备（CAN、板载 IMU）另有 <名字>/time，见 TimedDevice；串口没有板上戳，没有它。
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
    /// 最近 offline_timeout 这么长的时间内有过有效数据。
    [[nodiscard]] bool online() const noexcept HCS_NONBLOCKING {
        return received_ && silence_ < offline_timeout_;
    }

    virtual void update_status(const hcs_sync::Tick& tick) HCS_NONBLOCKING = 0;

    /// 上线时打一行：驱动按什么参数理解这台设备。
    [[nodiscard]] virtual std::string describe() const = 0;
    /// 尽力域：自上次调用以来的新问题。
    virtual std::optional<std::string> take_problem() { return std::nullopt; }

protected:
    /// 注册 <name>/online 与 <name>/health（给安全锁存这类跨板的消费者读）。
    /// health 的初值（也是组件被隔离时复位到的值）是"上过线、现在不在线"：板组件一旦被隔离，
    /// 读它的锁存立刻看到失效，而不是"从未上线"那种会被放过的状态。
    /// @param offline_timeout 连续多久没有有效数据算掉线。设备与控制回路不同源，零星的空拍
    ///        是常态，所以它要比设备的发送周期宽出几倍。是时长而不是拍数：同一个数在不同的
    ///        控制频率下、在跳拍的时候，都还是同一段时间。
    Device(hcs_executor::Component& status, std::string name, hcs_sync::Duration offline_timeout)
        : name_(std::move(name))
        , offline_timeout_(offline_timeout) {
        if (offline_timeout_ <= hcs_sync::Duration::zero())
            throw std::invalid_argument(std::format(
                "{}: offline_timeout must be positive, got {}", name_,
                std::chrono::duration_cast<std::chrono::milliseconds>(offline_timeout_)));
        status.register_output(name_ + "/online", online_output_, false);
        status.register_output(
            name_ + "/health", health_output_,
            util::DeviceHealth{.received = true, .online = false, .faulted = false});
    }

    /// 周期域每拍一次，在驱动的 on_tick() 之前：推进掉线计时。
    ///
    /// 没有数据的时间用 tick.dt 累加，不数调用次数：跳拍时 dt 是 N × period，被跳过的那几拍
    /// 照样算在"没有数据"里——控制线程来不及的时候，掉线不会因此判得更晚。
    /// @param fresh 这一拍有没有交给驱动有效的新数据
    /// @return 这一拍设备是否在线
    bool watch(bool fresh, const hcs_sync::Tick& tick) noexcept HCS_NONBLOCKING {
        if (fresh) {
            received_ = true;
            silence_ = hcs_sync::Duration::zero();
        } else if (silence_ < offline_timeout_) // 到了就不再加：掉线之后不必接着数
            silence_ += tick.dt;
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
    hcs_sync::Duration offline_timeout_;
    hcs_sync::Duration silence_{}; ///< 距上一次有效数据过了多久，封顶在 offline_timeout_
    bool received_ = false;
    hcs_executor::Component::OutputInterface<bool> online_output_;
    hcs_executor::Component::OutputInterface<util::DeviceHealth> health_output_;
};

/// 数据带板卡给的时刻的设备：CAN 反馈（开了 time_sync 是帧起始的板戳，否则是到达时刻）与
/// 板载 IMU 样本（另有板上 64 位时刻）。多一个 <name>/time 输出给要算数据年龄的控制器。
class TimedDevice : public Device {
public:
    /// 最近一次交给驱动的数据的样本时刻（libhcs/time/sample_time.hpp），也就是 <name>/time。
    /// SDK 在 IO 线程算好，这里原样转交。数据年龄 = tick.scheduled - time()->host（同一个
    /// steady_clock）；精度看 source：板戳约 1 us，到达时刻带 USB 抖动。为空 = 还没收到过。
    [[nodiscard]] const std::optional<libhcs::time::SampleTime>& time() const noexcept
        HCS_NONBLOCKING {
        return *time_output_;
    }

protected:
    TimedDevice(
        hcs_executor::Component& status, std::string name, hcs_sync::Duration offline_timeout)
        : Device(status, std::move(name), offline_timeout) {
        status.register_output(
            this->name() + "/time", time_output_, std::optional<libhcs::time::SampleTime>{});
    }

    /// 周期域：这一拍交给驱动的数据的样本时刻，直接写进 <name>/time。
    void stamp(const libhcs::time::SampleTime& time) noexcept HCS_NONBLOCKING {
        *time_output_ = time;
    }

private:
    hcs_executor::Component::OutputInterface<std::optional<libhcs::time::SampleTime>> time_output_;
};

class CanDevice : public TimedDevice {
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
    /// @param time 这一帧的时刻（libhcs/time/sample_time.hpp）：SDK 的 IO 线程在解码时
    ///        算好，board 戳在不在、时间轴锁没锁定都已经折算成 source。它在这里只是随帧
    ///        存下；周期域直接读，不再做任何换算。
    virtual void receive(
        std::uint32_t can_id, std::span<const std::byte> data,
        libhcs::time::SampleTime time) noexcept HCS_NONBLOCKING = 0;
    /// 周期域：safe = 这一拍发失能帧。
    virtual void append_command(util::CommandFrames::Bus& bus, bool safe) HCS_NONBLOCKING = 0;

protected:
    using TimedDevice::TimedDevice;
};

class SerialDevice : public Device {
public:
    /// 设备要求的串口设置（速率、帧格式）：端口据此经 EP0 下发。
    [[nodiscard]] virtual device::SerialLine serial_line() const noexcept = 0;
    /// 这台设备往串口上发不发字节。只收不发的（IMU 模块、遥控接收机）发送线程不必来问。
    [[nodiscard]] virtual bool transmits() const noexcept = 0;

    /// 事件域：一段收到的字节，帧边界不保证。
    virtual void receive(std::span<const std::byte> data) noexcept HCS_NONBLOCKING = 0;
    /// 周期域：把这一拍要发的字节排进发件箱；safe 的含义同 CanDevice::append_command()。
    virtual void append_command(bool safe) HCS_NONBLOCKING = 0;
    /// 发送域：取走待发的字节，最多 out.size() 个。@return 取走的字节数
    virtual std::size_t take_transmit(std::span<std::byte> out) noexcept = 0;

protected:
    using Device::Device;
};

class ImuDevice : public TimedDevice {
public:
    /// 设备要求的量程与速率：端口据此经 EP0 下发，板子按它初始化传感器。
    [[nodiscard]] virtual device::ImuSetup imu_setup() const noexcept = 0;

    /// 事件域：板载 IMU 的一个原始样本（加速度计或陀螺仪）。
    virtual void receive(const device::ImuSample& sample) noexcept HCS_NONBLOCKING = 0;

protected:
    using TimedDevice::TimedDevice;
};

} // namespace hcs_core::hardware::board
