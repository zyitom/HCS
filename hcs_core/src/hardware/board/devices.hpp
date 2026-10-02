#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <utility>

#include <hcs_base/channel/byte_tunnel.hpp>
#include <hcs_base/channel/event_queue.hpp>
#include <hcs_base/channel/snapshot.hpp>
#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>

#include "hardware/board/board.hpp"
#include "hardware/board/device.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16_remote.hpp"
#include "hardware/device/hipnuc.hpp"
#include "hardware/device/imu_sample.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/vt13_remote.hpp"

namespace hcs_core::hardware::board {

// ============================================================================
// 把驱动接到端口上。
//
// 驱动（device::*）只写协议：字节 ↔ 物理量。它不认识端口、板卡，也**不认识线程**——
// 驱动的每个函数都在控制线程（周期域）上被调用，可以当单线程代码写，里面不该出现
// atomic。线程边界只在这个文件里：
//
//   事件域（libhcs IO 线程）  包装层把原始帧 / 字节 / 样本放进无锁通道
//                               CAN：Snapshot，只留最新一帧
//                               串口：ByteTunnel，字节按序全留
//                               板载 IMU：EventQueue，样本按序全留
//   周期域（控制线程）        包装层取出来交给驱动 on_frame() / on_bytes() / on_sample()，
//                             推进掉线计数，写 <名字>/online 与 <名字>/health
//
//   DmMotor pitch{can2, "/gimbal/pitch", {.control_mode = ..., ...}};
//
// 写一个新驱动要满足的约定就是下面三个 concept 之一；满足不了时编译器会指出缺哪一条。
// ============================================================================

/// CAN 上的一种协议：一帧反馈进，一帧指令出。
///
/// 必须有：
///   Config                      配置（聚合），含 offline_timeout：连续多少拍没有反馈算掉线
///   D(status, command, name, config)
///                               状态输出注册在 status 上，指令输入注册在 command 上
///   kCanBitrate / kCanFd        对总线的要求；与总线声明的不符，登记时拒绝
///   feedback_id() / command_id()
///                               反馈帧与指令帧的 CAN id
///   on_frame(frame)             周期域：来了一帧新的反馈。解码，写输出。
///                               一拍内到了几帧只会看到最新那帧
///   append_command(bus, safe)   周期域：把这一拍的指令帧交给总线；safe = 发失能帧
///
/// 可以有（没有就用默认行为）：
///   accepts(frame) const        **事件域**上的过滤：返回 false 的帧不进通道，也不算"收到"。
///                               同 id 上有别的回复（LK）或别的电机（DM）时用。它跑在 IO 线程上，
///                               所以只许读构造之后不再变的配置
///   on_tick(online)             周期域：每拍一次，不管有没有新帧（退避计数、掉线后的处理）
///   faulted() const             设备自报的、需要人处理的故障，进 <名字>/health
///   command_slot() const        几台合一帧（DJI）时本机写的槽位
///   describe() const            上线时打的那一行
///   take_problem()              尽力域：自上次调用以来的新问题；也可以写成
///   take_problem(rejected)      带一个参数：accepts() 至今拒掉的帧数
template <class D>
concept CanDriver =
    requires { typename D::Config; }
    && std::constructible_from<
        D, hcs_executor::Component&, hcs_executor::Component&, const std::string&,
        const typename D::Config&>
    && requires(
        D driver, const D const_driver, const typename D::Config config, device::CanPacket8 frame,
        util::CommandFrames::Bus& bus, bool safe) {
           { D::kCanBitrate } -> std::convertible_to<std::uint32_t>;
           { D::kCanFd } -> std::convertible_to<bool>;
           { config.offline_timeout } -> std::convertible_to<int>;
           { const_driver.feedback_id() } -> std::convertible_to<std::uint32_t>;
           { const_driver.command_id() } -> std::convertible_to<std::uint32_t>;
           driver.on_frame(frame);
           driver.append_command(bus, safe);
       };

/// 串口上的一种协议：字节流进。
///
/// 必须有：
///   Config                      配置（聚合），含 offline_timeout
///   D(status, name, config)     状态输出注册在 status 上
///   serial_line() const         设备要求的串口设置（速率、帧格式）
///   on_bytes(bytes) -> bool     周期域：按到达顺序喂一段字节，帧边界不保证——半帧要自己留着。
///                               返回这一段里有没有解出至少一帧有效数据（掉线计数只认它）
///
/// 可以有：on_tick(online)、faulted()、describe()、take_problem()，含义同 CanDriver。
template <class D>
concept SerialDriver =
    requires { typename D::Config; }
    && std::constructible_from<
        D, hcs_executor::Component&, const std::string&, const typename D::Config&>
    && requires(
        D driver, const D const_driver, const typename D::Config config,
        std::span<const std::byte> bytes) {
           { config.offline_timeout } -> std::convertible_to<int>;
           { const_driver.serial_line() } -> std::same_as<device::SerialLine>;
           { driver.on_bytes(bytes) } -> std::convertible_to<bool>;
       };

/// 板载 IMU 上的一种解算：加速度计与陀螺仪的原始样本进。
///
/// 必须有：
///   Config                      配置（聚合），含 offline_timeout
///   D(status, name, config)     状态输出注册在 status 上
///   on_sample(sample) -> bool   周期域：按到达顺序喂一个样本（加速度计或陀螺仪）。
///                               返回这个样本算不算一次有效的反馈（掉线计数只认它）
///
/// 可以有：on_tick(online)、faulted()、describe()、take_problem()，含义同 CanDriver。
template <class D>
concept ImuDriver =
    requires { typename D::Config; }
    && std::constructible_from<
        D, hcs_executor::Component&, const std::string&, const typename D::Config&>
    && requires(D driver, const typename D::Config config, device::ImuSample sample) {
           { config.offline_timeout } -> std::convertible_to<int>;
           { driver.on_sample(sample) } -> std::convertible_to<bool>;
       };

namespace detail {

template <class Driver>
bool faulted_of(const Driver& driver) noexcept HCS_NONBLOCKING {
    if constexpr (requires { { driver.faulted() } -> std::convertible_to<bool>; })
        return driver.faulted();
    else
        return false;
}

} // namespace detail

/// CAN 总线上的一台设备。
template <CanDriver Driver>
class Can
    : public CanDevice
    , public Driver {
public:
    using Config = typename Driver::Config;

    Can(CanBus& bus, std::string name, const Config& config)
        : CanDevice(bus.board(), std::move(name), config.offline_timeout)
        , Driver(bus.board(), bus.board().command(), this->name(), config) {
        bus.attach(*this);
    }

    [[nodiscard]] std::uint32_t feedback_id() const noexcept override {
        return Driver::feedback_id();
    }
    [[nodiscard]] std::uint32_t command_id() const noexcept override {
        return Driver::command_id();
    }
    [[nodiscard]] std::optional<std::size_t> command_slot() const noexcept override {
        if constexpr (requires(const Driver& d) { d.command_slot(); })
            return Driver::command_slot();
        else
            return std::nullopt;
    }
    [[nodiscard]] std::uint32_t bitrate() const noexcept override { return Driver::kCanBitrate; }
    [[nodiscard]] bool fd() const noexcept override { return Driver::kCanFd; }

    /// 事件域：过滤，然后把这一帧放进通道。驱动的状态这里一个字节都不碰。
    void receive(std::uint32_t, std::span<const std::byte> data) noexcept
        HCS_NONBLOCKING override {
        if (data.size() != sizeof(Frame)) [[unlikely]]
            return;
        Frame frame;
        std::ranges::copy(data, frame.begin());

        if constexpr (requires(const Driver& d, device::CanPacket8 f) {
                          { d.accepts(f) } -> std::convertible_to<bool>;
                      }) {
            if (!Driver::accepts(device::CanPacket8{std::span<const std::byte, 8>{frame}}))
                [[unlikely]] {
                rejected_frames_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
        }

        // 这条线程上不许读时钟（nonblocking），CAN 帧自己也不带到达时刻：
        // 读侧只看 Reading::fresh，不看 age。
        inbox_.publish(frame, hcs_sync::Timestamp{});
    }

    /// 周期域：有新帧就交给驱动，然后是所有设备都一样的那几步。
    void update_status(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        const auto reading = inbox_.read(hcs_sync::Timestamp{});
        if (reading.fresh)
            Driver::on_frame(device::CanPacket8{std::span<const std::byte, 8>{reading.value}});

        [[maybe_unused]] const bool online = watch(reading.fresh);
        if constexpr (requires(Driver& d) { d.on_tick(online); })
            Driver::on_tick(online);
        publish_health(detail::faulted_of<Driver>(*this));
    }

    void append_command(util::CommandFrames::Bus& bus, bool safe) HCS_NONBLOCKING override {
        Driver::append_command(bus, safe);
    }

    /// accepts() 至今拒掉的帧数（尽力域读）。
    [[nodiscard]] std::uint32_t rejected_frames() const noexcept {
        return rejected_frames_.load(std::memory_order_relaxed);
    }

    [[nodiscard]] std::string describe() const override {
        if constexpr (requires(const Driver& d) { d.describe(); })
            return Driver::describe();
        else
            return std::format("feedback 0x{:x} command 0x{:x}", feedback_id(), command_id());
    }

    std::optional<std::string> take_problem() override {
        if constexpr (requires(Driver& d, std::uint32_t rejected) { d.take_problem(rejected); })
            return Driver::take_problem(rejected_frames());
        else if constexpr (requires(Driver& d) { d.take_problem(); })
            return Driver::take_problem();
        else
            return std::nullopt;
    }

private:
    /// 经典 CAN 的 8 字节。存成字节数组而不是 CanPacket8：通道里的格子要能值初始化。
    using Frame = std::array<std::byte, 8>;

    hcs_sync::Snapshot<Frame> inbox_;                ///< 事件域发布，周期域读：只留最新一帧
    std::atomic<std::uint32_t> rejected_frames_{0};  ///< 事件域写，尽力域读
};

/// 串口上的一台设备。
template <SerialDriver Driver>
class Serial
    : public SerialDevice
    , public Driver {
public:
    using Config = typename Driver::Config;

    Serial(SerialPort& port, std::string name, const Config& config)
        : SerialDevice(port.board(), std::move(name), config.offline_timeout)
        , Driver(port.board(), this->name(), config) {
        port.attach(*this);
    }

    [[nodiscard]] device::SerialLine serial_line() const noexcept override {
        return Driver::serial_line();
    }

    /// 事件域：字节原样搬过去，不解析。放不下的部分丢弃并计数（见 take_problem()）。
    void receive(std::span<const std::byte> data) noexcept HCS_NONBLOCKING override {
        tunnel_.write(data);
    }

    /// 周期域：把攒下的字节按序喂给驱动，然后是所有设备都一样的那几步。
    void update_status(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        bool fresh = false;
        std::array<std::byte, kChunkBytes> chunk;
        // 一拍最多搬一整条通道的量：对端写得再快，这一拍的工作量也有上界。
        for (std::size_t budget = kTunnelBytes; budget != 0;) {
            const auto count = tunnel_.read(std::span{chunk}.first(std::min(kChunkBytes, budget)));
            if (count == 0)
                break;
            fresh |= static_cast<bool>(
                Driver::on_bytes(std::span<const std::byte>{chunk}.first(count)));
            budget -= count;
        }

        [[maybe_unused]] const bool online = watch(fresh);
        if constexpr (requires(Driver& d) { d.on_tick(online); })
            Driver::on_tick(online);
        publish_health(detail::faulted_of<Driver>(*this));
    }

    [[nodiscard]] std::string describe() const override {
        if constexpr (requires(const Driver& d) { d.describe(); })
            return Driver::describe();
        else
            return std::format("serial device at {} baud", serial_line().baudrate);
    }

    /// 尽力域：驱动自己的问题优先；其次是通道溢出（事件域只计数，那里不许打日志）。
    std::optional<std::string> take_problem() override {
        if constexpr (requires(Driver& d) { d.take_problem(); })
            if (auto problem = Driver::take_problem())
                return problem;

        const auto overrun = tunnel_.overrun_bytes();
        if (overrun == reported_overrun_)
            return std::nullopt;
        const auto more = overrun - reported_overrun_;
        reported_overrun_ = overrun;
        return std::format(
            "serial input overran: {} more byte(s) dropped, {} in total; update() is not "
            "draining the stream fast enough",
            more, overrun);
    }

private:
    /// 921600 波特率满速约 92 字节/毫秒：2048 字节能扛住控制线程 20 ms 不来取。
    static constexpr std::size_t kTunnelBytes = 2048;
    static constexpr std::size_t kChunkBytes = 256;

    hcs_sync::ByteTunnel<kTunnelBytes> tunnel_; ///< 事件域写，周期域读：字节按序全留
    std::uint64_t reported_overrun_ = 0;        ///< 只在尽力域碰
};

/// 板载 IMU 上的一台设备。
template <ImuDriver Driver>
class Imu
    : public ImuDevice
    , public Driver {
public:
    using Config = typename Driver::Config;

    Imu(ImuPort& port, std::string name, const Config& config)
        : ImuDevice(port.board(), std::move(name), config.offline_timeout)
        , Driver(port.board(), this->name(), config) {
        port.attach(*this);
    }

    /// 事件域：样本原样放进队列，不解算。放不下的丢弃并计数（见 take_problem()）。
    void receive(const device::ImuSample& sample) noexcept HCS_NONBLOCKING override {
        (void)samples_.try_push(sample);
    }

    /// 周期域：把攒下的样本按序喂给驱动，然后是所有设备都一样的那几步。
    void update_status(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        bool fresh = false;
        device::ImuSample sample;
        // 一拍最多取一整条队列的量：对端写得再快，这一拍的工作量也有上界。
        for (std::size_t budget = kQueueSamples; budget != 0 && samples_.try_pop(sample); --budget)
            fresh |= static_cast<bool>(Driver::on_sample(sample));

        [[maybe_unused]] const bool online = watch(fresh);
        if constexpr (requires(Driver& d) { d.on_tick(online); })
            Driver::on_tick(online);
        publish_health(detail::faulted_of<Driver>(*this));
    }

    [[nodiscard]] std::string describe() const override {
        if constexpr (requires(const Driver& d) { d.describe(); })
            return Driver::describe();
        else
            return "onboard imu";
    }

    /// 尽力域：驱动自己的问题优先；其次是队列溢出（事件域只计数，那里不许打日志）。
    std::optional<std::string> take_problem() override {
        if constexpr (requires(Driver& d) { d.take_problem(); })
            if (auto problem = Driver::take_problem())
                return problem;

        const auto dropped = samples_.dropped();
        if (dropped == reported_dropped_)
            return std::nullopt;
        const auto more = dropped - reported_dropped_;
        reported_dropped_ = dropped;
        return std::format(
            "imu samples overran: {} more sample(s) dropped, {} in total; update() is not "
            "draining the queue fast enough",
            more, dropped);
    }

private:
    /// 陀螺仪 2 kHz 加上加速度计 1.6 kHz，每毫秒不到 4 个样本：256 个能扛住控制线程约 70 ms 不来取。
    static constexpr std::size_t kQueueSamples = 256;

    hcs_sync::EventQueue<device::ImuSample, kQueueSamples> samples_; ///< 事件域写，周期域读
    std::uint64_t reported_dropped_ = 0;                             ///< 只在尽力域碰
};

// 常用驱动的简写，整车文件里直接用。
using DmMotor = Can<device::DmMotor>;
using LkMotor = Can<device::LkMotor>;
using DjiMotor = Can<device::DjiMotor>;
using Hipnuc = Serial<device::Hipnuc>;
using Vt13Remote = Serial<device::Vt13Remote>;
using Dr16Remote = Serial<device::Dr16Remote>;
using Bmi088 = Imu<device::Bmi088>;
using Bmi088Ekf = Imu<device::Bmi088Ekf>;

} // namespace hcs_core::hardware::board
