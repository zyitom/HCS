#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>

#include <hcs_executor/component.hpp>
#include <hcs_msgs/switch.hpp>

#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_outputs.hpp"
#include "hardware/device/serial_line.hpp"

namespace hcs_core::hardware::device {

/// DR16 接收机：串口字节交给 Dr16 解帧，在线就把解出的量原样放到 /remote/* 上，
/// 失联则是空安全态（全部归零，拨杆 UNKNOWN）。拨轮是 DR16 独有的，另有两个输出。
///
/// 输出名是 RemoteOutputs 固定的（/remote/*），name 只用于日志。
class Dr16Remote {
public:
    struct Config {
        /// 接收信号要不要在板上取反。DBUS 在线上是反相的；板上已经带了硬件反相器（c_board）
        /// 就不设，由板卡决定；没有的（mc02 的 DBUS 口靠寄存器取反）设 true。待上台架核对。
        std::optional<bool> rx_inverted = std::nullopt;
        /// 连续多少拍没有帧算失联。DR16 每 14 ms 发一帧，500 拍即 1 kHz 回路下的 500 ms。
        int offline_timeout = 500;
    };

    Dr16Remote(hcs_executor::Component& status_component, const std::string&, const Config& config)
        : outputs_(status_component)
        , rx_inverted_(config.rx_inverted) {
        status_component.register_output("/remote/rotary_knob", rotary_knob_, 0.0);
        status_component.register_output(
            "/remote/rotary_knob_switch", rotary_knob_switch_, hcs_msgs::Switch::UNKNOWN);
    }

    Dr16Remote(const Dr16Remote&) = delete;
    Dr16Remote& operator=(const Dr16Remote&) = delete;

    /// DBUS 是协议定死的：100 kbit/s，8 数据位，偶校验，1 停止位。
    [[nodiscard]] SerialLine serial_line() const noexcept {
        return {
            .baudrate = kBaudrate,
            .parity = SerialLine::Parity::kEven,
            .stop_bits = 1,
            .rx_inverted = rx_inverted_,
        };
    }

    /// 周期域：一段串口字节。返回有没有解出一帧。
    bool on_bytes(std::span<const std::byte> data) noexcept { return dr16_.on_bytes(data); }

    /// 周期域，每拍一次：这一拍的字节已经喂完；失联时先把 Dr16 归零，再决定这一拍发布什么。
    void on_tick(bool online) noexcept {
        dr16_.end_of_tick();
        dr16_.set_online(online);
        outputs_.publish(state());
        *rotary_knob_ = dr16_.rotary_knob();
        *rotary_knob_switch_ = dr16_.rotary_knob_switch();
        rejected_frames_.store(dr16_.rejected_frames(), std::memory_order_relaxed);
    }

    [[nodiscard]] hcs_msgs::Switch switch_left() const noexcept { return outputs_.switch_left(); }

    [[nodiscard]] std::string describe() const {
        return std::format("DR16 remote (DBUS, {} baud 8E1) -> /remote/*", kBaudrate);
    }

    /// 尽力域：流没对齐（凑满一帧却不像一帧）的次数每涨一次报一次。
    std::optional<std::string> take_problem() {
        const auto rejected = rejected_frames_.load(std::memory_order_relaxed);
        if (rejected == reported_rejected_frames_)
            return std::nullopt;
        reported_rejected_frames_ = rejected;
        return std::format(
            "{} DBUS frames did not look like DR16 frames so far; the stream is misaligned or the "
            "line settings (inversion, parity) are wrong",
            rejected);
    }

private:
    static constexpr std::uint32_t kBaudrate = 100'000;

    [[nodiscard]] RemoteOutputs::State state() const noexcept {
        RemoteOutputs::State state; // 空安全态
        if (!dr16_.valid())
            return state;

        state.joystick_right = dr16_.joystick_right();
        state.joystick_left = dr16_.joystick_left();
        state.switch_right = dr16_.switch_right();
        state.switch_left = dr16_.switch_left();
        state.mouse_velocity = dr16_.mouse_velocity();
        state.mouse_wheel = dr16_.mouse_wheel();
        state.mouse = dr16_.mouse();
        state.keyboard = dr16_.keyboard();
        return state;
    }

    Dr16 dr16_;
    RemoteOutputs outputs_;
    hcs_executor::Component::OutputInterface<double> rotary_knob_;
    hcs_executor::Component::OutputInterface<hcs_msgs::Switch> rotary_knob_switch_;
    std::optional<bool> rx_inverted_;

    // take_problem() 在尽力域（另一条线程）上被调用，而 Dr16 的计数只归控制线程：
    // 每拍抄一份到这里。驱动里别处都不需要原子量，这是唯一跨域的一个数。
    std::atomic<std::uint32_t> rejected_frames_{0};
    std::uint32_t reported_rejected_frames_ = 0; ///< 只在尽力域碰
};

} // namespace hcs_core::hardware::device
