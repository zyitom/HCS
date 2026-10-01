#pragma once

#include <chrono>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>

#include <hcs_executor/component.hpp>
#include <hcs_msgs/switch.hpp>

#include "hardware/device/remote_control.hpp"
#include "hardware/device/serial_line.hpp"
#include "hardware/device/vt13.hpp"

namespace hcs_core::hardware::device {

/// VT13 图传遥控接收机：串口收字节（Vt13），解出的量经 RemoteControl 以 /remote/* 输出。
/// 输出名是 RemoteControl 固定的，name 只用于日志。
class Vt13Remote {
public:
    struct Config {
        /// 接收机串口速率（待上台架核对）。
        std::uint32_t baudrate = 115200;
    };

    Vt13Remote(hcs_executor::Component& status_component, const std::string&, const Config& config)
        : remote_(status_component)
        , baudrate_(config.baudrate) {
        remote_.register_vt13(&vt13_);
    }

    Vt13Remote(const Vt13Remote&) = delete;
    Vt13Remote& operator=(const Vt13Remote&) = delete;

    [[nodiscard]] SerialLine serial_line() const noexcept { return {.baudrate = baudrate_}; }

    /// libhcs IO 线程（事件域）。
    void store_status(std::span<const std::byte> data) noexcept { vt13_.store_status(data); }

    /// @param now 有效性计时的时间源，周期域传 tick.scheduled。
    void update_status(std::chrono::steady_clock::time_point now) {
        vt13_.update_status(now);
        remote_.update();
    }

    [[nodiscard]] bool received() const noexcept { return vt13_.valid(); }
    [[nodiscard]] bool online() const noexcept { return vt13_.valid(); }
    [[nodiscard]] hcs_msgs::Switch switch_left() const { return remote_.switch_left(); }

    [[nodiscard]] std::string describe() const {
        return std::format("VT13 remote at {} baud -> /remote/*", baudrate_);
    }

    /// 尽力域：输入缓冲溢出（IO 线程只计数，那里不许打日志），有新增才报。
    std::optional<std::string> take_problem() {
        const auto overflows = vt13_.overflow_count();
        if (overflows == reported_overflows_)
            return std::nullopt;
        const auto more = overflows - reported_overflows_;
        reported_overflows_ = overflows;
        return std::format(
            "VT13 input buffer overflowed {} more time(s), {} bytes dropped in total; "
            "update() is not draining the remote stream fast enough",
            more, vt13_.overflow_dropped_bytes());
    }

private:
    Vt13 vt13_;
    RemoteControl remote_;
    std::uint32_t baudrate_;
    std::uint64_t reported_overflows_ = 0;
};

} // namespace hcs_core::hardware::device
