#pragma once

#include <cstdint>
#include <optional>

namespace hcs_core::hardware::device {

/// 一个串口设备要它的端口设成什么样。这是设备协议的一部分（对 CH040 这样的模块，则是它 flash
/// 里存的东西），所以由驱动说、由端口去设。每一项都写明：端口"固件启动时是什么样就什么样"
/// 不是一个可选项——板卡的 CDC 接口允许主机操作系统在会话之外改写串口参数（实测 ModemManager
/// 的探测把波特率设成过 9596），而重连时只有配置过的端口才会被恢复。
struct SerialLine {
    enum class Parity : std::uint8_t { kNone, kEven, kOdd };

    std::uint32_t baudrate;
    Parity parity = Parity::kNone;
    std::uint8_t stop_bits = 1;
    /// 接收信号取反，仅在板子有这一位的端口上有效（mc02 的 DBUS）。不设：保持端口的默认。
    std::optional<bool> rx_inverted = std::nullopt;
};

} // namespace hcs_core::hardware::device
