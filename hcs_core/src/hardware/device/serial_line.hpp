#pragma once

#include <cstdint>
#include <optional>

namespace hcs_core::hardware::device {

/// What a serial device needs its port set to: part of the device's protocol (or, for a
/// module like the CH040, of what its flash holds), so the driver states it and the port
/// applies it. Every field is explicit: a port left "as the firmware brought it up" is not an
/// option, because the board's CDC interface lets the host OS rewrite the UART outside the
/// session (ModemManager's probe was measured setting 9596 baud), and only configured ports
/// are put back on reconnect.
struct SerialLine {
    enum class Parity : std::uint8_t { kNone, kEven, kOdd };

    std::uint32_t baudrate;
    Parity parity = Parity::kNone;
    std::uint8_t stop_bits = 1;
    /// RX inversion, where the board has the bit (mc02 DBUS). Unset: leave the port's default.
    std::optional<bool> rx_inverted = std::nullopt;
};

} // namespace hcs_core::hardware::device
