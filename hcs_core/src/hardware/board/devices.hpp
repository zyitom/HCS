#pragma once

#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <utility>

#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>

#include "hardware/board/board.hpp"
#include "hardware/board/device.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/hipnuc.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/vt13_remote.hpp"

namespace hcs_core::hardware::board {

// ============================================================================
// 把驱动接到端口上。驱动本身（device::*）不认识端口和板卡；这里的包装在构造时
// 把驱动登记到所在的口，并把端口接口转发给驱动：
//
//   DmMotor pitch{can2, "/gimbal/pitch", {.control_mode = ..., ...}};
//
// 驱动提供的：总线要求（kCanBitrate / kCanFd）或串口要求（serial_line()），
// 可选的 describe() / take_problem() / command_slot() / faulted()。
// ============================================================================

namespace detail {

template <class Driver>
util::DeviceHealth health_of(const Driver& driver) noexcept HCS_NONBLOCKING {
    bool faulted = false;
    if constexpr (requires { driver.faulted(); })
        faulted = driver.faulted();
    return {.received = driver.received(), .online = driver.online(), .faulted = faulted};
}

} // namespace detail

/// CAN 总线上的一台设备：Driver 需要 (status, command, name, config) 构造，并提供
/// recv_id() / send_id() / match_then_store_status() / update_status() / append_command()。
template <class Driver>
class Can
    : public CanDevice
    , public Driver {
public:
    using Config = typename Driver::Config;

    Can(CanBus& bus, std::string name, const Config& config)
        : CanDevice(bus.board(), std::move(name))
        , Driver(bus.board(), bus.board().command(), this->name(), config) {
        bus.attach(*this);
    }

    [[nodiscard]] std::uint32_t feedback_id() const noexcept override { return Driver::recv_id(); }
    [[nodiscard]] std::uint32_t command_id() const noexcept override { return Driver::send_id(); }
    [[nodiscard]] std::optional<std::size_t> command_slot() const noexcept override {
        if constexpr (requires(const Driver& d) { d.command_slot(); })
            return Driver::command_slot();
        else
            return std::nullopt;
    }
    [[nodiscard]] std::uint32_t bitrate() const noexcept override { return Driver::kCanBitrate; }
    [[nodiscard]] bool fd() const noexcept override { return Driver::kCanFd; }

    void receive(std::uint32_t can_id, std::span<const std::byte> data) noexcept
        HCS_NONBLOCKING override {
        Driver::match_then_store_status(can_id, data);
    }

    void update_status(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        Driver::update_status();
        publish_health(detail::health_of<Driver>(*this));
    }

    void append_command(util::CommandFrames::Bus& bus, bool safe) HCS_NONBLOCKING override {
        Driver::append_command(bus, safe);
    }

    [[nodiscard]] std::string describe() const override {
        if constexpr (requires(const Driver& d) { d.describe(); })
            return Driver::describe();
        else
            return std::format("feedback 0x{:x} command 0x{:x}", feedback_id(), command_id());
    }

    std::optional<std::string> take_problem() override {
        if constexpr (requires(Driver& d) { d.take_problem(); })
            return Driver::take_problem();
        else
            return std::nullopt;
    }
};

/// 串口上的一台设备：Driver 需要 (status, name, config) 构造，并提供 serial_line() /
/// store_status() / update_status()（可带 tick.scheduled）/ received() / online()。
template <class Driver>
class Serial
    : public SerialDevice
    , public Driver {
public:
    using Config = typename Driver::Config;

    Serial(SerialPort& port, std::string name, const Config& config)
        : SerialDevice(port.board(), std::move(name))
        , Driver(port.board(), this->name(), config) {
        port.attach(*this);
    }

    [[nodiscard]] device::SerialLine serial_line() const noexcept override {
        return Driver::serial_line();
    }

    void receive(std::span<const std::byte> data) noexcept HCS_NONBLOCKING override {
        Driver::store_status(data);
    }

    void update_status(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        if constexpr (requires(Driver& d) { d.update_status(tick.scheduled); })
            Driver::update_status(tick.scheduled);
        else
            Driver::update_status();
        publish_health(detail::health_of<Driver>(*this));
    }

    [[nodiscard]] std::string describe() const override {
        if constexpr (requires(const Driver& d) { d.describe(); })
            return Driver::describe();
        else
            return std::format("serial device at {} baud", serial_line().baudrate);
    }

    std::optional<std::string> take_problem() override {
        if constexpr (requires(Driver& d) { d.take_problem(); })
            return Driver::take_problem();
        else
            return std::nullopt;
    }
};

// 常用驱动的简写，整车文件里直接用。
using DmMotor = Can<device::DmMotor>;
using LkMotor = Can<device::LkMotor>;
using DjiMotor = Can<device::DjiMotor>;
using Hipnuc = Serial<device::Hipnuc>;
using Vt13Remote = Serial<device::Vt13Remote>;

} // namespace hcs_core::hardware::board
