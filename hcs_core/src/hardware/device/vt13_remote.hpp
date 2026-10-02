#pragma once

#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <string>

#include <hcs_executor/component.hpp>
#include <hcs_msgs/switch.hpp>

#include "hardware/device/remote_outputs.hpp"
#include "hardware/device/serial_line.hpp"
#include "hardware/device/vt13.hpp"

namespace hcs_core::hardware::device {

/// VT13 图传遥控接收机：串口字节交给 Vt13 解帧，按挡位决定这一拍 /remote/* 上是什么。
///
///   S 挡：遥控生效（比赛用），两个拨杆强制 MIDDLE / MIDDLE；
///   C 挡：安全态（救车用），两个拨杆强制 DOWN / DOWN，其余全空；
///   其余（失联 / N 挡 / 挡位未知）：空安全态，全部归零，拨杆 UNKNOWN。
///
/// 输出名是 RemoteOutputs 固定的（/remote/*），name 只用于日志。
class Vt13Remote {
public:
    struct Config {
        /// 接收机串口速率（待上台架核对）。
        std::uint32_t baudrate = 115200;
        /// 连续多少拍没有遥控帧算失联。500 拍即 1 kHz 回路下的 500 ms。
        int offline_timeout = 500;
    };

    Vt13Remote(hcs_executor::Component& status_component, const std::string&, const Config& config)
        : outputs_(status_component)
        , baudrate_(config.baudrate) {}

    Vt13Remote(const Vt13Remote&) = delete;
    Vt13Remote& operator=(const Vt13Remote&) = delete;

    [[nodiscard]] SerialLine serial_line() const noexcept { return {.baudrate = baudrate_}; }

    /// 周期域：一段串口字节。返回有没有解出遥控帧。
    bool on_bytes(std::span<const std::byte> data) noexcept { return vt13_.on_bytes(data); }

    /// 周期域，每拍一次：失联时先把 Vt13 归零，再决定这一拍发布什么。
    void on_tick(bool online) noexcept {
        vt13_.set_online(online);
        outputs_.publish(state());
    }

    [[nodiscard]] hcs_msgs::Switch switch_left() const noexcept { return outputs_.switch_left(); }

    [[nodiscard]] std::string describe() const {
        return std::format("VT13 remote at {} baud -> /remote/*", baudrate_);
    }

private:
    [[nodiscard]] RemoteOutputs::State state() const noexcept {
        RemoteOutputs::State state; // 空安全态
        if (!vt13_.valid())
            return state;

        switch (vt13_.mode_switch()) {
        case Vt13::ModeSwitch::kSport:
            state.joystick_right = vt13_.joystick_right();
            state.joystick_left = vt13_.joystick_left();
            state.switch_right = hcs_msgs::Switch::MIDDLE;
            state.switch_left = hcs_msgs::Switch::MIDDLE;
            state.mouse_velocity = vt13_.mouse_velocity();
            state.mouse_wheel = vt13_.mouse_wheel();
            state.mouse = vt13_.mouse();
            state.keyboard = vt13_.keyboard();
            break;
        case Vt13::ModeSwitch::kCine:
            state.switch_right = hcs_msgs::Switch::DOWN;
            state.switch_left = hcs_msgs::Switch::DOWN;
            break;
        case Vt13::ModeSwitch::kNormal:
        case Vt13::ModeSwitch::kUnknown: break;
        }
        return state;
    }

    Vt13 vt13_;
    RemoteOutputs outputs_;
    std::uint32_t baudrate_;
};

} // namespace hcs_core::hardware::device
