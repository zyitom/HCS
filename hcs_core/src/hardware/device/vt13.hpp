#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <variant>

#include <eigen3/Eigen/Dense>
#include <hcs_executor/component.hpp>
#include <hcs_msgs/keyboard.hpp>
#include <hcs_msgs/mouse.hpp>
#include <hcs_msgs/switch.hpp>
#include <hcs_base/protocol/dji_crc.hpp>
#include <hcs_base/protocol/ring_buffer.hpp>

// DJI 遥控帧为原生类型 packed，依赖小端主机。
static_assert(std::endian::native == std::endian::little, "wire structs assume a LE host");

namespace hcs_core::hardware::device {

class Vt13 {
public:
    enum class ModeSwitch : uint8_t {
        kUnknown = 0,
        kCine = 1,
        kNormal = 2,
        kSport = 3,
    };

    Vt13() = default;

    // libhcs IO 线程（事件域）调用：和周期域一样不许打日志——同步写日志一旦被堵住，
    // 整条 IO 线程就停在这里，三块板的反馈一起断。溢出只计数，由硬件组件的尽力域
    // 定时器读 overflow_count() / overflow_dropped_bytes() 报出来。
    void store_status(std::span<const std::byte> uart_data) noexcept {
        store_calls_.fetch_add(1, std::memory_order_relaxed);
        received_bytes_.fetch_add(uart_data.size(), std::memory_order_relaxed);

        const auto written = data_buffer_.emplace_back_n(
            [iter = uart_data.cbegin()](std::byte* storage) mutable noexcept {
                *storage = *iter++;
            },
            uart_data.size());
        if (written != uart_data.size()) {
            overflow_count_.fetch_add(1, std::memory_order_relaxed);
            overflow_dropped_bytes_.fetch_add(
                uart_data.size() - written, std::memory_order_relaxed);
        }
    }

    /// 输入缓冲溢出的次数与累计丢弃字节数（尽力域读，relaxed 即可）。
    uint64_t overflow_count() const noexcept {
        return overflow_count_.load(std::memory_order_relaxed);
    }
    uint64_t overflow_dropped_bytes() const noexcept {
        return overflow_dropped_bytes_.load(std::memory_order_relaxed);
    }

    /// @param now 有效性计时的时间源。周期域调用方传 tick.scheduled（同一
    /// steady_clock 时基），把 Clock::now() 这个系统调用从 RT 拍内挪出去。
    void update_status(std::chrono::steady_clock::time_point now) {
        auto readable = data_buffer_.readable();
        peak_readable_ = std::max(peak_readable_, readable);

        while (readable) {
            ReadResult result = VerificationFailed{};

            const std::byte front = *data_buffer_.peek_front();
            if (front == std::byte{0xa9})
                result = read_remote_control_data(readable, now);
            else if (front == std::byte(0xa5))
                result = read_referee_style_data(readable);
            else {
                // 周期域禁日志（-Wfunction-effects）：失败只计数，
                // 统计由硬件组件的尽力域定时器读 counters 呈现。
                unknown_prefix_count_++;
            }

            if (std::holds_alternative<Incomplete>(result)) {
                break;
            }
            if (std::holds_alternative<VerificationFailed>(result)) {
                verification_failures_++;
                data_buffer_.pop_front([](std::byte&&) noexcept {});
                readable--;
                continue;
            }
            // get_if 而不是 get：get 会抛 bad_variant_access，clang 的
            // -Wfunction-effects 据此拒绝把这条路径推断为 nonblocking。
            if (const auto* success = std::get_if<Success>(&result)) {
                readable -= success->read;
                continue;
            }
        }

        refresh_validity(now);
    }

    ModeSwitch mode_switch() const noexcept { return mode_switch_; }
    bool valid() const noexcept { return valid_; }

    void set_timeout_enabled(bool enabled) { timeout_enabled_ = enabled; }

    const Eigen::Vector2d& joystick_left() const noexcept { return joystick_left_; }
    const Eigen::Vector2d& joystick_right() const noexcept { return joystick_right_; }

    const Eigen::Vector2d& mouse_velocity() const noexcept { return mouse_velocity_; }
    double mouse_wheel() const noexcept { return mouse_wheel_; }

    hcs_msgs::Mouse mouse() const noexcept { return mouse_; }
    hcs_msgs::Keyboard keyboard() const noexcept { return keyboard_; }

private:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    static constexpr auto kFreshTimeout = std::chrono::milliseconds(500);
    static constexpr auto kVerificationLogInterval = std::chrono::seconds(1);
    static constexpr auto kStatisticsLogInterval = std::chrono::seconds(5);
    static constexpr std::size_t kRefereeFrameMaxSize = 256;

    struct Incomplete {};
    struct VerificationFailed {};
    struct Success {
        std::size_t read;
    };
    using ReadResult = std::variant<Incomplete, VerificationFailed, Success>;

    struct [[gnu::packed]] RemoteControlData {
        static constexpr uint16_t kHeaderMagic = 0x53a9;

        uint16_t header;

        uint16_t joystick_channel0 : 11;
        uint16_t joystick_channel1 : 11;
        uint16_t joystick_channel2 : 11;
        uint16_t joystick_channel3 : 11;

        uint8_t mode_switch         : 2;
        uint8_t pause_button        : 1;
        uint8_t left_custom_button  : 1;
        uint8_t right_custom_button : 1;
        uint16_t dial               : 11;
        uint8_t trigger             : 1;
        uint8_t padding1            : 3;

        int16_t mouse_velocity_x;
        int16_t mouse_velocity_y;
        int16_t mouse_velocity_z;
        uint8_t mouse_left   : 2;
        uint8_t mouse_right  : 2;
        uint8_t mouse_middle : 2;
        uint8_t padding2     : 2;

        uint16_t keyboard;

        uint16_t crc16;
    };

    struct [[gnu::packed]] RefereeFrameHeader {
        uint8_t sof;
        uint16_t data_length;
        uint8_t seq;
        uint8_t crc8;
    };

    ReadResult read_remote_control_data(const std::size_t readable, const TimePoint now) {
        if (readable < sizeof(RemoteControlData))
            return Incomplete{};

        RemoteControlData data;
        data_buffer_.peek_front_n(
            [dst = reinterpret_cast<std::byte*>(&data)](std::byte src) mutable noexcept {
                *dst++ = src;
            },
            sizeof(RemoteControlData));

        if (data.header != RemoteControlData::kHeaderMagic) {
            remote_bad_header_count_++;
            return VerificationFailed{};
        }
        if (!hcs_utility::dji_crc::verify_crc16(data)) {
            remote_bad_crc_count_++;
            return VerificationFailed{};
        }

        data_buffer_.pop_front_n([](std::byte&&) noexcept {}, sizeof(RemoteControlData));

        update_remote_control_data(data);
        valid_ = true;
        last_remote_control_received_at_ = now;
        remote_success_count_++;
        return Success{sizeof(RemoteControlData)};
    }

    void update_remote_control_data(const RemoteControlData& data) {
        mode_switch_ = static_cast<ModeSwitch>(data.mode_switch + 1);

        joystick_right_ = {
            channel_to_double(static_cast<uint16_t>(data.joystick_channel1)),
            -channel_to_double(static_cast<uint16_t>(data.joystick_channel0)),
        };
        joystick_left_ = {
            channel_to_double(static_cast<uint16_t>(data.joystick_channel2)),
            -channel_to_double(static_cast<uint16_t>(data.joystick_channel3)),
        };

        mouse_velocity_ = {
            -data.mouse_velocity_y / 32768.0,
            -data.mouse_velocity_x / 32768.0,
        };
        mouse_wheel_ = -static_cast<double>(data.mouse_velocity_z) / 32768.0;

        mouse_ = {
            .left = static_cast<bool>(data.mouse_left),
            .right = static_cast<bool>(data.mouse_right),
        };
        keyboard_ = std::bit_cast<hcs_msgs::Keyboard>(data.keyboard);
    }

    ReadResult read_referee_style_data(const std::size_t readable) {
        if (readable < sizeof(RefereeFrameHeader))
            return Incomplete{};

        RefereeFrameHeader header;
        data_buffer_.peek_front_n(
            [dst = reinterpret_cast<std::byte*>(&header)](std::byte src) mutable noexcept {
                *dst++ = src;
            },
            sizeof(RefereeFrameHeader));

        if (!hcs_utility::dji_crc::verify_crc8(header)) {
            referee_bad_crc8_count_++;
            return VerificationFailed{};
        }

        const std::size_t total_frame_size =
            sizeof(RefereeFrameHeader) + 2 + header.data_length + 2;
        if (total_frame_size > kRefereeFrameMaxSize) {
            referee_oversize_count_++;
            return VerificationFailed{};
        }
        if (readable < total_frame_size)
            return Incomplete{};

        data_buffer_.pop_front_n([](std::byte&&) noexcept {}, total_frame_size);
        referee_discarded_count_++;
        return Success{total_frame_size};
    }

    bool should_log_verification_failure(const TimePoint now) {
        if (last_verification_log_time_ != TimePoint::min()
            && now - last_verification_log_time_ < kVerificationLogInterval)
            return false;
        last_verification_log_time_ = now;
        return true;
    }

    void refresh_validity(const TimePoint now) {
        if (!timeout_enabled_ || !valid_ || now - last_remote_control_received_at_ <= kFreshTimeout)
            return;

        reset_remote_control_state();
        valid_ = false;
    }

    void reset_remote_control_state() {
        mode_switch_ = ModeSwitch::kUnknown;
        joystick_left_ = Eigen::Vector2d::Zero();
        joystick_right_ = Eigen::Vector2d::Zero();
        mouse_velocity_ = Eigen::Vector2d::Zero();
        mouse_wheel_ = 0;
        mouse_ = hcs_msgs::Mouse::zero();
        keyboard_ = hcs_msgs::Keyboard::zero();
    }

    static double channel_to_double(int32_t value) {
        value -= 1024;
        if (-660 <= value && value <= 660)
            return value / 660.0;
        return 0.0;
    }

    hcs_utility::RingBuffer<std::byte> data_buffer_{1024};

    std::atomic<uint64_t> store_calls_{0};
    std::atomic<uint64_t> received_bytes_{0};
    std::atomic<uint64_t> overflow_count_{0};
    std::atomic<uint64_t> overflow_dropped_bytes_{0};

    TimePoint last_remote_control_received_at_ = TimePoint::min();
    TimePoint last_verification_log_time_ = TimePoint::min();
    TimePoint last_statistics_log_time_ = TimePoint::min();

    bool valid_ = false;
    bool timeout_enabled_ = true;
    std::size_t peak_readable_ = 0;
    std::size_t remote_success_count_ = 0;
    std::size_t verification_failures_ = 0;
    std::size_t remote_bad_header_count_ = 0;
    std::size_t remote_bad_crc_count_ = 0;
    std::size_t referee_discarded_count_ = 0;
    std::size_t referee_bad_crc8_count_ = 0;
    std::size_t referee_oversize_count_ = 0;
    std::size_t unknown_prefix_count_ = 0;

    ModeSwitch mode_switch_ = ModeSwitch::kUnknown;

    Eigen::Vector2d joystick_left_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d joystick_right_ = Eigen::Vector2d::Zero();

    Eigen::Vector2d mouse_velocity_ = Eigen::Vector2d::Zero();
    double mouse_wheel_ = 0;

    hcs_msgs::Mouse mouse_ = hcs_msgs::Mouse::zero();
    hcs_msgs::Keyboard keyboard_ = hcs_msgs::Keyboard::zero();
};

} // namespace hcs_core::hardware::device
