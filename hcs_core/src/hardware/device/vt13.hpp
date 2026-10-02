#pragma once

#include <algorithm>
#include <array>
#include <bit>
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

// DJI 遥控帧为原生类型 packed，依赖小端主机。
static_assert(std::endian::native == std::endian::little, "wire structs assume a LE host");

namespace hcs_core::hardware::device {

/// VT13 图传遥控链路的字节流：遥控帧（0xA9 0x53 开头，21 字节，CRC16）与裁判系统格式的帧
/// （0xA5 开头）混在同一路串口上。只解遥控帧，裁判格式的帧按帧长跳过。
///
/// 只写协议：全部成员都在控制线程上，字节由端口包装层（board::Serial<>）按到达顺序喂进来；
/// 掉线判定也在包装层，结果经 set_online() 告诉这里。
class Vt13 {
public:
    enum class ModeSwitch : std::uint8_t {
        kUnknown = 0,
        kCine = 1,
        kNormal = 2,
        kSport = 3,
    };

    Vt13() = default;

    /// 喂一段串口字节，帧边界不保证：半帧留在缓存里等下一段。
    /// 周期域禁日志（-Wfunction-effects）：校验失败只计数。
    /// @return 这一段里有没有解出至少一帧遥控数据（裁判格式的帧不算）
    bool on_bytes(std::span<const std::byte> uart_data) noexcept {
        bool received = false;
        while (!uart_data.empty()) {
            // parse_cache() 之后缓存里最多剩一个不完整的帧（< kFrameMaxSize），所以这里总有空位。
            const auto count = std::min(uart_data.size(), cache_.size() - cached_);
            std::memcpy(cache_.data() + cached_, uart_data.data(), count);
            cached_ += count;
            uart_data = uart_data.subspan(count);

            received |= parse_cache();
        }
        return received;
    }

    /// 链路是否在线，每拍由掉线计数的那一方告知。掉线的那一拍把摇杆、鼠标、键盘归零、
    /// 挡位置未知：读它的人拿到的永远不是掉线前的最后一帧。
    void set_online(bool online) noexcept {
        if (valid_ && !online)
            reset_remote_control_state();
        valid_ = online;
    }

    ModeSwitch mode_switch() const noexcept { return mode_switch_; }
    bool valid() const noexcept { return valid_; }

    const Eigen::Vector2d& joystick_left() const noexcept { return joystick_left_; }
    const Eigen::Vector2d& joystick_right() const noexcept { return joystick_right_; }

    const Eigen::Vector2d& mouse_velocity() const noexcept { return mouse_velocity_; }
    double mouse_wheel() const noexcept { return mouse_wheel_; }

    hcs_msgs::Mouse mouse() const noexcept { return mouse_; }
    hcs_msgs::Keyboard keyboard() const noexcept { return keyboard_; }

private:
    static constexpr std::size_t kFrameMaxSize = 256;

    struct Incomplete {};
    struct VerificationFailed {};
    struct Success {
        std::size_t read;
        bool remote_control; ///< 解出的是遥控帧（而不是被跳过的裁判格式帧）
    };
    using ReadResult = std::variant<Incomplete, VerificationFailed, Success>;

    struct [[gnu::packed]] RemoteControlData {
        static constexpr std::uint16_t kHeaderMagic = 0x53a9;

        std::uint16_t header;

        std::uint16_t joystick_channel0 : 11;
        std::uint16_t joystick_channel1 : 11;
        std::uint16_t joystick_channel2 : 11;
        std::uint16_t joystick_channel3 : 11;

        std::uint8_t mode_switch         : 2;
        std::uint8_t pause_button        : 1;
        std::uint8_t left_custom_button  : 1;
        std::uint8_t right_custom_button : 1;
        std::uint16_t dial               : 11;
        std::uint8_t trigger             : 1;
        std::uint8_t padding1            : 3;

        std::int16_t mouse_velocity_x;
        std::int16_t mouse_velocity_y;
        std::int16_t mouse_velocity_z;
        std::uint8_t mouse_left   : 2;
        std::uint8_t mouse_right  : 2;
        std::uint8_t mouse_middle : 2;
        std::uint8_t padding2     : 2;

        std::uint16_t keyboard;

        std::uint16_t crc16;
    };
    static_assert(sizeof(RemoteControlData) <= kFrameMaxSize);

    struct [[gnu::packed]] RefereeFrameHeader {
        std::uint8_t sof;
        std::uint16_t data_length;
        std::uint8_t seq;
        std::uint8_t crc8;
    };

    /// 从缓存头部开始逐帧解，解不动（剩下的不够一帧）就停，把剩下的挪到缓存开头。
    /// 开头不是任何一种帧头、或者校验不过：丢一个字节重新对齐。
    bool parse_cache() noexcept {
        bool received = false;
        std::size_t offset = 0;
        while (offset < cached_) {
            const auto pending =
                std::span<const std::byte>{cache_}.subspan(offset, cached_ - offset);

            ReadResult result = VerificationFailed{};
            if (pending.front() == std::byte{0xa9})
                result = read_remote_control_data(pending);
            else if (pending.front() == std::byte{0xa5})
                result = read_referee_style_data(pending);
            else
                unknown_prefix_count_++;

            if (std::holds_alternative<Incomplete>(result))
                break;
            // get_if 而不是 get：get 会抛 bad_variant_access，clang 的
            // -Wfunction-effects 据此拒绝把这条路径推断为 nonblocking。
            if (const auto* success = std::get_if<Success>(&result)) {
                offset += success->read;
                received |= success->remote_control;
                continue;
            }
            verification_failures_++;
            offset += 1;
        }

        cached_ -= offset;
        std::memmove(cache_.data(), cache_.data() + offset, cached_);
        return received;
    }

    ReadResult read_remote_control_data(std::span<const std::byte> pending) noexcept {
        if (pending.size() < sizeof(RemoteControlData))
            return Incomplete{};

        // memcpy 而不是把指针转过去读：缓存里的帧不保证对齐。
        RemoteControlData data;
        std::memcpy(&data, pending.data(), sizeof(RemoteControlData));

        if (data.header != RemoteControlData::kHeaderMagic) {
            remote_bad_header_count_++;
            return VerificationFailed{};
        }
        if (!hcs_utility::dji_crc::verify_crc16(data)) {
            remote_bad_crc_count_++;
            return VerificationFailed{};
        }

        update_remote_control_data(data);
        remote_success_count_++;
        return Success{.read = sizeof(RemoteControlData), .remote_control = true};
    }

    void update_remote_control_data(const RemoteControlData& data) noexcept {
        mode_switch_ = static_cast<ModeSwitch>(data.mode_switch + 1);

        joystick_right_ = {
            channel_to_double(static_cast<std::uint16_t>(data.joystick_channel1)),
            -channel_to_double(static_cast<std::uint16_t>(data.joystick_channel0)),
        };
        joystick_left_ = {
            channel_to_double(static_cast<std::uint16_t>(data.joystick_channel2)),
            -channel_to_double(static_cast<std::uint16_t>(data.joystick_channel3)),
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
        // 先拷到局部变量：bit_cast 按引用收参数，而 packed 结构体里这个成员在奇数偏移上，
        // 直接把引用绑上去是未对齐访问（UBSan 报出）。
        const std::uint16_t keyboard = data.keyboard;
        keyboard_ = std::bit_cast<hcs_msgs::Keyboard>(keyboard);
    }

    ReadResult read_referee_style_data(std::span<const std::byte> pending) noexcept {
        if (pending.size() < sizeof(RefereeFrameHeader))
            return Incomplete{};

        RefereeFrameHeader header;
        std::memcpy(&header, pending.data(), sizeof(RefereeFrameHeader));

        if (!hcs_utility::dji_crc::verify_crc8(header)) {
            referee_bad_crc8_count_++;
            return VerificationFailed{};
        }

        const std::size_t total_frame_size =
            sizeof(RefereeFrameHeader) + 2 + header.data_length + 2;
        if (total_frame_size > kFrameMaxSize) {
            referee_oversize_count_++;
            return VerificationFailed{};
        }
        if (pending.size() < total_frame_size)
            return Incomplete{};

        referee_discarded_count_++;
        return Success{.read = total_frame_size, .remote_control = false};
    }

    void reset_remote_control_state() noexcept {
        mode_switch_ = ModeSwitch::kUnknown;
        joystick_left_ = Eigen::Vector2d::Zero();
        joystick_right_ = Eigen::Vector2d::Zero();
        mouse_velocity_ = Eigen::Vector2d::Zero();
        mouse_wheel_ = 0;
        mouse_ = hcs_msgs::Mouse::zero();
        keyboard_ = hcs_msgs::Keyboard::zero();
    }

    static double channel_to_double(std::int32_t value) noexcept {
        value -= 1024;
        if (-660 <= value && value <= 660)
            return value / 660.0;
        return 0.0;
    }

    /// 还没凑成一帧的字节。容量取最长帧的两倍：解析停下时剩下的不到一帧，
    /// 所以每喂进来一段，缓存里至少还有一帧长的空位，on_bytes() 的循环必然前进。
    std::array<std::byte, 2 * kFrameMaxSize> cache_{};
    std::size_t cached_ = 0;

    bool valid_ = false;
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
