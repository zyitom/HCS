#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <eigen3/Eigen/Dense>
#include <hcs_msgs/keyboard.hpp>
#include <hcs_msgs/mouse.hpp>
#include <hcs_msgs/switch.hpp>

// DBUS 帧里的多字节字段按小端排，位域从低位排起。
static_assert(std::endian::native == std::endian::little, "wire layout assumes a LE host");

namespace hcs_core::hardware::device {

/// DR16 接收机的 DBUS 字节流：18 字节一帧，**没有帧头，也没有校验**，帧与帧之间只靠线路上的
/// 空闲隔开（一帧发 2 ms，然后静默到下一帧）。
///
///   位 0-43     四个 11 位摇杆通道，中位 1024，量程 364..1684
///   位 44-45    右拨杆，46-47 左拨杆：1 上 / 2 下 / 3 中
///   字节 6-11   鼠标 x / y / z，int16
///   字节 12-13  鼠标左键、右键，各一个字节（0 / 1）
///   字节 14-15  键盘位图
///   字节 16-17  拨轮，同摇杆通道的编码
///
/// 只写协议：全部成员都在控制线程上，字节由端口包装层（board::Serial<>）按到达顺序喂进来；
/// 掉线判定也在包装层，结果经 set_online() 告诉这里。
///
/// 对齐。字节流里没有任何东西标着"一帧从这里开始"，所以靠两条规矩：
///   - 空闲：一拍里一个字节都没来，手上那半帧就不要了（end_of_tick()）。帧间的静默远比一拍长，
///     所以半帧只可能是上电时从中间接上的、或者丢过字节之后剩下的。
///   - 合理性：凑满 18 字节但通道或拨杆的值不在协议允许的范围里，说明没对齐，丢一个字节再试。
/// 第二条是给"控制回路慢到每一拍都有字节"的情况兜底的；它会放过恰好看起来合理的错位帧，
/// 所以真正靠的是第一条。
class Dr16 {
public:
    static constexpr std::size_t kFrameSize = 18;

    Dr16() = default;

    /// 喂一段串口字节，帧边界不保证。
    /// @return 这一段里有没有解出至少一帧
    bool on_bytes(std::span<const std::byte> uart_data) noexcept {
        received_this_tick_ = received_this_tick_ || !uart_data.empty();

        bool decoded = false;
        for (const std::byte byte : uart_data) {
            frame_[filled_++] = byte;
            if (filled_ < kFrameSize)
                continue;

            if (decode()) {
                filled_ = 0;
                decoded = true;
            } else {
                ++rejected_frames_;
                std::memmove(frame_.data(), frame_.data() + 1, kFrameSize - 1);
                filled_ = kFrameSize - 1;
            }
        }
        return decoded;
    }

    /// 每拍一次，在这一拍的 on_bytes() 都调完之后：这一拍没有字节来，就把半帧扔掉。
    void end_of_tick() noexcept {
        if (!received_this_tick_)
            filled_ = 0;
        received_this_tick_ = false;
    }

    /// 链路是否在线，每拍由掉线计数的那一方告知。掉线的那一拍把摇杆、拨杆、鼠标、键盘、拨轮
    /// 全部归零：读它的人拿到的永远不是掉线前的最后一帧。
    void set_online(bool online) noexcept {
        if (valid_ && !online)
            reset();
        valid_ = online;
    }

    [[nodiscard]] bool valid() const noexcept { return valid_; }

    [[nodiscard]] const Eigen::Vector2d& joystick_right() const noexcept { return joystick_right_; }
    [[nodiscard]] const Eigen::Vector2d& joystick_left() const noexcept { return joystick_left_; }

    [[nodiscard]] hcs_msgs::Switch switch_right() const noexcept { return switch_right_; }
    [[nodiscard]] hcs_msgs::Switch switch_left() const noexcept { return switch_left_; }

    [[nodiscard]] const Eigen::Vector2d& mouse_velocity() const noexcept { return mouse_velocity_; }
    [[nodiscard]] double mouse_wheel() const noexcept { return mouse_wheel_; }

    [[nodiscard]] hcs_msgs::Mouse mouse() const noexcept { return mouse_; }
    [[nodiscard]] hcs_msgs::Keyboard keyboard() const noexcept { return keyboard_; }

    /// 拨轮，-1..1。
    [[nodiscard]] double rotary_knob() const noexcept { return rotary_knob_; }
    /// 拨轮当三挡开关用：过 ±0.7 换挡，带 0.05 的回差防抖。
    [[nodiscard]] hcs_msgs::Switch rotary_knob_switch() const noexcept {
        return rotary_knob_switch_;
    }

    /// 凑满了 18 字节但不像一帧的次数。一直在涨说明流没对齐：查波特率、校验位、取反。
    [[nodiscard]] std::uint32_t rejected_frames() const noexcept { return rejected_frames_; }

private:
    static constexpr int kChannelCentre = 1024;
    static constexpr int kChannelRange = 660;

    struct [[gnu::packed]] Sticks {
        std::uint64_t joystick_channel0 : 11;
        std::uint64_t joystick_channel1 : 11;
        std::uint64_t joystick_channel2 : 11;
        std::uint64_t joystick_channel3 : 11;
        std::uint64_t switch_right      : 2;
        std::uint64_t switch_left       : 2;
        std::uint64_t padding           : 16;
    };
    static_assert(sizeof(Sticks) == 8);

    struct [[gnu::packed]] Pointer {
        std::int16_t mouse_velocity_x;
        std::int16_t mouse_velocity_y;
        std::int16_t mouse_velocity_z;
        std::uint8_t mouse_left;
        std::uint8_t mouse_right;
    };
    static_assert(sizeof(Pointer) == 8);

    struct [[gnu::packed]] Keys {
        std::uint16_t keyboard;
        std::uint16_t rotary_knob;
    };
    static_assert(sizeof(Keys) == 4);
    static_assert(6 + sizeof(Pointer) + sizeof(Keys) == kFrameSize);

    [[nodiscard]] static constexpr bool channel_in_range(int raw) noexcept {
        return kChannelCentre - kChannelRange <= raw && raw <= kChannelCentre + kChannelRange;
    }

    [[nodiscard]] static constexpr double channel_to_double(int raw) noexcept {
        return channel_in_range(raw) ? (raw - kChannelCentre) / static_cast<double>(kChannelRange)
                                     : 0.0;
    }

    /// 把凑满的 18 字节当一帧解。值不在协议允许的范围里就不解，返回 false。
    bool decode() noexcept {
        // memcpy 到对齐的局部变量上再读：缓存里的帧不保证对齐。前 6 个字节摊在一个 64 位里。
        std::uint64_t sticks_bits = 0;
        std::memcpy(&sticks_bits, frame_.data(), 6);
        const auto sticks = std::bit_cast<Sticks>(sticks_bits);
        Pointer pointer;
        std::memcpy(&pointer, frame_.data() + 6, sizeof(pointer));
        Keys keys;
        std::memcpy(&keys, frame_.data() + 6 + sizeof(pointer), sizeof(keys));

        const int channel0 = static_cast<int>(sticks.joystick_channel0);
        const int channel1 = static_cast<int>(sticks.joystick_channel1);
        const int channel2 = static_cast<int>(sticks.joystick_channel2);
        const int channel3 = static_cast<int>(sticks.joystick_channel3);
        const bool plausible =
            channel_in_range(channel0) && channel_in_range(channel1) && channel_in_range(channel2)
            && channel_in_range(channel3) && sticks.switch_right != 0 && sticks.switch_left != 0
            && pointer.mouse_left <= 1 && pointer.mouse_right <= 1;
        if (!plausible)
            return false;

        // 输出坐标：x 向前，y 向左。
        joystick_right_ = {channel_to_double(channel1), -channel_to_double(channel0)};
        joystick_left_ = {channel_to_double(channel3), -channel_to_double(channel2)};

        // 线上的 1 上 / 2 下 / 3 中 与 hcs_msgs::Switch 的取值相同。
        switch_right_ = static_cast<hcs_msgs::Switch>(sticks.switch_right);
        switch_left_ = static_cast<hcs_msgs::Switch>(sticks.switch_left);

        mouse_velocity_ = {
            -pointer.mouse_velocity_y / 32768.0,
            -pointer.mouse_velocity_x / 32768.0,
        };
        mouse_wheel_ = -pointer.mouse_velocity_z / 32768.0;
        mouse_ = {.left = pointer.mouse_left != 0, .right = pointer.mouse_right != 0};

        keyboard_ = std::bit_cast<hcs_msgs::Keyboard>(keys.keyboard);
        rotary_knob_ = channel_to_double(keys.rotary_knob);
        update_rotary_knob_switch();
        return true;
    }

    void update_rotary_knob_switch() noexcept {
        constexpr double kDivider = 0.7;
        constexpr double kHysteresis = 0.05;
        double upper = kDivider;
        double lower = -kDivider;

        // 离开当前挡要多走 kHysteresis：拨轮停在分界线上时不来回跳。
        if (rotary_knob_switch_ == hcs_msgs::Switch::UP) {
            upper -= kHysteresis;
            lower -= kHysteresis;
        } else if (rotary_knob_switch_ == hcs_msgs::Switch::MIDDLE) {
            upper += kHysteresis;
            lower -= kHysteresis;
        } else if (rotary_knob_switch_ == hcs_msgs::Switch::DOWN) {
            upper += kHysteresis;
            lower += kHysteresis;
        }

        const double value = -rotary_knob_;
        rotary_knob_switch_ = value > upper   ? hcs_msgs::Switch::UP
                            : value < lower   ? hcs_msgs::Switch::DOWN
                                              : hcs_msgs::Switch::MIDDLE;
    }

    void reset() noexcept {
        joystick_right_ = Eigen::Vector2d::Zero();
        joystick_left_ = Eigen::Vector2d::Zero();
        switch_right_ = hcs_msgs::Switch::UNKNOWN;
        switch_left_ = hcs_msgs::Switch::UNKNOWN;
        mouse_velocity_ = Eigen::Vector2d::Zero();
        mouse_wheel_ = 0.0;
        mouse_ = hcs_msgs::Mouse::zero();
        keyboard_ = hcs_msgs::Keyboard::zero();
        rotary_knob_ = 0.0;
        rotary_knob_switch_ = hcs_msgs::Switch::UNKNOWN;
    }

    /// 正在凑的那一帧，和已经凑了多少。
    std::array<std::byte, kFrameSize> frame_{};
    std::size_t filled_ = 0;
    bool received_this_tick_ = false;
    std::uint32_t rejected_frames_ = 0;

    bool valid_ = false;

    Eigen::Vector2d joystick_right_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d joystick_left_ = Eigen::Vector2d::Zero();
    hcs_msgs::Switch switch_right_ = hcs_msgs::Switch::UNKNOWN;
    hcs_msgs::Switch switch_left_ = hcs_msgs::Switch::UNKNOWN;

    Eigen::Vector2d mouse_velocity_ = Eigen::Vector2d::Zero();
    double mouse_wheel_ = 0.0;
    hcs_msgs::Mouse mouse_ = hcs_msgs::Mouse::zero();
    hcs_msgs::Keyboard keyboard_ = hcs_msgs::Keyboard::zero();

    double rotary_knob_ = 0.0;
    hcs_msgs::Switch rotary_knob_switch_ = hcs_msgs::Switch::UNKNOWN;
};

} // namespace hcs_core::hardware::device
