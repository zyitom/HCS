#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <optional>
#include <span>
#include <string>

#include <hcs_executor/component.hpp>
#include <hcs_msgs/serial_interface.hpp>

#include "hardware/device/serial_line.hpp"
#include "hardware/util/required.hpp"

namespace hcs_core::hardware::device {

/// 透传串口：这路串口上走的协议不在硬件层解，字节原样交给图里别的组件，两个方向都是。
///
/// 输出 <name>（hcs_msgs::SerialInterface）：
///   read(buffer, size)   取走收到的字节，按到达顺序，最多 size 个；返回取走的字节数
///   write(buffer, size)  把一整段排到待发字节后面：放得下返回 size，放不下一个字节都不收、
///                        返回 0——一次 write() 交一整帧，帧就不会被截断
/// 两个都只许在周期域（组件的 update()）里调，都不阻塞、不分配。写进去的字节在拍尾交给这路
/// 串口：写的组件排在板组件的指令伙伴之前就是这一拍，之后就是下一拍。
///
/// 现在的用户是裁判系统——referee::Status 读、referee::Command 写，都接在 /referee/serial 上：
///
///     SerialStream referee{uart1, "/referee/serial", {.baudrate = 115200}};
///
/// "在线"在这里只能是"线上有字节来"：帧对不对，要读的那一方才知道。
///
/// 不看 safe：这条线上走的是给别的系统看的数据，不是让东西动的指令，锁存期间照发。
/// 会动的串口设备不该接在它上面，该有自己的驱动，由那个驱动决定失能时发什么。
///
/// 驱动"只写协议"在这里退化成没有协议：全部成员都在控制线程上，两个方向各一个普通的环形
/// 缓冲；跨线程的交接在端口包装层（board::Serial<SerialStream>）。
class SerialStream {
public:
    /// 每个方向能攒下的字节数。裁判系统最长的帧约 1 KB。
    static constexpr std::size_t kBufferBytes = 2048;

    struct Config {
        /// 对端的串口速率。必填：它是对端的属性（裁判系统电源管理模块是 115200），8N1。
        util::Required<std::uint32_t> baudrate;
        /// 连续多久一个字节都没收到算掉线。
        std::chrono::milliseconds offline_timeout{500};
    };

    SerialStream(
        hcs_executor::Component& status_component, const std::string& name, const Config& config)
        : baudrate_(config.baudrate) {
        status_component.register_output(name, interface_);
        interface_->read = [this](std::byte* buffer, std::size_t size) {
            return read({buffer, size});
        };
        interface_->write = [this](const std::byte* buffer, std::size_t size) {
            return write({buffer, size}) ? size : std::size_t{0};
        };
    }

    SerialStream(const SerialStream&) = delete;
    SerialStream& operator=(const SerialStream&) = delete;

    [[nodiscard]] SerialLine serial_line() const noexcept { return {.baudrate = baudrate_}; }

    /// 周期域：一段收到的字节，留着等人来读。没人读、放不下的丢弃并计数（见 take_problem()）。
    /// @return 这一段里有没有字节
    bool on_bytes(std::span<const std::byte> bytes) noexcept {
        const auto kept = received_.push(bytes);
        if (kept < bytes.size()) [[unlikely]]
            unread_bytes_.fetch_add(bytes.size() - kept, std::memory_order_relaxed);
        return !bytes.empty();
    }

    /// 周期域：取走收到的字节，最多 out.size() 个。@return 取走的字节数
    std::size_t read(std::span<std::byte> out) noexcept { return received_.pop(out); }

    /// 周期域：把一整段排到待发字节后面，或者一个字节都不收（并计数）。@return 收下了没有
    bool write(std::span<const std::byte> bytes) noexcept {
        if (bytes.size() > pending_.free()) [[unlikely]] {
            refused_bytes_.fetch_add(bytes.size(), std::memory_order_relaxed);
            return false;
        }
        pending_.push(bytes);
        return true;
    }

    /// 周期域，拍尾：待发的字节按序交给这路串口。出口这一拍收不下的留到下一拍，不丢。
    template <class Writer>
    void append_command(Writer& out, bool /*safe*/) noexcept {
        std::array<std::byte, 256> chunk;
        for (;;) {
            const auto count =
                pending_.pop(std::span{chunk}.first(std::min(chunk.size(), out.writable())));
            if (count == 0)
                return;
            out.write(std::span<const std::byte>{chunk}.first(count));
        }
    }

    [[nodiscard]] std::string describe() const {
        return std::format("serial stream at {} baud, passed through untouched", baudrate_);
    }

    /// 尽力域：自上次调用以来，有没有字节因为缓冲放不下而没过去。一次报一个方向。
    std::optional<std::string> take_problem() {
        if (const auto unread = unread_bytes_.load(std::memory_order_relaxed);
            unread != reported_unread_) {
            const auto more = unread - reported_unread_;
            reported_unread_ = unread;
            return std::format(
                "{} more received byte(s) dropped unread, {} in total; no component is reading "
                "this stream fast enough",
                more, unread);
        }
        if (const auto refused = refused_bytes_.load(std::memory_order_relaxed);
            refused != reported_refused_) {
            const auto more = refused - reported_refused_;
            reported_refused_ = refused;
            return std::format(
                "{} more byte(s) to transmit refused, {} in total; the board is not sending them "
                "as fast as they are written",
                more, refused);
        }
        return std::nullopt;
    }

private:
    /// 普通的环形字节缓冲：读写都在控制线程上，所以没有 atomic。
    class Fifo {
    public:
        [[nodiscard]] std::size_t free() const noexcept { return kBufferBytes - (head_ - tail_); }

        /// 尽量多写。@return 写进去的字节数
        std::size_t push(std::span<const std::byte> data) noexcept {
            const std::size_t count = std::min(free(), data.size());
            if (count == 0)
                return 0;
            const std::size_t offset = head_ & kIndexMask;
            const std::size_t first = std::min(count, kBufferBytes - offset);
            std::memcpy(storage_.data() + offset, data.data(), first);
            std::memcpy(storage_.data(), data.data() + first, count - first);
            head_ += count;
            return count;
        }

        /// 尽量多读。@return 读出的字节数
        std::size_t pop(std::span<std::byte> out) noexcept {
            const std::size_t count = std::min(head_ - tail_, out.size());
            if (count == 0)
                return 0;
            const std::size_t offset = tail_ & kIndexMask;
            const std::size_t first = std::min(count, kBufferBytes - offset);
            std::memcpy(out.data(), storage_.data() + offset, first);
            std::memcpy(out.data() + first, storage_.data(), count - first);
            tail_ += count;
            return count;
        }

    private:
        static constexpr std::size_t kIndexMask = kBufferBytes - 1;

        std::array<std::byte, kBufferBytes> storage_;
        std::size_t head_ = 0; ///< 写到哪了（不回绕，取用时再取模）
        std::size_t tail_ = 0; ///< 读到哪了
    };
    static_assert(std::has_single_bit(kBufferBytes));

    std::uint32_t baudrate_;
    Fifo received_; ///< 收到、还没被读走的
    Fifo pending_;  ///< 写进来、还没交给串口的

    std::atomic<std::uint64_t> unread_bytes_{0};  ///< 周期域写，尽力域读
    std::atomic<std::uint64_t> refused_bytes_{0}; ///< 周期域写，尽力域读
    std::uint64_t reported_unread_ = 0;           ///< 只在尽力域碰
    std::uint64_t reported_refused_ = 0;          ///< 只在尽力域碰

    hcs_executor::Component::OutputInterface<hcs_msgs::SerialInterface> interface_;
};

} // namespace hcs_core::hardware::device
