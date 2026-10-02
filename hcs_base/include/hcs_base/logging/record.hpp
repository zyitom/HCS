#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>

#include "hcs_base/thread/rt_attributes.hpp"

namespace hcs_log {

enum class Level : std::uint8_t { kDebug, kInfo, kWarn, kError, kFatal };

[[nodiscard]] constexpr std::string_view to_string(Level level) noexcept {
    switch (level) {
    case Level::kDebug: return "DEBUG";
    case Level::kInfo: return "INFO";
    case Level::kWarn: return "WARN";
    case Level::kError: return "ERROR";
    case Level::kFatal: return "FATAL";
    }
    return "?";
}

/// 一条日志在队列里的样子：定长、平凡可拷贝，所以入队就是一次内存拷贝，
/// 没有析构、没有分配，哪条线程都放得起。
///
/// 载荷有三种：
///   - 文本（render == nullptr，spilled == false）：就放在载荷里。尽力域当场格式化好的；
///     或者周期域手上现成的一段（Logger::Realtime::write），那边装不下是截断而不是溢出。
///   - 溢出的文本（spilled == true）：文本比载荷长，放在堆上，载荷里只有一个 Spill。
///     只有尽力域会产生——那里本来就允许分配——所以尽力域的日志**不截断**。
///     这块内存的所有权随记录走：入队成功就归日志线程，由它写完后释放。
///   - 延迟格式化（render != nullptr）：载荷是实参的原始字节，文本要等日志线程调 render
///     才生成。周期域走这条：调用点只做几次 memcpy，格式化整个挪到了日志线程上。
///
/// ⚠ 第二种记录里存着**代码和只读数据的地址**（render、格式串字面量），它们属于发出这条
///   日志的那个动态库。库被卸载之前必须先把队列排空（Backend::flush()），否则日志线程会
///   去调一个已经不存在的函数。hcs_executor 的 main 在卸组件库之前做了这件事。
struct Record {
    /// 把 (格式串, 实参字节) 还原成文本。每一种实参类型组合对应一个实例，见 Deferred。
    using Render = std::string (*)(std::string_view format, const std::byte* arguments);

    static constexpr std::size_t kSize = 512;
    static constexpr std::size_t kNameCapacity = 40;
    static constexpr std::size_t kPayloadCapacity = 442;

    Render render = nullptr;
    std::string_view format{}; ///< 只在 render != nullptr 时有意义；指向字符串字面量
    Level level = Level::kInfo;
    bool spilled = false;          ///< 载荷是一个 Spill，文本在堆上
    std::uint8_t name_size = 0;
    std::uint16_t payload_size = 0;
    std::array<char, kNameCapacity> name{};
    std::array<std::byte, kPayloadCapacity> payload{};

    [[nodiscard]] std::string_view logger_name() const noexcept {
        return {name.data(), name_size};
    }

    /// 仅当 render == nullptr 且 !spilled。
    [[nodiscard]] std::string_view text() const noexcept {
        return {reinterpret_cast<const char*>(payload.data()), payload_size};
    }

    /// 堆上那段文本，new char[] 分配。按字节存进 / 取出载荷（载荷没有对齐保证）。
    struct Spill {
        char* data;
        std::size_t size;
    };

    void set_spill(Spill spill) noexcept {
        spilled = true;
        std::memcpy(payload.data(), &spill, sizeof(spill));
    }

    /// 仅当 spilled。
    [[nodiscard]] Spill spill() const noexcept {
        Spill spill{};
        std::memcpy(&spill, payload.data(), sizeof(spill));
        return spill;
    }
};
static_assert(sizeof(Record) == Record::kSize, "调整字段后记得把 kPayloadCapacity 配平");
static_assert(std::is_trivially_copyable_v<Record>);

/// 能延迟格式化的实参：按字节拷走、晚些时候在另一条线程上再格式化，语义不变。
///
/// 一切"指向别处"的东西都被排除在外：指针（含 const char*）、std::string_view、字符数组。
/// 它们指向的内容到日志线程去读的时候可能已经没了。std::string 不是平凡可拷贝，天然进不来。
template <typename T>
concept DeferredArgument =
    std::is_trivially_copyable_v<T> && std::default_initializable<T> && !std::is_pointer_v<T>
    && !std::convertible_to<T, std::string_view> && std::formattable<T, char>;

/// 周期域入口用的格式串：和 std::format_string 一样在**编译期**按实参类型检查，
/// 只是自己这一层标了 nonblocking。
///
/// 不能直接用 std::format_string：它那个 consteval 构造函数没有 nonblocking 标注，
/// clang 的效果分析会在每一个周期域调用点上报"调用了非 nonblocking 的构造函数"——
/// 尽管那次调用整个发生在编译期，运行时一条指令都没有。
///
/// 只接受字符串字面量（或别的静态存储期字符数组）：这里存的是它的地址，
/// 要等到日志线程才去读。
template <typename... Args>
class BasicDeferredFormat {
public:
    template <typename Text>
    requires std::convertible_to<const Text&, std::string_view>
    consteval BasicDeferredFormat(const Text& text) noexcept HCS_NONBLOCKING // NOLINT: 隐式转换是本意
        : text_(text) {
#if defined(__clang__)
#    pragma clang diagnostic push
#    pragma clang diagnostic ignored "-Wfunction-effects"
#endif
        // 借标准库做检查，结果不要。这一行只在编译期求值。
        (void)std::format_string<Args...>{text};
#if defined(__clang__)
#    pragma clang diagnostic pop
#endif
    }

    [[nodiscard]] constexpr std::string_view get() const noexcept HCS_NONBLOCKING {
        return text_;
    }

private:
    std::string_view text_;
};

/// type_identity 挡住从格式串推导 Args：Args 只由后面的实参决定。
template <typename... Args>
using DeferredFormat = BasicDeferredFormat<std::type_identity_t<Args>...>;

/// 一组实参的"按字节装箱 / 拆箱再格式化"。实参类型在调用点就定了，render 的地址随之确定，
/// 所以 Record 里只需要存一个函数指针——日志线程不需要认识任何一种实参类型。
template <DeferredArgument... Args>
requires((sizeof(Args) + ... + std::size_t{0}) <= Record::kPayloadCapacity)
struct Deferred {
    static constexpr std::size_t kSize = (sizeof(Args) + ... + std::size_t{0});

    /// 调用点（任何域）。只有 memcpy。
    static void pack(std::byte* out, const Args&... args) noexcept {
        std::size_t offset = 0;
        ((std::memcpy(out + offset, &args, sizeof(Args)), offset += sizeof(Args)), ...);
        (void)offset; // 无实参时
    }

    /// 日志线程。格式串在调用点已经过编译期检查，这里不会因为格式不匹配而抛。
    [[nodiscard]] static std::string render(std::string_view format, const std::byte* in) {
        std::tuple<Args...> values;
        std::size_t offset = 0;
        std::apply(
            [&](Args&... value) {
                ((std::memcpy(&value, in + offset, sizeof(Args)), offset += sizeof(Args)), ...);
            },
            values);
        (void)offset;
        return std::apply(
            [&](const Args&... value) {
                return std::vformat(format, std::make_format_args(value...));
            },
            values);
    }
};

} // namespace hcs_log
