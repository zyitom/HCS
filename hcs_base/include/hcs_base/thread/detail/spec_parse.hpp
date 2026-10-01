#pragma once

// key=value;key=value 配置串的公共解析底盘。
//
// ThreadConfig 与 RealtimeArmOptions 曾经各自手写一遍:同样的 trim、同样的分号
// 切分、同样的"缺 '=' / 空键 / 空值"报错、各一套重复键标记。语法归这里,
// 语义(每个键是什么意思、值怎么校验)留在各自类里 —— 这条界线也是两个类
// 恰好不重合的部分。

#include <charconv>
#include <concepts>
#include <format>
#include <optional>
#include <stdexcept>
#include <string_view>

namespace hcs_utility::detail {

inline std::string_view trim(std::string_view text) noexcept {
    while (!text.empty() && (text.front() == ' ' || text.front() == '\t'))
        text.remove_prefix(1);
    while (!text.empty() && (text.back() == ' ' || text.back() == '\t'))
        text.remove_suffix(1);
    return text;
}

/// 统一的规格串报错格式:`Invalid <what> spec (<reason>): "<spec>"`。
/// spec 原样透传 —— 用原始串还是修剪过的串报警,由调用方决定。
[[noreturn]] inline void
throw_invalid_spec(std::string_view what, std::string_view reason, std::string_view spec) {
    throw std::invalid_argument(std::format("Invalid {} spec ({}): \"{}\"", what, reason, spec));
}

/// 逐字段回调 `on_field(key, value)`。键值两侧空白已去,空字段(`;;`)跳过。
/// 语法错误(缺 '='、空键、空值)统一在此抛 std::invalid_argument。
template <typename F>
    requires std::invocable<F&, std::string_view, std::string_view>
void for_each_kv_field(std::string_view what, std::string_view spec, F&& on_field) {
    auto remaining = spec;
    while (true) {
        const auto separator = remaining.find(';');
        const auto field = trim(remaining.substr(0, separator));

        if (!field.empty()) {
            const auto equals = field.find('=');
            if (equals == std::string_view::npos)
                throw_invalid_spec(what, "missing '='", spec);

            const auto key = trim(field.substr(0, equals));
            const auto value = trim(field.substr(equals + 1));
            if (key.empty() || value.empty())
                throw_invalid_spec(what, "empty key or value", spec);

            on_field(key, value);
        }

        if (separator == std::string_view::npos)
            break;
        remaining.remove_prefix(separator + 1);
    }
}

/// std::from_chars 的平凡封装:整串完整解析才返回值,否则 nullopt。
/// 报错的措辞归调用方 —— 那里有键名和规格串的上下文。
template <typename T>
    requires std::integral<T>
std::optional<T> parse_integer(std::string_view text) noexcept {
    T result{};
    const auto* begin = text.data();
    const auto* end = text.data() + text.size();
    const auto [ptr, error_code] = std::from_chars(begin, end, result);
    if (error_code != std::errc{} || ptr != end)
        return std::nullopt;
    return result;
}

} // namespace hcs_utility::detail
