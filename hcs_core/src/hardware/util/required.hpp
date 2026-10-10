#pragma once

#include <concepts>
#include <utility>

namespace hcs_core::hardware::util {

/// 没有默认值的配置字段：初始化时漏写它就编不过。
///
/// 配置都是聚合类型，这样接线表可以把它设的每个字段都写出名字
/// （`{.esc_id = 0x02, .master_id = 0x06, ...}`）。聚合里被漏掉的成员是用 `{}` 初始化的，
/// 而这个类型没有默认构造函数，所以漏写一个必填字段是编译错误，而不是一个悄悄的 0。
/// 按位置初始化（`Config{mode, 0x02, 0x06, range}`）照样能用。
template <class T>
class Required {
public:
    constexpr Required(T value) // NOLINT(google-explicit-constructor)：要的就是能写 `.x = v`
        : value_(std::move(value)) {}

    /// T 收什么它也收什么，所以 `.mit_range = {6.28, 45.0, 40.0}` 和 `.id = 1` 都能写。
    template <class... Args>
        requires(sizeof...(Args) > 0) && std::constructible_from<T, Args...>
    constexpr Required(Args&&... args) // NOLINT(google-explicit-constructor)
        : value_(std::forward<Args>(args)...) {}

    constexpr Required& operator=(T value) {
        value_ = std::move(value);
        return *this;
    }

    constexpr operator const T&() const noexcept { return value_; } // NOLINT
    constexpr const T& get() const noexcept { return value_; }
    constexpr const T* operator->() const noexcept { return &value_; }

private:
    T value_;
};

} // namespace hcs_core::hardware::util
