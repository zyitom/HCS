#pragma once

#include <concepts>
#include <utility>

namespace hcs_core::hardware::util {

/// A config field with no default: leaving it out of an initializer does not compile.
///
/// Configs are aggregates so a wiring table can name every field it sets
/// (`{.esc_id = 0x02, .master_id = 0x06, ...}`). An aggregate member that is left out is
/// initialized from `{}`, and this type has no default constructor, so forgetting a required
/// field is a compile error instead of a quiet zero. Positional initialization
/// (`Config{mode, 0x02, 0x06, range}`) keeps working.
template <class T>
class Required {
public:
    constexpr Required(T value) // NOLINT(google-explicit-constructor): the point is `.x = v`
        : value_(std::move(value)) {}

    /// Also accepts what T accepts, so `.mit_range = {6.28, 45.0, 40.0}` and `.id = 1` work.
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
