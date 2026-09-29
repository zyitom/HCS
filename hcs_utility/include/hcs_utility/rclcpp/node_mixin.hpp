#pragma once
#include <format>
#include <stdexcept>
#include <rclcpp/node_options.hpp>

namespace hcs_utility {

struct NodeMixin {
    using node = NodeMixin;

    static constexpr auto options() noexcept {
        return rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true);
    }

    template <typename Self, typename... Args>
    auto info(this const Self& self, std::format_string<Args...> fmt, Args&&... args) -> void {
        auto text = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_INFO(self.get_logger(), "%s", text.c_str());
    }

    template <typename Self, typename... Args>
    auto warn(this const Self& self, std::format_string<Args...> fmt, Args&&... args) -> void {
        auto text = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_WARN(self.get_logger(), "%s", text.c_str());
    }

    template <typename Self, typename... Args>
    auto error(this const Self& self, std::format_string<Args...> fmt, Args&&... args) -> void {
        auto text = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_ERROR(self.get_logger(), "%s", text.c_str());
    }

    template <typename T>
    auto param(this const auto& self, const std::string& name, T& dst) {
        if (!self.has_parameter(name)) {
            self.error("param [ {} ] for {} is needed", name, self.get_name());
            throw std::runtime_error{"lack of param"};
        }

        // 必须走 get_value<T>:参数存在但类型不符时它会抛出来。
        // get_parameter_or 在类型不符时静默回退到传入的默认值 —— 配错类型的
        // 参数会无声消失,而拿到的 T{} 看起来完全正常。
        dst = self.get_parameter(name).get_value<T>();
    }
    template <typename T1, typename T2>
    auto param_or(this const auto& self, const std::string& name, T1& dst, const T2& fallback)
        requires std::convertible_to<T2, T1> {
        dst = self.template get_parameter_or<T1>(name, fallback);
    }

    template <typename T>
    auto param_or(this const auto& self, const std::string& name, const T& fallback) -> T {
        return self.template get_parameter_or<T>(name, fallback);
    }
};

} // namespace hcs_utility
