#pragma once

#include <format>
#include <stdexcept>
#include <string>

#include <hcs_executor/component.hpp>
#include <rclcpp/node.hpp>

namespace hcs_core::controller::pid {

// 移植自 RMCS 的 controller/pid/smart_input.hpp：
// 把「参数可以是常数也可以是接口名」的输入抽象统一。
// 参数值是 DOUBLE 时直接绑常数，不走配对；是 STRING 时按接口名注册输入，
// 名字首字符为 '-' 时去掉负号注册并把读到的值取反。
class SmartInput {
public:
    template <typename T>
    requires std::is_base_of_v<hcs_executor::Component, T> && std::is_base_of_v<rclcpp::Node, T>
    explicit SmartInput(T& component, const std::string& name) {
        if (!component.has_parameter(name))
            throw std::runtime_error(
                std::format(
                    "Parameter '{}' was not found in component '{}'", name,
                    component.get_component_name()));

        register_input(component, component.get_parameter(name));
    }

    template <typename T>
    requires std::is_base_of_v<hcs_executor::Component, T> && std::is_base_of_v<rclcpp::Node, T>
    explicit SmartInput(T& component, const std::string& name, double default_value) {
        if (!component.has_parameter(name)) {
            input_.make_and_bind_directly(default_value);
            return;
        }

        register_input(component, component.get_parameter(name));
    }

    double operator*() const { return is_negative_ ? -*input_ : *input_; }

private:
    void register_input(hcs_executor::Component& component, const rclcpp::Parameter& value) {
        if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            input_.make_and_bind_directly(value.as_double());
        } else {
            const auto& input_name = value.as_string();
            if (input_name[0] == '-') {
                is_negative_ = true;
                component.register_input(input_name.substr(1), input_);
            } else
                component.register_input(input_name, input_);
        }
    }

    hcs_executor::Component::InputInterface<double> input_;
    bool is_negative_ = false;
};

} // namespace hcs_core::controller::pid
