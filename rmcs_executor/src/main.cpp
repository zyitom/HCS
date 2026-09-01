#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <execinfo.h>
#include <unistd.h>

#include <regex>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>

#include "executor.hpp"
#include "rmcs_executor/component.hpp"
#include "rmcs_utility/thread_config.hpp"

void segmentation_fault_handler(int) {
    void* array[100];

    int size = backtrace(array, 100);
    fprintf(stderr, "[Fatal] Segmentation fault\n>>> STACK TRACE BEGIN\n");

    // Print the stack trace to stderr.
    if (size >= 2)
        // Remove the stack trace used to call this function.
        backtrace_symbols_fd(array + 2, size - 2, STDERR_FILENO);
    else
        backtrace_symbols_fd(array, size, STDERR_FILENO);

    fprintf(stderr, "<<< STACK TRACE END\n");

    exit(1);
}

int main(int argc, char** argv) {
    std::signal(SIGSEGV, segmentation_fault_handler);

    rclcpp::init(argc, argv);

    pluginlib::ClassLoader<rmcs_executor::Component> component_loader(
        "rmcs_executor", "rmcs_executor::Component");

    rclcpp::executors::SingleThreadedExecutor rcl_executor;
    auto executor = std::make_shared<rmcs_executor::Executor>("rmcs_executor", rcl_executor);
    rcl_executor.add_node(executor);

    std::vector<std::string> component_descriptions;
    executor->get_parameter("components", component_descriptions);

    std::regex regex(R"(\s*(\S+)\s*->\s*(\S+)\s*)");
    for (const auto& component_description : component_descriptions) {
        std::smatch matches;
        std::string plugin_name, component_name;

        if (std::regex_search(component_description, matches, regex)) {
            if (matches.size() != 3)
                throw std::runtime_error("In regex matching: unexpected number of matches");

            plugin_name = matches[1].str();
            component_name = matches[2].str();
        } else {
            plugin_name = component_name = component_description;
        }

        // pluginlib 只能默认构造组件，名字只能靠这个 RAII 作用域递进去；
        // 出了作用域自动还原，不再留一个写完不擦的全局。
        auto component = [&] {
            auto scope = rmcs_executor::Component::NameScope{component_name};
            return component_loader.createSharedInstance(plugin_name);
        }();
        executor->add_component(component);
    }

    executor->start();

    // 纯用户态的核隔离：把 spin 线程按 "除 RT 核以外的全部核" 绑一遍，不用 isolcpus。
    // 这是尽力域 —— 绑不上只警告，不该因此把整机拉下来。
    std::string spin_thread_config_spec;
    executor->get_parameter_or<std::string>("spin_thread_config", spin_thread_config_spec, "");
    if (!spin_thread_config_spec.empty()) {
        try {
            const auto spin_thread_config =
                rmcs_utility::ThreadConfig{spin_thread_config_spec, "rmcs-spin"};
            if (const auto result = spin_thread_config.apply_to_current_thread(); !result)
                RCLCPP_WARN(
                    executor->get_logger(), "Failed to apply spin_thread_config: %s",
                    result.error().c_str());
        } catch (const std::exception& exception) {
            RCLCPP_WARN(
                executor->get_logger(), "Invalid spin_thread_config: %s", exception.what());
        }
    }

    rcl_executor.spin();

    rclcpp::shutdown();
}
