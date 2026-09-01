#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <execinfo.h>
#include <sched.h>
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

/// 进程级的"纯用户态核隔离"，必须在 rclcpp::init() 之前做。
///
/// 为什么不能用 ROS 参数：参数要先建节点才读得到，而建节点就会建 DDS participant，
/// 十几条 rmw/DDS 线程在那一刻就已经生出去了 —— 事后改主线程的亲和追不上它们。
/// 实测一个跑着的 executor 有 17 条线程，spin_thread_config 只管得住其中 1 条。
/// 这里设的是**进程**的亲和，之后创建的每一条线程都继承它，控制线程再单独绑回 RT 核。
///
/// 装了 isolcpus 的机器（TL101 是 isolcpus=7）不需要这个：内核已经把隔离核从默认掩码里
/// 摘走了，所有线程天生就不在上面。这条是给没有 isolcpus 的机器用的。
///
/// 用法：RMCS_NON_RT_CPUS="0-2,4-19" rmcs_executor ...
static void apply_process_affinity_from_environment() {
    const char* spec = std::getenv("RMCS_NON_RT_CPUS");
    if (spec == nullptr || *spec == '\0')
        return;

    try {
        // 借 ThreadConfig 的 cpus= 解析器，语法和 thread_config 完全一致。
        const auto config = rmcs_utility::ThreadConfig{std::string{"cpus="} + spec};
        if (!config.cpus())
            return;
        if (sched_setaffinity(0, sizeof(cpu_set_t), &*config.cpus()) != 0)
            fprintf(
                stderr, "[warn] RMCS_NON_RT_CPUS=%s: sched_setaffinity failed: %s\n", spec,
                std::strerror(errno));
    } catch (const std::exception& exception) {
        fprintf(stderr, "[warn] RMCS_NON_RT_CPUS=%s is invalid: %s\n", spec, exception.what());
    }
}

int main(int argc, char** argv) {
    std::signal(SIGSEGV, segmentation_fault_handler);

    // 在 init 之前 —— 见上面那段注释。
    apply_process_affinity_from_environment();

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

    // spin 线程自己的亲和/优先级。注意它**只管这一条线程** —— DDS 那十几条早在建节点时
    // 就生出去了，管它们的是上面 RMCS_NON_RT_CPUS 那一步（或者内核的 isolcpus）。
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
