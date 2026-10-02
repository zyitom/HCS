#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <execinfo.h>
#include <sched.h>
#include <sys/prctl.h>
#include <unistd.h>

#include <chrono>
#include <exception>
#include <regex>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/executors.hpp>

#include "executor.hpp"
#include "rclcpp_log_sink.hpp"
#include "hcs_executor/component.hpp"
#include "hcs_base/thread/thread_config.hpp"

void segmentation_fault_handler(int) {
    void* array[100];

    int size = backtrace(array, 100);
    fprintf(stderr, "[Fatal] Segmentation fault\n>>> STACK TRACE BEGIN\n");

    // 把栈回溯打到 stderr。
    if (size >= 2)
        // 去掉调用这个函数本身占的那几层栈。
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
/// 用法：HCS_NON_RT_CPUS="0-2,4-19" hcs_executor ...
static void apply_process_affinity_from_environment() {
    const char* spec = std::getenv("HCS_NON_RT_CPUS");
    if (spec == nullptr || *spec == '\0')
        return;

    try {
        // 借 ThreadConfig 的 cpus= 解析器，语法和 thread_config 完全一致。
        const auto config = hcs_utility::ThreadConfig{std::string{"cpus="} + spec};
        if (!config.cpus())
            return;
        if (sched_setaffinity(0, sizeof(cpu_set_t), &*config.cpus()) != 0)
            fprintf(
                stderr, "[warn] HCS_NON_RT_CPUS=%s: sched_setaffinity failed: %s\n", spec,
                std::strerror(errno));
    } catch (const std::exception& exception) {
        fprintf(stderr, "[warn] HCS_NON_RT_CPUS=%s is invalid: %s\n", spec, exception.what());
    }
}

/// 允许外部调试工具（cescan/gdb/自研 reader）对本进程做 process_vm_readv/writev。
///
/// yama ptrace_scope=1（Ubuntu 默认）下，非后代的进程一律 EPERM，除非被跟踪方
/// 自己 prctl(PR_SET_PTRACER) 点头。只放开本进程、必须显式设环境变量才生效，
/// 不影响正常部署。
///
/// 用法：HCS_ALLOW_DEBUG_ATTACH=1 ros2 launch hcs_bringup hcs.launch.py robot:=demo
static void apply_debug_ptracer_grant_from_environment() {
    const char* spec = std::getenv("HCS_ALLOW_DEBUG_ATTACH");
    if (spec == nullptr || *spec == '\0' || *spec == '0')
        return;

    if (prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY, 0, 0, 0) != 0)
        fprintf(stderr, "[warn] PR_SET_PTRACER failed: %s\n", std::strerror(errno));
}

/// 未捕获的异常会走到 std::terminate，那里不做栈展开，也不跑静态析构：
/// 还在队列里的日志——多半正是解释"为什么起不来"的那几行——会跟着进程一起消失。
/// 所以先把队列排空，再交回原来的处理函数（它负责打印异常的 what() 并 abort）。
///
/// 只等一小会儿：输出端堵着（stderr 那头的管道没人读）的话，这几行日志就不要了。
/// 进程必须死得掉——它不死，USB 不断，板子不会进失联保护，电机停在最后一条指令上。
static std::terminate_handler previous_terminate_handler = nullptr;

static void flush_logs_then_terminate() {
    // 兜底，先设好再做别的。排空日志有超时，但原来的处理函数往 stderr 打 what() 是一次
    // 没有超时的 fputs：实测 stderr 接一根写满了没人读的管道时，进程就挂在那一句上，
    // SIGTERM 也收不走它（rclcpp 的信号处理只置一个标志，等 spin 去看）。
    // SIGALRM 没有人接管，默认动作就是结束进程——两秒后内核无条件送到。
    ::alarm(2);

    (void)hcs_executor::process_log_backend().flush_for(std::chrono::milliseconds{500});
    if (previous_terminate_handler != nullptr)
        previous_terminate_handler();
    std::abort();
}

int main(int argc, char** argv) {
    std::signal(SIGSEGV, segmentation_fault_handler);
    previous_terminate_handler = std::set_terminate(flush_logs_then_terminate);

    // 在 init 之前 —— 见上面那段注释。
    apply_process_affinity_from_environment();
    apply_debug_ptracer_grant_from_environment();

    rclcpp::init(argc, argv);

    pluginlib::ClassLoader<hcs_executor::Component> component_loader(
        "hcs_executor", "hcs_executor::Component");

    // 从这里起日志走 rclcpp。声明在 component_loader 之后是有意的：它先析构，
    // 在组件库被卸载之前把日志队列排空（见 RclcppLogScope）。
    hcs_executor::RclcppLogScope rclcpp_log_scope{hcs_executor::process_log_backend()};
    const hcs_log::Logger log{hcs_executor::process_log_backend(), "hcs_executor"};

    rclcpp::executors::SingleThreadedExecutor rcl_executor;
    auto executor = std::make_shared<hcs_executor::Executor>("hcs_executor", rcl_executor);
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
            auto scope = hcs_executor::Component::NameScope{component_name};
            return component_loader.createSharedInstance(plugin_name);
        }();
        executor->add_component(component);
    }

    // 组件到齐了，哪些名字背后是节点也就定了：不是节点的那些从这里起挂到 executor 的
    // logger 下面，才上得了 /rosout（见 RclcppSink）。
    rclcpp_log_scope.attach(executor->get_logger(), executor->node_loggers());

    executor->start();

    // spin 线程自己的亲和/优先级。注意它**只管这一条线程** —— DDS 那十几条早在建节点时
    // 就生出去了，管它们的是上面 HCS_NON_RT_CPUS 那一步（或者内核的 isolcpus）。
    // 这是尽力域 —— 绑不上只警告，不该因此把整机拉下来。
    std::string spin_thread_config_spec;
    executor->get_parameter_or<std::string>("spin_thread_config", spin_thread_config_spec, "");
    if (!spin_thread_config_spec.empty()) {
        try {
            const auto spin_thread_config =
                hcs_utility::ThreadConfig{spin_thread_config_spec, "hcs-spin"};
            if (const auto result = spin_thread_config.apply_to_current_thread(); !result)
                log.warn("Failed to apply spin_thread_config: {}", result.error());
        } catch (const std::exception& exception) {
            log.warn("Invalid spin_thread_config: {}", exception.what());
        }
    }

    rcl_executor.spin();

    // 关 rclcpp 之前把日志的输出端换回 stderr：关掉之后再走 rclcpp 的日志没人保证。
    rclcpp_log_scope.restore();
    rclcpp::shutdown();
}
