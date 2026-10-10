#include "hcs_executor/component.hpp"

namespace hcs_executor::detail {

/// 改造前这里是 `std::string Component::initializing_component_name;` —— 一个静态全局。
/// 换成 thread_local 之后，"谁正在被构造"这件事不再跨线程共享：
/// 单测里两个 fixture 并行、或者两个 Executor 各自建图，都不会互相顶掉名字。
/// 函数局部静态而不是命名空间变量，是为了让初始化次序与动态加载的 .so 无关
/// —— 组件是 pluginlib dlopen 进来的，它们会在本翻译单元之后才被构造。
std::string& pending_component_name() {
    thread_local std::string name;
    return name;
}

} // namespace hcs_executor::detail

namespace hcs_executor {

/// 函数局部静态：第一次有人要日志时才建（连同它的日志线程），进程退出时析构，
/// 析构里会把队列里剩下的写完。第一次调用发生在主线程构造第一个组件 / Executor 的时候，
/// 远在控制线程启动之前——"建线程"这件事不会落到周期域里。
hcs_log::Backend& process_log_backend() {
    static hcs_log::Backend backend;
    return backend;
}

} // namespace hcs_executor
