#pragma once

// 运行期的实时上下文标记。补 RMCS_NONBLOCKING 覆盖不到的那一块。
//
// 为什么需要它：`RMCS_NONBLOCKING`（[[clang::nonblocking]]）是**静态**约束，而且
// clang 的效果分析明确禁止在 nonblocking 函数里 catch 异常——异常对象要分配、
// unwinder 要拿全局锁，这判断本身是对的。
//
// 但 executor 的拍内主体恰恰**必须** catch：组件失效隔离（单点失效不拉整机）就是靠它。
// 于是那段代码——1/z 闩存、record_tick、时基推进——整块落在 RTSan 的覆盖之外。
// 这不是推测：往 execute_update_iteration 里塞一次 `malloc(64)`，
// RTSan 跑一万五千拍报告 `Total error count: 0`。组件的 update() 干净，
// 不代表调度器自己的拍内代码干净，而后者每拍都跑，还没人替你盯着。
//
// RealtimeScope 用的就是编译器给 nonblocking 函数生成的那对运行期调用
// （`__rtsan_realtime_enter` / `__rtsan_realtime_exit`，RTSan 运行时导出的 C 符号，
// 只是没进 sanitizer/rtsan_interface.h 那个公开头）。纯运行期、不做静态推断，
// 所以能和 try/catch 共存。深度是计数的，组件 update() 的 nonblocking 上下文
// 嵌套在里面没有问题。
//
// 不开 RTSan 时它是空类，一条指令都不留——包括生产构建（gcc）。

#include <type_traits>

#if defined(__has_feature)
#  if __has_feature(realtime_sanitizer)
#    define RMCS_HAS_RTSAN 1
#  endif
#endif

#ifndef RMCS_HAS_RTSAN
#  define RMCS_HAS_RTSAN 0
#endif

#if RMCS_HAS_RTSAN
extern "C" void __rtsan_realtime_enter(void);
extern "C" void __rtsan_realtime_exit(void);
#endif

namespace rmcs_utility {

class RealtimeScope {
public:
#if RMCS_HAS_RTSAN
    RealtimeScope() noexcept { __rtsan_realtime_enter(); }
    // 异常穿过时也要平衡地退出：失效隔离的 catch 在这个作用域**里面**，
    // 但整拍提前返回的路径（将来若有）会走这里。
    ~RealtimeScope() { __rtsan_realtime_exit(); }
#else
    RealtimeScope() noexcept = default;
    ~RealtimeScope() = default;
#endif

    RealtimeScope(const RealtimeScope&) = delete;
    RealtimeScope& operator=(const RealtimeScope&) = delete;
    RealtimeScope(RealtimeScope&&) = delete;
    RealtimeScope& operator=(RealtimeScope&&) = delete;
};

#if !RMCS_HAS_RTSAN
static_assert(std::is_empty_v<RealtimeScope>, "不开 RTSan 时必须能被完全优化掉");
#endif

} // namespace rmcs_utility
