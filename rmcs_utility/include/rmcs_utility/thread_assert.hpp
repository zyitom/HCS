#pragma once

#include <cstdint>

#ifndef NDEBUG
#  include <sys/syscall.h>
#  include <unistd.h>

#  include <atomic>
#  include <cstdio>
#  include <cstdlib>
#else
#  include <type_traits>
#endif

namespace rmcs_utility {

/// 断言"这个对象只被那条线程碰"。
///
/// 为什么要有：分域（周期域 / 事件域 / 尽力域）今天只活在注释里，而注释拦不住任何人——
/// 一次跨域直接访问带来的数据竞争，往往几万拍才发作一次，事后从现场根本读不出来。
/// 把归属做成一个字段，debug 构建下第一次越界访问就当场 abort，"分域"于是从约定变成事实。
/// release 构建下它是真正的空类、三个函数是空实现，热路径上一条指令都不留。
///
/// 用法上有一点：把成员声明成 `[[no_unique_address]] ThreadOwnership owner_;`，
/// release 下连那 1 字节和随之而来的对齐填充都不会占——空类的 sizeof 仍然是 1。
///
/// 注意：类布局随 NDEBUG 变，所以整个工程必须用同一套 NDEBUG 编译（colcon 的统一 build type
/// 已经保证）。混编会直接导致 ODR 违规。
#ifdef NDEBUG

class ThreadOwnership {
public:
    void claim() noexcept {}
    void assert_owned() const noexcept {}
    void release() noexcept {}
};

static_assert(std::is_empty_v<ThreadOwnership>, "release 构建下必须能被完全优化掉");

#else

class ThreadOwnership {
public:
    /// 由拥有者线程调一次。再调一次就是改判归属，用于线程交接。
    void claim() noexcept { owner_.store(current_tid(), std::memory_order_relaxed); }

    /// 非拥有者调用 → 打印并 abort。
    void assert_owned() const noexcept {
        const std::uint64_t owner = owner_.load(std::memory_order_relaxed);
        if (owner == 0)
            return; // 还没认领：对象尚未绑给任何线程，谈不上越界
        const std::uint64_t self = current_tid();
        if (owner == self) [[likely]]
            return;

        // 不用 iostream / std::format：违规现场很可能就在 RT 线程上，而且下一句就 abort，
        // 运行期依赖越少，这行字越有可能真的被打出来。
        std::fprintf(
            stderr,
            "[rmcs_utility] ThreadOwnership violation: object owned by tid %llu, "
            "touched by tid %llu\n",
            static_cast<unsigned long long>(owner), static_cast<unsigned long long>(self));
        std::fflush(stderr);
        std::abort();
    }

    void release() noexcept { owner_.store(0, std::memory_order_relaxed); }

private:
    /// 用 syscall(SYS_gettid) 而不是 pthread_self()：tid 和 /proc、perf、gdb 里看到的号一致，
    /// 报出来的数字能直接拿去查是哪条线程。
    [[nodiscard]] static std::uint64_t current_tid() noexcept {
        return static_cast<std::uint64_t>(::syscall(SYS_gettid));
    }

    /// gettid，0 = 未认领。原子只是为了让"认领"与"检查"之间不构成数据竞争本身，
    /// 不承担任何同步语义，所以一律 relaxed。
    std::atomic<std::uint64_t> owner_{0};
};

#endif

} // namespace rmcs_utility

// 参数在 release 下不求值，但仍需被"提及"一次：否则只在断言里用到的
// ThreadOwnership 成员会在 clang 上触发 -Wunused-private-field。
#ifndef NDEBUG
#  define RMCS_ASSERT_THREAD(ownership) (ownership).assert_owned()
#else
#  define RMCS_ASSERT_THREAD(ownership) ((void)sizeof(ownership), (void)0)
#endif
