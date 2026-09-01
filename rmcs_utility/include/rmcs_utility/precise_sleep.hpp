#pragma once

#include <time.h>

#include <cerrno>
#include <chrono>

namespace rmcs_utility {

/// 单次 CPU 放松提示。
///
/// 忙等循环里必须有它：它告诉乱序核"这是自旋"，避免流水线塞满推测执行的 load 之后
/// 在退出循环时整条清空；在 SMT 上还会把发射槽让给同核的另一条超线程。
inline void cpu_relax() noexcept {
#if defined(__x86_64__) || defined(__i386__)
    __builtin_ia32_pause();
#elif defined(__aarch64__)
    __asm__ __volatile__("yield" ::: "memory");
#else
    // 其它架构没有对应指令。留空：忙等依然正确，只是更费电、对同核邻居更不客气。
#endif
}

// 下面要把绝对时刻拆成 timespec，前提是 steady_clock 的 tick 是"1/N 秒"这种形状。
// 另一个前提在 Linux 上天然成立：steady_clock 就是 CLOCK_MONOTONIC，两者 epoch 同源，
// 绝对唤醒时刻才有意义。
static_assert(
    std::chrono::steady_clock::period::num == 1,
    "steady_clock must tick in whole fractions of a second");

/// 睡到 deadline。clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME) + 可选的尾部忙等。
///
/// 为什么不用 std::this_thread::sleep_until：libstdc++ 会把它换算成**相对**的 nanosleep，
/// 于是多读一次表（读表到进内核之间的那段时间就是净误差），而且相对睡眠要吃内核给非 RT 线程的
/// 默认 timer slack（50us 量级）——对 1kHz 的拍来说这是致命的。绝对超时既没有换算误差，
/// 被信号打断后重试也天然幂等。
///
/// @param deadline    绝对唤醒时刻（steady_clock，在 Linux 上就是 CLOCK_MONOTONIC）。
///                    已经过点则立即返回，不进内核。
/// @param spin_guard  提前多久退出内核睡眠、转为忙等。0 表示完全不忙等。
///                    两条前提都成立才值得开：
///                      1. 每拍最多烧掉 spin_guard 那么长的**一整个核**。它和"把这条线程绑到
///                         一个独占核、把别人都挪走"是一对；共享核上开它就是纯粹抢别人的 CPU，
///                         端到端延迟只会更差。
///                      2. 先测再开。如果 cyclictest 显示唤醒抖动本来就在十几微秒以内，
///                         这条不值——省下的几微秒买不回一个核。
inline void sleep_until_precise(
    std::chrono::steady_clock::time_point deadline,
    std::chrono::nanoseconds spin_guard) noexcept {

    const auto now = std::chrono::steady_clock::now();
    if (deadline <= now)
        return; // 已经过点：再进一次内核只会更晚

    const bool spin = spin_guard > std::chrono::nanoseconds::zero();
    const auto wake = spin ? deadline - spin_guard : deadline;

    if (wake > now) {
        const auto ns_since_epoch =
            std::chrono::duration_cast<std::chrono::nanoseconds>(wake.time_since_epoch()).count();
        ::timespec target{};
        target.tv_sec = static_cast<::time_t>(ns_since_epoch / 1'000'000'000);
        target.tv_nsec = static_cast<long>(ns_since_epoch % 1'000'000'000);

        // EINTR 必须重试而不是返回：gdb 挂上来、profiler 的定时器、SIGCHLD 都会打断睡眠，
        // 此时返回就等于这一拍提前跑，节拍直接废掉。目标是绝对时刻，重试不累积漂移。
        // 其它错误（只可能是参数非法）无从补救，落到下面的忙等或直接返回。
        while (::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target, nullptr) == EINTR) {
        }
    }

    if (spin) {
        while (std::chrono::steady_clock::now() < deadline)
            cpu_relax();
    }
}

} // namespace rmcs_utility
