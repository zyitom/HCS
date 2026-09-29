#pragma once

#include <cerrno>
#include <climits>
#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <ctime>

namespace hcs_utility {
namespace detail {

inline auto atomic_futex_address(const std::atomic<uint32_t>& atomic) noexcept -> uint32_t* {
    return const_cast<uint32_t*>(reinterpret_cast<const uint32_t*>(&atomic));
}

inline bool atomic_futex_wait_until_steady(
    const std::atomic<uint32_t>& atomic, uint32_t old_val,
    std::chrono::steady_clock::time_point deadline, std::memory_order order) noexcept {

    if (atomic.load(order) != old_val)
        return true;

    // 只读一次表:绝对截止时刻与"是否已过期"从同一个"现在"推出来。
    // 旧实现先 clock_gettime 再 steady_clock::now(),两次读表之间的间隙会让
    // futex 的绝对超时比目标时刻早几十纳秒 —— wait_until 可能假超时。
    // Linux 上 steady_clock 与 CLOCK_MONOTONIC 同源(precise_sleep 已 static_assert
    // 过 tick 形状),deadline 的 epoch 直接就是 futex 想要的绝对时刻。
    ::timespec now_ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &now_ts);
    const std::int64_t now_ns =
        static_cast<std::int64_t>(now_ts.tv_sec) * 1'000'000'000LL + now_ts.tv_nsec;
    const auto deadline_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(deadline.time_since_epoch())
            .count();

    if (deadline_ns <= now_ns)
        return atomic.load(order) != old_val;

    ::timespec abs_time{};
    abs_time.tv_sec = static_cast<time_t>(deadline_ns / 1'000'000'000);
    abs_time.tv_nsec = static_cast<long>(deadline_ns % 1'000'000'000);

    auto* const ptr = atomic_futex_address(atomic);

    while (atomic.load(order) == old_val) {
        if (syscall(
                SYS_futex, ptr, FUTEX_WAIT_BITSET_PRIVATE, old_val, &abs_time, nullptr,
                FUTEX_BITSET_MATCH_ANY)
            == 0)
            continue;

        if (errno == EAGAIN || errno == EINTR)
            continue;

        if (errno == ETIMEDOUT)
            return atomic.load(order) != old_val;

        return atomic.load(order) != old_val;
    }

    return true;
}

} // namespace detail

/**
 * @brief Waits until a 32-bit atomic value changes or the timeout expires.
 *
 * This helper uses Linux futex syscalls directly and must be paired with
 * hcs_utility::atomic_futex_notify_one() or hcs_utility::atomic_futex_notify_all()
 * on the same atomic object.
 *
 * @warning This implementation is not compatible with `std::atomic::notify_one()` or
 * `std::atomic::notify_all()`. The standard library may track waiters in an internal pool,
 * so mixing these APIs on the same atomic object can miss wakeups.
 *
 * @param atomic Atomic word used as the futex key.
 * @param old_val Expected value observed before waiting.
 * @param timeout Maximum time to wait. A non-positive timeout performs a non-blocking check.
 * @param order Memory order used for polling loads around the futex wait.
 * @return `true` if `atomic` no longer equals `old_val`, otherwise `false` when the timeout
 * expires.
 */
inline bool atomic_futex_wait_for(
    const std::atomic<uint32_t>& atomic, uint32_t old_val,
    std::chrono::steady_clock::duration timeout,
    std::memory_order order = std::memory_order_seq_cst) noexcept {

    if (atomic.load(order) != old_val)
        return true;

    if (timeout <= std::chrono::steady_clock::duration::zero())
        return false;

    return detail::atomic_futex_wait_until_steady(
        atomic, old_val, std::chrono::steady_clock::now() + timeout, order);
}

/**
 * @brief Waits until a 32-bit atomic value changes or the steady-clock deadline is reached.
 *
 * This helper uses Linux futex syscalls directly and must be paired with
 * hcs_utility::atomic_futex_notify_one() or hcs_utility::atomic_futex_notify_all()
 * on the same atomic object.
 *
 * @warning This implementation is not compatible with `std::atomic::notify_one()` or
 * `std::atomic::notify_all()`. The standard library may track waiters in an internal pool,
 * so mixing these APIs on the same atomic object can miss wakeups.
 *
 * @param atomic Atomic word used as the futex key.
 * @param old_val Expected value observed before waiting.
 * @param deadline Steady-clock deadline after which the wait times out.
 * @param order Memory order used for polling loads around the futex wait.
 * @return `true` if `atomic` no longer equals `old_val`, otherwise `false` when the deadline
 * is reached.
 */
inline bool atomic_futex_wait_until(
    const std::atomic<uint32_t>& atomic, uint32_t old_val,
    std::chrono::steady_clock::time_point deadline,
    std::memory_order order = std::memory_order_seq_cst) noexcept {

    return detail::atomic_futex_wait_until_steady(atomic, old_val, deadline, order);
}

/**
 * @brief Wakes one thread blocked in hcs_utility::atomic_futex_wait_for() or
 * hcs_utility::atomic_futex_wait_until().
 *
 * @warning This must not be mixed with `std::atomic::notify_one()` or
 * `std::atomic::notify_all()` on the same atomic object.
 *
 * @param atomic Atomic word used as the futex key.
 */
inline void atomic_futex_notify_one(const std::atomic<uint32_t>& atomic) noexcept {
    syscall(
        SYS_futex, detail::atomic_futex_address(atomic), FUTEX_WAKE_BITSET_PRIVATE, 1, nullptr,
        nullptr, FUTEX_BITSET_MATCH_ANY);
}

/**
 * @brief Wakes all threads blocked in hcs_utility::atomic_futex_wait_for() or
 * hcs_utility::atomic_futex_wait_until().
 *
 * @warning This must not be mixed with `std::atomic::notify_one()` or
 * `std::atomic::notify_all()` on the same atomic object.
 *
 * @param atomic Atomic word used as the futex key.
 */
inline void atomic_futex_notify_all(const std::atomic<uint32_t>& atomic) noexcept {
    syscall(
        SYS_futex, detail::atomic_futex_address(atomic), FUTEX_WAKE_BITSET_PRIVATE, INT_MAX,
        nullptr, nullptr, FUTEX_BITSET_MATCH_ANY);
}

} // namespace hcs_utility
