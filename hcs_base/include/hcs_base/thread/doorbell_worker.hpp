#pragma once

#include <atomic>
#include <chrono>
#include <concepts>
#include <cstdint>
#include <exception>
#include <future>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include "hcs_base/thread/doorbell.hpp"
#include "hcs_base/thread/thread_config.hpp"

namespace hcs_utility {

/**
 * @brief 由门铃驱动的工作线程。
 *
 * 典型用途是传输的发送线程：周期域拍尾 `ring()`，它醒来把最新命令帧发出去。
 * 但它**不认识传输、不认识 CAN、不认识任何设备**——干什么由传进来的可调用体决定。
 *
 * 它存在的意义是把这类线程的纪律收在一处、只写一遍、只测一遍：
 *
 *   - **异常不许带走线程。** 发送路径是会抛的（libhcs 的 `PacketBuilder` 遇到非法数据、
 *     或往这块板没有的总线上发，都会抛）。抛了就计数、丢这一帧、下一次继续；
 *     线程一旦没了，之后每一拍的命令都会静默地发不出去，而日志里什么都没有。
 *   - **阻塞在这里是对的。** 这条线程就是为了替周期域挨那一下而存在的：
 *     libhcs 的 `acquire_transmit_buffer()` 里有互斥量和条件变量，
 *     `libusb_submit_transfer()` 自己还要拿三把锁。这些都躲不掉，
 *     能做的只是让它们发生在**不是控制回路**的线程上。
 *   - **析构必须先停再 join。** 否则回调还在跑，它捕获的东西已经开始析构了。
 *
 * @warning 线程配置失败不抛、不打日志（这个包不认识 ROS），而是记在
 *          `thread_config_error()` 里，由创建方在自己的域里打出来。
 *          这和控制线程的 `RealtimeArm`「失败即抛」是刻意不同的取舍：
 *          控制线程不是 RT 就没有意义，而发送线程掉到 SCHED_OTHER 只是变慢。
 */
template <typename Work>
requires std::invocable<Work&>
class DoorbellWorker {
public:
    /**
     * @param thread_config 线程名 / 优先级 / 绑核。发送线程的建议值是 FIFO 80。
     * @param work 每次被叫醒时执行一次。允许抛（会被计数并吞掉）。
     * @param stop_latency 等待超时。它只决定析构时最坏等多久，不影响正常唤醒延迟。
     */
    DoorbellWorker(
        ThreadConfig thread_config, Work work,
        std::chrono::steady_clock::duration stop_latency = std::chrono::milliseconds{100})
        : work_(std::move(work))
        , thread_config_(std::move(thread_config))
        , stop_latency_(stop_latency) {

        std::promise<void> configured;
        auto configured_future = configured.get_future();
        thread_ = std::thread{[this, configured = std::move(configured)]() mutable {
            main(std::move(configured));
        }};
        // 构造返回时，thread_config_error() 已经可读 —— 创建方紧接着就能把它打出来。
        configured_future.wait();
    }

    ~DoorbellWorker() {
        stop_requested_.store(true, std::memory_order_release);
        doorbell_.ring();
        if (thread_.joinable())
            thread_.join();
    }

    DoorbellWorker(const DoorbellWorker&) = delete;
    DoorbellWorker& operator=(const DoorbellWorker&) = delete;
    DoorbellWorker(DoorbellWorker&&) = delete;
    DoorbellWorker& operator=(DoorbellWorker&&) = delete;

    /// 周期域拍尾对它 `ring()`。
    [[nodiscard]] Doorbell& doorbell() noexcept { return doorbell_; }

    /// 空串表示线程配置成功。构造返回后即可读。
    [[nodiscard]] const std::string& thread_config_error() const noexcept {
        return thread_config_error_;
    }

    /// 被叫醒并执行了几次。
    [[nodiscard]] std::uint64_t wakeups() const noexcept {
        return wakeups_.load(std::memory_order_relaxed);
    }

    /// `work` 抛了几次。**非零就是有命令没发出去**，该报出来。
    [[nodiscard]] std::uint64_t exceptions() const noexcept {
        return exceptions_.load(std::memory_order_relaxed);
    }

private:
    void main(std::promise<void> configured) {
        if (const auto result = thread_config_.apply_to_current_thread(); !result)
            thread_config_error_ = result.error();
        configured.set_value();

        while (!stop_requested_.load(std::memory_order_acquire)) {
            if (!doorbell_.wait_for(stop_latency_))
                continue; // 超时：回去看一眼停机标志
            if (stop_requested_.load(std::memory_order_acquire))
                break;

            wakeups_.fetch_add(1, std::memory_order_relaxed);
            try {
                work_();
            } catch (...) {
                // 吞掉。这里没有日志可打（不认识 ROS），也不该在这条线程上打。
                // 计数由创建方在尽力域里读出来报。
                exceptions_.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }

    Work work_;
    ThreadConfig thread_config_;
    std::chrono::steady_clock::duration stop_latency_;

    Doorbell doorbell_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<std::uint64_t> wakeups_{0};
    std::atomic<std::uint64_t> exceptions_{0};

    /// 只在工作线程 set_value() 之前写，创建方在 future.wait() 之后读。
    std::string thread_config_error_;

    std::thread thread_;
};

} // namespace hcs_utility
