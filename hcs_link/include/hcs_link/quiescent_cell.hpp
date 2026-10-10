#pragma once

// 把一个对象从普通线程交给周期域线程用，并且只在周期域确定不再用旧对象之后才销毁它。
// 这是 RCU 的"静止状态"（QSBR）变体：周期域每拍结束就是一个静止点，不需要读者登记。
//
//   周期域，每拍：  T* p = cell.acquire();  …只在本拍内用 p…  cell.quiescent();
//   普通线程：      cell.replace(std::move(next), grace);
//
// replace 先换指针，再等周期域至少走过一个静止点：那之后开始的拍一定拿到新指针，
// 之前拿到旧指针的那一拍一定已经结束。四个操作都是 seq_cst——"我写 A 再读 B、
// 你写 B 再读 A"是 Dekker 形状，release/acquire 挡不住 StoreLoad 重排。

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#include "hcs_link/detail/nonblocking.hpp"

namespace hcs_link {

template <typename T, typename Deleter = std::default_delete<T>>
class QuiescentCell {
public:
    QuiescentCell() noexcept = default;
    QuiescentCell(const QuiescentCell&) = delete;
    QuiescentCell& operator=(const QuiescentCell&) = delete;

    /// 析构时周期域必须已经停止。
    ~QuiescentCell() { Deleter{}(current_.load(std::memory_order_relaxed)); }

    /// 周期域：本拍要用的对象，可能为空。只在下一次 quiescent() 之前有效。
    [[nodiscard]] T* acquire() const noexcept HCS_LINK_NONBLOCKING {
        return current_.load(std::memory_order_seq_cst);
    }

    /// 周期域：本拍结束，之前 acquire() 到的指针一律不再用。
    void quiescent() noexcept HCS_LINK_NONBLOCKING {
        epoch_.store(epoch_.load(std::memory_order_relaxed) + 1, std::memory_order_seq_cst);
    }

    /// 普通线程：换上 next（可以为空），周期域走过一个静止点之后销毁旧对象。
    ///
    /// 周期域在 grace 之内一个静止点都没走（没在跑拍）时放弃销毁、把旧对象泄漏掉并返回 false：
    /// 宁可漏，不可让周期域读到已释放的内存。只许一个线程调用。
    bool replace(std::unique_ptr<T, Deleter> next, std::chrono::nanoseconds grace) {
        std::unique_ptr<T, Deleter> previous{
            current_.exchange(next.release(), std::memory_order_seq_cst)};
        if (!previous)
            return true;

        const std::uint64_t epoch = epoch_.load(std::memory_order_seq_cst);
        const auto deadline = std::chrono::steady_clock::now() + grace;
        while (epoch_.load(std::memory_order_seq_cst) == epoch) {
            if (std::chrono::steady_clock::now() >= deadline) {
                (void)previous.release();
                return false;
            }
            std::this_thread::sleep_for(std::chrono::microseconds{100});
        }
        return true;
    }

    /// 周期域已经走过的静止点个数（诊断用）。
    [[nodiscard]] std::uint64_t epoch() const noexcept {
        return epoch_.load(std::memory_order_relaxed);
    }

private:
    std::atomic<T*> current_{nullptr};
    std::atomic<std::uint64_t> epoch_{0};
};

} // namespace hcs_link
