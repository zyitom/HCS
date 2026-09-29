#pragma once

#include <atomic>
#include <cstring>
#include <type_traits>

#include "hcs_utility/raw_storage.hpp"
#include "hcs_utility/thread_assert.hpp"

namespace hcs_utility {

/// @brief 单写者 / 多读者的无锁双缓冲(latest-wins),seqlock 括号协议。
/// @tparam T The type of data to be stored in the buffer.
/// @tparam i_am_very_sure_this_is_trivially_copyable_and_destructible A flag to bypass type checks.
///         Set to `true` only if you are certain that `T` is trivially copyable and destructible.
///
/// 并发契约(旧注释只说 "thread-safe",漏掉了关键的一半):
///   - **write() 只允许一条线程调用。** 两个写者会同时 memcpy 同一个后备缓冲、
///     各自推进序号,读者的检测完全失效,撕裂数据被当成功返回 —— 而且是静默的。
///     debug 构建下用 ThreadOwnership 在第一个越界写者身上当场 abort;
///     release 构建零开销。
///   - read() 可从任意条线程调用。
///
/// 序号协议(每写一次 +2,写前占坑、写后发布):
///
///     store 2w+1 (奇:正在写)  ──fence──▶  memcpy  ──fence──▶  store 2w+2 (偶:可读)
///
/// 旧协议只有最后那一次 store。它留下两个洞:第一,首笔写之前 current_==0 而
/// 后备缓冲未初始化,读者会拿着未初始化数据"成功"返回;第二,"数据可见 ⇒ 序号
/// 已变"的检测在 x86-TSO 上由 store buffer 的 FIFO 顺序兜住,但弱内存序机器
/// (aarch64 也是本库的目标架构)上没有写前那次占坑 store + fence,读者可能看到
/// 新数据、复查到的却还是旧序号 —— 撕裂被放行成 true。seqlock 的括号就是为此
/// 存在的:占坑 store 先于数据可见,读者只要看到任何新字节就一定能看到奇数序号。
///
/// 正确性边界(诚实声明):读侧对后备缓冲的非原子 memcpy 在 C++ 内存模型形式上
/// 仍是数据竞争(seqlock 的经典困局,标准尚无解)。这里给的是与内核 seqlock
/// 同构的工程保证:写侧两道 release fence + 读侧数据读取与序号复查之间的
/// acquire fence,配合 per-location coherence,"撕裂必被检出"在 x86-64 与
/// aarch64 上都成立。若某天要形式上无竞争,需要把 T 的读写全部换成 atomic_ref,
/// 代价是失去 memcpy 的向量化。
template <typename T, bool i_am_very_sure_this_is_trivially_copyable_and_destructible = false>
requires(std::is_trivially_copyable_v<T> && std::is_trivially_destructible_v<T>)
     || i_am_very_sure_this_is_trivially_copyable_and_destructible class DoubleBuffer {
public:
    /// @brief 写入最新样本(单写者!)。
    /// @param data The data to be written to the buffer.
    void write(const T& data) noexcept {
        // 单写者契约的 debug 执行:首个写者认领,之后任何别的线程写 → abort。
        // assert_owned 对未认领状态直接放行,所以第一次 write 就是认领本身。
        writer_.assert_owned();
        writer_.claim();

        // current_ = 2w(w = 已发布笔数,偶 = 稳定)。本笔是第 w 笔(0 基),写 w&1 号缓冲。
        const uint64_t completed = current_.load(std::memory_order_relaxed) >> 1;

        current_.store((completed << 1) | 1, std::memory_order_relaxed);
        // 数据不得越过占坑:读者看到新字节 ⇒ 必已看到奇数序号
        std::atomic_thread_fence(std::memory_order_release);
        std::memcpy(buffers_[completed & 1].bytes, &data, sizeof(T));
        // 发布不得越过数据
        std::atomic_thread_fence(std::memory_order_release);
        current_.store((completed + 1) << 1, std::memory_order_relaxed);
    }

    /// @brief 读取最新已发布样本的快照。
    /// @param data The variable where the read data will be stored.
    /// @return Returns `true` if a stable snapshot was read, `false` otherwise.
    /// @note 三种情况返回 false:从未写过(序号 0)、写者正在写(奇数序号)、
    ///       读取期间又发生了写入(序号变化)。失败时 **`data` 保持原值不动**:
    ///       样本先落到对齐的暂存区、序号确认后才提交,绝不留下半新半旧的数据。
    ///       高写压下失败是常态,调用方应把"这一拍不更新"当作正常分支。
    bool read(T& data) noexcept {
        const uint64_t version = current_.load(std::memory_order::acquire);

        // 0 = 从未发布过(后备缓冲未初始化,不能给);奇 = 正在写。
        if (version == 0 || (version & 1) != 0)
            return false;

        // 序号 2(w+1) 表示第 w 笔写已发布,落在第 w&1 号后备缓冲
        RawStorage<T> staged;
        std::memcpy(staged.bytes, &buffers_[((version >> 1) - 1) & 1], sizeof(T));

        // 数据已读完、复查序号之前立一道 acquire 栅栏:防止弱内存架构上
        // "数据读到了新值、序号却复查到旧值"的重排把撕裂放行成 true。
        std::atomic_thread_fence(std::memory_order::acquire);

        if (current_.load(std::memory_order::relaxed) != version)
            return false;

        std::memcpy(&data, staged.bytes, sizeof(T));
        return true;
    }

private:
    /// debug 构建下抓双写者;release 构建经 [[no_unique_address]] 完全消失。
    [[no_unique_address]] ThreadOwnership writer_;

    RawStorage<T> buffers_[2];
    std::atomic<uint64_t> current_{0};
};

} // namespace hcs_utility
