#pragma once

#include <chrono>
#include <cstdint>

namespace rmcs_sync {

using Clock = std::chrono::steady_clock;
using Timestamp = Clock::time_point;
using Duration = std::chrono::nanoseconds;

/// 周期域的时间。update() 唯一允许的时间来源。
///
/// 这里没有 `locked`（时基是否锁相）那一位：任何形式的时钟对齐都不做，
/// 所以它恒为 false、零个读者，留着只会让人以为这套支持锁相。
///
/// 两条硬规矩：
///   - 参考轨迹只许用 `scheduled`（或 `sequence * period`）推，禁止用计数乘 dt。
///   - dt 只许来自 `dt`，禁止用 1.0 / update_rate。
struct Tick {
    Timestamp scheduled{};       ///< 本拍名义起点。跨跳拍完全正确。
    Duration dt{};               ///< 距上一拍 scheduled。跳拍时是 N × period。首拍为 period。
    std::uint64_t sequence = 0;  ///< 拍号，按“拍数”递增（含被跳过的拍），不是执行次数。

    /// 秒制 dt，给控制律用。
    [[nodiscard]] constexpr double dt_seconds() const noexcept {
        return std::chrono::duration<double>(dt).count();
    }
};

} // namespace rmcs_sync
