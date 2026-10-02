#pragma once

#include <cstdint>
#include <optional>

#include <hcs_msgs/board_clock.hpp>

namespace hcs_core::hardware::device {

/// 把板上 32 位、会回绕的时间戳（四分之一微秒，约 18 分钟绕一圈）展开成不回绕的 64 位时刻。
///
/// 做法是留一个"时基"：一路持续到来的时间戳（加速度计）不断把它往前推，每次按 32 位差值累加，
/// 回绕自然就被吃掉了。别的时间戳（陀螺仪、相机触发）只要离时基不超过半圈（约 9 分钟），
/// 就能靠时基的高位把自己补全。
///
/// 只归一条线程：没有锁，也没有原子量。
class BoardClockLifter {
public:
    using time_point = hcs_msgs::BoardClock::time_point;

    /// 用持续到来的那一路时间戳推进时基，返回它展开后的时刻。第一次调用把时基定在这个时间戳上。
    time_point advance_timebase(std::uint32_t raw_timestamp_quarter_us) noexcept {
        if (!has_timebase_) {
            has_timebase_ = true;
            last_raw_ = raw_timestamp_quarter_us;
            timebase_ = raw_timestamp_quarter_us;
        }

        // 无符号相减：跨过回绕点时差值仍然是对的。
        timebase_ += static_cast<std::uint32_t>(raw_timestamp_quarter_us - last_raw_);
        last_raw_ = raw_timestamp_quarter_us;
        return as_time_point(timebase_);
    }

    /// 当前的时基；还没推进过则为空。
    [[nodiscard]] std::optional<time_point> timebase() const noexcept {
        if (!has_timebase_)
            return std::nullopt;
        return as_time_point(timebase_);
    }

    /// 把另一路的 32 位时间戳展开到时基所在的那一圈；时基还没建立则为空。
    /// 它可以比时基早，也可以比时基晚，只要相差不到半圈。
    [[nodiscard]] std::optional<time_point> lift_timestamp(
        std::uint32_t timestamp_quarter_us) const noexcept {
        if (!has_timebase_)
            return std::nullopt;

        // 先在 32 位里求差再转成有符号：得到的是 (-半圈, +半圈] 里的偏移。
        const auto timebase_low = static_cast<std::uint32_t>(timebase_);
        const auto offset = static_cast<std::int32_t>(timestamp_quarter_us - timebase_low);
        return as_time_point(timebase_ + static_cast<std::int64_t>(offset));
    }

private:
    [[nodiscard]] static time_point as_time_point(std::int64_t quarter_us) noexcept {
        return time_point{hcs_msgs::BoardClock::duration{quarter_us}};
    }

    bool has_timebase_ = false;
    std::uint32_t last_raw_ = 0;
    std::int64_t timebase_ = 0;
};

} // namespace hcs_core::hardware::device
