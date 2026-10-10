#pragma once

#include <chrono>
#include <cstdint>
#include <ratio>

namespace hcs_msgs {

/// 板卡上的时钟：计数单位是四分之一微秒（板载 IMU、相机触发的时间戳都是它）。
/// 只是一个 std::chrono 的时钟标签，用来区分"板上的时刻"和主机的时刻——两者不同源，不能相减。
struct BoardClock {
    using rep = std::int64_t;
    using period = std::ratio<1, 4'000'000>;
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<BoardClock, duration>;

    static constexpr bool is_steady = true;
};

} // namespace hcs_msgs
