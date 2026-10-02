#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace hcs_core::controller::shooting {

/// 一批弹速的统计。
struct ShootingStatistics {
    double mean = 0.0;
    /// 落在平均值 ±0.025 m/s 以内的比例。
    double excellence_rate = 0.0;
    /// 落在平均值 ±0.05 m/s 以内的比例。
    double pass_rate = 0.0;
    /// 最大减最小。
    double range = 0.0;
    /// 去掉一个最大、一个最小之后的最大减最小；不足三发时等于 range。
    double trimmed_range = 0.0;
    double max = 0.0;
    double min = 0.0;
};

/// 算一批弹速的统计。会把 velocities **原地排序**；空的时候返回全零。
[[nodiscard]] inline ShootingStatistics analyse(std::vector<double>& velocities) {
    ShootingStatistics result;
    if (velocities.empty())
        return result;

    const auto count = static_cast<double>(velocities.size());

    double sum = 0.0;
    for (const double velocity : velocities)
        sum += velocity;
    result.mean = sum / count;

    std::ranges::sort(velocities);
    result.min = velocities.front();
    result.max = velocities.back();
    result.range = result.max - result.min;
    result.trimmed_range =
        velocities.size() >= 3 ? velocities[velocities.size() - 2] - velocities[1] : result.range;

    std::size_t excellent = 0;
    std::size_t passed = 0;
    for (const double velocity : velocities) {
        if (velocity >= result.mean - 0.05 && velocity <= result.mean + 0.05)
            ++passed;
        if (velocity >= result.mean - 0.025 && velocity <= result.mean + 0.025)
            ++excellent;
    }
    result.excellence_rate = static_cast<double>(excellent) / count;
    result.pass_rate = static_cast<double>(passed) / count;
    return result;
}

} // namespace hcs_core::controller::shooting
