#pragma once

// 自瞄与 hcs_executor 之间的契约：两种载荷，外加两边都要遵守的校验和外推规则。
//
//   GimbalState：控制写，视觉读。每拍一条。
//   AimCommand ：视觉写，控制读。每处理完一帧一条，没看到目标也发（兼作心跳）。
//
// 时间一律是 CLOCK_MONOTONIC 纳秒（std::chrono::steady_clock），两个进程天然共用。
// 角度一律在 OdomImu 世界系：方位角绕 +z、自 +x 逆时针为正，俯仰角向上为正。

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <string_view>

namespace hcs_link::autoaim {

/// 会合用的 abstract socket 名。
inline constexpr std::string_view kEndpoint = "hcs/autoaim";

struct GimbalState {
    static constexpr std::string_view kName = "hcs.autoaim.GimbalState";
    static constexpr std::uint32_t kVersion = 1;

    std::int64_t tick_ns;                  ///< 这一拍的 tick.scheduled
    std::uint64_t tick_sequence;           ///< 拍号（跳拍时不连续）
    std::array<double, 4> quaternion;      ///< w x y z：云台 IMU 机体 FLU 相对 OdomImu 世界系
    std::array<double, 3> angular_velocity; ///< rad/s，机体 FLU
    std::uint32_t imu_online;              ///< 0 / 1
    std::uint32_t reserved;                ///< 写 0
};
static_assert(sizeof(GimbalState) == 80, "layout changed: bump GimbalState::kVersion");

struct AimCommand {
    static constexpr std::string_view kName = "hcs.autoaim.AimCommand";
    static constexpr std::uint32_t kVersion = 1;

    static constexpr std::uint32_t kHasTarget = 1U << 0;
    static constexpr std::uint32_t kFire = 1U << 1;
    static constexpr std::uint32_t kKnownFlags = kHasTarget | kFire;

    std::int64_t t_ref_ns;          ///< 下面各角度量成立的时刻
    std::uint64_t frame_id;         ///< 视觉自己的帧号，只用于日志对照
    double azimuth;                 ///< rad
    double elevation;               ///< rad，|·| ≤ π/2
    double azimuth_rate;            ///< rad/s
    double elevation_rate;          ///< rad/s
    double azimuth_acceleration;    ///< rad/s²
    double elevation_acceleration;  ///< rad/s²
    std::int64_t fire_from_ns;      ///< 允许开火的时间窗，kFire 置位时才看
    std::int64_t fire_until_ns;
    std::uint32_t flags;            ///< kHasTarget | kFire，其余位必须为 0
    std::uint32_t reserved;         ///< 写 0
};
static_assert(sizeof(AimCommand) == 88, "layout changed: bump AimCommand::kVersion");

/// 控制侧拒收命令的界限。视觉写出界的值说明视觉有 bug，宁可不跟也不乱跟。
struct Limits {
    std::int64_t max_age_ns = 200'000'000;        ///< t_ref 最多比现在早这么多
    std::int64_t max_lead_ns = 50'000'000;        ///< t_ref 最多比现在晚这么多
    double max_rate = 50.0;                       ///< rad/s
    double max_acceleration = 1000.0;             ///< rad/s²
    std::int64_t max_fire_window_ns = 200'000'000;
};

[[nodiscard]] inline bool is_valid(const AimCommand& command, std::int64_t now_ns,
                                   const Limits& limits = {}) noexcept {
    if ((command.flags & ~AimCommand::kKnownFlags) != 0 || command.reserved != 0)
        return false;

    const std::array values{
        command.azimuth,      command.elevation,           command.azimuth_rate,
        command.elevation_rate, command.azimuth_acceleration, command.elevation_acceleration};
    if (!std::ranges::all_of(values, [](double value) { return std::isfinite(value); }))
        return false;

    if (std::abs(command.elevation) > std::numbers::pi / 2)
        return false;
    if (std::abs(command.azimuth_rate) > limits.max_rate
        || std::abs(command.elevation_rate) > limits.max_rate)
        return false;
    if (std::abs(command.azimuth_acceleration) > limits.max_acceleration
        || std::abs(command.elevation_acceleration) > limits.max_acceleration)
        return false;

    if (command.t_ref_ns < now_ns - limits.max_age_ns || command.t_ref_ns > now_ns + limits.max_lead_ns)
        return false;

    if ((command.flags & AimCommand::kFire) != 0
        && (command.fire_from_ns > command.fire_until_ns
            || command.fire_until_ns - command.fire_from_ns > limits.max_fire_window_ns))
        return false;

    return true;
}

struct Aim {
    double azimuth;
    double elevation;
};

/// 把命令外推到 at_ns。外推时长钳在 [0, max_extrapolation_ns]：不往回推，也不无限往前推。
[[nodiscard]] inline Aim extrapolate(const AimCommand& command, std::int64_t at_ns,
                                     std::int64_t max_extrapolation_ns) noexcept {
    const std::int64_t span = std::clamp<std::int64_t>(at_ns - command.t_ref_ns, 0, max_extrapolation_ns);
    const double dt = static_cast<double>(span) * 1e-9;
    const double half_dt2 = 0.5 * dt * dt;
    return Aim{
        .azimuth = command.azimuth + command.azimuth_rate * dt + command.azimuth_acceleration * half_dt2,
        .elevation = std::clamp(
            command.elevation + command.elevation_rate * dt + command.elevation_acceleration * half_dt2,
            -std::numbers::pi / 2, std::numbers::pi / 2),
    };
}

/// OdomImu 世界系下的单位方向向量 (x, y, z)。
[[nodiscard]] inline std::array<double, 3> direction(const Aim& aim) noexcept {
    const double horizontal = std::cos(aim.elevation);
    return {horizontal * std::cos(aim.azimuth), horizontal * std::sin(aim.azimuth),
            std::sin(aim.elevation)};
}

[[nodiscard]] inline bool fire_allowed(const AimCommand& command, std::int64_t at_ns) noexcept {
    return (command.flags & AimCommand::kFire) != 0 && command.fire_from_ns <= at_ns
        && at_ns <= command.fire_until_ns;
}

} // namespace hcs_link::autoaim
