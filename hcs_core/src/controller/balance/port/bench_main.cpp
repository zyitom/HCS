// 阶段 3：纯逻辑层 1 kHz 实时性基准。
// 以 LQR_ON（NLMPC 求解活跃）的典型状态连续推进 ChassisLogic，统计每拍耗时。

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

#include "../chassis_logic.hpp"

#include "scenarios.hpp"

using namespace hcs_core::controller::balance;
using namespace hcs_core::controller::balance::port;
using Clock = std::chrono::steady_clock;

int main(int argc, char** argv) {
    const int ticks = argc > 1 ? std::atoi(argv[1]) : 100000;

    Params params;
    ChassisLogic logic(params);

    // 站立+行驶混合输入：NLMPC 每拍求解的最重路径。
    TickInputs in;
    const Poses P;
    in.if_enable = true;
    in.pose_left = P.stand_low;
    in.pose_right = P.stand_low;
    in.speed_level[0] = 1.0f;
    in.wheel_speed_l = 33.0f;
    in.wheel_speed_r = -33.0f;
    in.leg_torque_l0 = -12.0f;
    in.leg_torque_l1 = -12.0f;
    in.leg_torque_r0 = -12.0f;
    in.leg_torque_r1 = -12.0f;

    const auto [phi0t_l, d_l] = solve_motor_angles(in.pose_left);
    const auto [phi0t_r, d_r] = solve_motor_angles(in.pose_right);
    const MotorAngles angles =
        phi_to_motor(phi0t_l + d_l, phi0t_l - d_l, phi0t_r + d_r, phi0t_r - d_r);

    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(ticks));

    LogicInput input{};
    input.imu.acc_z = 1.0f;
    input.leg_motor[0] = {angles.bl1, 0.0f, in.leg_torque_l1};
    input.leg_motor[1] = {angles.bl0, 0.0f, in.leg_torque_l0};
    input.leg_motor[2] = {angles.br1, 0.0f, in.leg_torque_r1};
    input.leg_motor[3] = {angles.br0, 0.0f, in.leg_torque_r0};
    input.wheel[0] = {in.wheel_speed_l, 0.0f};
    input.wheel[1] = {in.wheel_speed_r, 0.0f};
    input.command.if_enable = true;
    input.command.speed_level[0] = 1.0f;

    // 预热（分支预测、缓存、NLMPC 的 Riccati 构建）
    for (int i = 0; i < 2000; ++i)
        logic.update(input, 0.001f);

    for (int i = 0; i < ticks; ++i) {
        const auto start = Clock::now();
        logic.update(input, 0.001f);
        const auto end = Clock::now();
        samples.push_back(
            std::chrono::duration<double, std::nano>(end - start).count());
    }

    std::sort(samples.begin(), samples.end());
    const auto percentile = [&](double p) {
        return samples[static_cast<std::size_t>(p * static_cast<double>(samples.size() - 1))];
    };
    double sum = 0.0;
    for (const double s : samples)
        sum += s;

    std::printf("ticks=%d  mean=%.0f ns  p50=%.0f ns  p99=%.0f ns  p99.9=%.0f ns  max=%.0f ns\n",
                ticks, sum / static_cast<double>(ticks), percentile(0.50), percentile(0.99),
                percentile(0.999), samples.back());
    std::printf("budget: 1000000 ns/tick  ->  usage mean=%.2f%% max=%.2f%%\n",
                100.0 * sum / static_cast<double>(ticks) / 1e6,
                100.0 * samples.back() / 1e6);

    const auto& diag = logic.nlmpc_diagnostics();
    std::printf("nlmpc: solve_count=%u fallback_count=%u\n", diag.solve_count,
                diag.fallback_count);
    return 0;
}
