// 平衡图性能基准：无板环境量化算力占用，回答「1 kHz 预算里有没有瓶颈」。
//
// 三个度量：
//   1) 全图整拍耗时（无逐组件插桩）—— 执行预算占用率
//   2) 逐组件耗时 —— 热点定位
//   3) NLMPC / LQR 表求解微基准 —— 最重组件的最坏路径（含 Riccati 重建触发）
//
// 断言只设病理性上限（防回归），绝对时间只打印不判：共享机器上时间断言会闪。
// 节奏质量（拍点抖动、跳拍）受桌面内核噪声支配，无板测了不算数——上板后由
// executor 的 rt_sampler + traces 脚本测，本基准只测「算力」这一维。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <expected>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <hcs_executor/component.hpp>
#include <hcs_executor/wiring.hpp>
#include <hcs_msgs/keyboard.hpp>
#include <hcs_msgs/mouse.hpp>
#include <hcs_msgs/switch.hpp>
#include <hcs_sync/tick.hpp>

#include "controller/chassis/balance/balance_types.hpp"
#include "controller/chassis/balance/nlmpc_solver.hpp"

// 与 test_balance_graph 相同的单文件类编译模式。
#include "controller/chassis/balance/balance_chassis_controller.cpp"
#include "controller/chassis/balance/balance_state_estimator.cpp"
#include "controller/chassis/balance/balance_mode_manager.cpp"
#include "controller/chassis/balance/balance_lqr_controller.cpp"
#include "controller/chassis/balance/leg_force_controller.cpp"
#include "controller/chassis/balance/leg_joint_controller.cpp"

namespace hcs = hcs_core::controller::chassis::balance;

namespace {

constexpr auto kPeriod = std::chrono::milliseconds{1};
using hcs_executor::Component;
using hcs_executor::Linker;

/// 固定种子 LCG：确定性激励，不引入 <random> 的运行期差异。
class Noise {
public:
    double next() noexcept { // [-1, 1]
        state_ = state_ * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(state_ >> 11) / 4503599627370496.0 - 1.0;
    }

private:
    std::uint64_t state_ = 0x243F6A8885A308D3ULL;
};

/// 假硬件（带激励）：腿长摆动跨过 NLMPC 的 2mm 重建阈值，轮速/IMU 持续激励，
/// 让估计器、模式管理、NLMPC 在线线性化做真活而不是空转。
class BenchFakeHardware : public Component {
public:
    BenchFakeHardware() {
        register_output("/remote/joystick/right", joystick_, Eigen::Vector2d::Zero());
        register_output("/remote/keyboard", keyboard_, hcs_msgs::Keyboard::zero());
        register_output("/remote/switch/left", switch_left_, hcs_msgs::Switch::MIDDLE);
        register_output("/gimbal/yaw/angle", gimbal_yaw_angle_, 0.0);

        register_joint_interfaces("left_front");
        register_joint_interfaces("left_back");
        register_joint_interfaces("right_front");
        register_joint_interfaces("right_back");
        register_output("/chassis/left_wheel/velocity", left_wheel_velocity_, 0.0);
        register_output("/chassis/right_wheel/velocity", right_wheel_velocity_, 0.0);
        register_output("/chassis/imu/euler", imu_euler_, Eigen::Vector3d::Zero());
        register_output("/chassis/imu/angular_velocity", imu_angular_velocity_, Eigen::Vector3d::Zero());
        register_output("/chassis/imu/acceleration", imu_acceleration_, Eigen::Vector3d::Zero());
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        const double t = 0.001 * static_cast<double>(tick.sequence);
        for (std::size_t i = 0; i < 4; ++i) {
            // 摆角 ±0.3rad、腿长随之波动：周期性跨过 |ΔLL|>2mm 的 Riccati 重建条件。
            *joint_angles_[i] =
                1.5 + 0.28 * std::sin(2.0 * t + 1.6 * static_cast<double>(i)) + 0.03 * noise_.next();
            *joint_velocities_[i] =
                0.56 * std::cos(2.0 * t + 1.6 * static_cast<double>(i)) + 0.2 * noise_.next();
        }
        *left_wheel_velocity_  = 2.5 * std::sin(t) + 0.5 * noise_.next();
        *right_wheel_velocity_ = 2.5 * std::sin(t) + 0.8 * std::sin(0.3 * t) + 0.5 * noise_.next();
        // 欧拉角分量顺序 roll/pitch/yaw，与 Hipnuc 的 euler_angles() 一致
        *imu_euler_ = Eigen::Vector3d{0.06 * std::cos(2.3 * t), 0.1 * std::sin(3.0 * t), 0.0};
        *imu_angular_velocity_ = Eigen::Vector3d{
            0.2 * std::sin(2.3 * t) + 0.1 * noise_.next(),
            0.3 * std::cos(3.0 * t) + 0.1 * noise_.next(),
            0.5 * std::sin(0.7 * t) + 0.1 * noise_.next()};
    }

private:
    void register_joint_interfaces(const std::string& joint) {
        const std::string prefix = "/chassis/" + joint + "_joint";
        const auto index = joint_count_++;
        register_output(prefix + "/angle", joint_angles_[index], 1.5);
        register_output(prefix + "/velocity", joint_velocities_[index], 0.0);
        register_output(prefix + "/torque", joint_torque_feedbacks_[index], 0.0);
    }

    std::size_t joint_count_ = 0;
    Noise noise_;

    OutputInterface<Eigen::Vector2d> joystick_;
    OutputInterface<hcs_msgs::Keyboard> keyboard_;
    OutputInterface<hcs_msgs::Switch> switch_left_;
    OutputInterface<double> gimbal_yaw_angle_;
    std::array<OutputInterface<double>, 4> joint_angles_{};
    std::array<OutputInterface<double>, 4> joint_velocities_{};
    std::array<OutputInterface<double>, 4> joint_torque_feedbacks_{};
    OutputInterface<double> left_wheel_velocity_;
    OutputInterface<double> right_wheel_velocity_;
    OutputInterface<Eigen::Vector3d> imu_euler_;
    OutputInterface<Eigen::Vector3d> imu_angular_velocity_;
    OutputInterface<Eigen::Vector3d> imu_acceleration_;
};

class BenchFakeCommand : public Component {
public:
    BenchFakeCommand() {
        register_input("/chassis/left_wheel/control_torque", left_wheel_torque_, false);
        register_input("/chassis/right_wheel/control_torque", right_wheel_torque_, false);
        register_input("/chassis/left_front_joint/control_torque", left_front_torque_, false);
        register_input("/chassis/left_back_joint/control_torque", left_back_torque_, false);
        register_input("/chassis/right_front_joint/control_torque", right_front_torque_, false);
        register_input("/chassis/right_back_joint/control_torque", right_back_torque_, false);
    }

    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {}

private:
    InputInterface<double> left_wheel_torque_;
    InputInterface<double> right_wheel_torque_;
    InputInterface<double> left_front_torque_;
    InputInterface<double> left_back_torque_;
    InputInterface<double> right_front_torque_;
    InputInterface<double> right_back_torque_;
};

using Clock = std::chrono::steady_clock;

/// p50 / p99 / max，单位 µs。入参按值收走排序。
std::array<double, 3> percentiles_us(std::vector<double> samples) {
    std::sort(samples.begin(), samples.end());
    const auto at = [&samples](double q) {
        const auto index = static_cast<std::size_t>(q * static_cast<double>(samples.size() - 1));
        return samples[index] * 1e6;
    };
    return {at(0.50), at(0.99), samples.back() * 1e6};
}

/// 图 + 假硬件。组件名必须与 balance_graph_test.yaml 一致（参数按节点名挂）；
/// 构造必须发生在 NameScope 之内——组件构造时就要拿名字建 Node。
template <typename T>
void add(std::vector<std::shared_ptr<Component>>& out, const std::string& name) {
    const Component::NameScope scope{name};
    out.push_back(std::make_shared<T>());
}

std::expected<hcs_executor::Wiring, hcs_executor::graph::BuildError> build_graph(
    std::vector<std::shared_ptr<Component>>& out) {
    add<BenchFakeHardware>(out, "balance_hardware");
    add<BenchFakeCommand>(out, "balance_hardware_command");
    add<hcs::BalanceStateEstimator>(out, "balance_state_estimator");
    add<hcs::BalanceChassisController>(out, "balance_chassis_controller");
    add<hcs::BalanceModeManager>(out, "balance_mode_manager");
    add<hcs::BalanceLqrController>(out, "balance_lqr_controller");
    add<hcs::LegForceController>(out, "leg_force_controller");
    add<hcs::LegJointController>(out, "left_leg_joint_controller");
    add<hcs::LegJointController>(out, "right_leg_joint_controller");
    return Linker::link(out);
}

TEST(BalanceBench, WholeGraphTickCost) {
    std::vector<std::shared_ptr<Component>> components;
    auto wiring = build_graph(components);
    ASSERT_TRUE(wiring.has_value()) << wiring.error().message;

    hcs_sync::Timestamp start = hcs_sync::Clock::now();
    std::uint64_t sequence = 0;
    // 预热：缓存、Riccati 表、TLB。
    for (int i = 0; i < 500; ++i) {
        for (const auto& entry : wiring->latches)
            entry.latch(entry.interface);
        const hcs_sync::Tick tick{
            .scheduled = start + kPeriod * sequence,
            .dt = std::chrono::duration_cast<hcs_sync::Duration>(kPeriod),
            .sequence = sequence};
        ++sequence;
        for (auto* component : wiring->updating_order)
            if (!component->failed())
                component->update(tick);
        start = hcs_sync::Clock::now();
    }

    constexpr int kSamples = 20000;
    std::vector<double> tick_times;
    tick_times.reserve(kSamples);
    for (int i = 0; i < kSamples; ++i) {
        const auto t0 = Clock::now();
        for (const auto& entry : wiring->latches)
            entry.latch(entry.interface);
        const hcs_sync::Tick tick{
            .scheduled = start + kPeriod * sequence,
            .dt = std::chrono::duration_cast<hcs_sync::Duration>(kPeriod),
            .sequence = sequence};
        ++sequence;
        for (auto* component : wiring->updating_order)
            if (!component->failed())
                component->update(tick);
        start = hcs_sync::Clock::now();
        const auto t1 = Clock::now();
        tick_times.push_back(std::chrono::duration<double>(t1 - t0).count());
    }

    for (const auto* component : wiring->updating_order)
        ASSERT_FALSE(component->failed()) << component->get_component_name() << " threw";

    const auto [p50, p99, max] = percentiles_us(tick_times);
    std::printf("\n[bench] 整拍（latch+全部组件，不含睡眠）  n=%d\n", kSamples);
    std::printf("[bench]   p50=%7.1f us  p99=%7.1f us  max=%7.1f us  → 1ms 预算占用 p99 %.1f%%\n",
                p50, p99, max, p99 / 10.0);

    EXPECT_LT(max, 10000.0); // 病理性上限 10ms 防回归；紧阈值在共享机器上会闪测
}

TEST(BalanceBench, PerComponentCost) {
    std::vector<std::shared_ptr<Component>> components;
    auto wiring = build_graph(components);
    ASSERT_TRUE(wiring.has_value()) << wiring.error().message;

    const auto& order = wiring->updating_order;
    std::vector<std::vector<double>> samples(order.size());
    hcs_sync::Timestamp start = hcs_sync::Clock::now();
    std::uint64_t sequence = 0;
    for (int i = 0; i < 500 + 5000; ++i) {
        for (const auto& entry : wiring->latches)
            entry.latch(entry.interface);
        const hcs_sync::Tick tick{
            .scheduled = start + kPeriod * sequence,
            .dt = std::chrono::duration_cast<hcs_sync::Duration>(kPeriod),
            .sequence = sequence};
        ++sequence;
        for (std::size_t c = 0; c < order.size(); ++c) {
            const auto t0 = Clock::now();
            if (!order[c]->failed())
                order[c]->update(tick);
            const auto t1 = Clock::now();
            if (i >= 500)
                samples[c].push_back(std::chrono::duration<double>(t1 - t0).count());
        }
        start = hcs_sync::Clock::now();
    }

    std::printf("\n[bench] 逐组件（每组件含两次时钟读取约 40-60ns 开销）  n=5000\n");
    for (std::size_t c = 0; c < order.size(); ++c) {
        const auto [p50, p99, max] = percentiles_us(samples[c]);
        std::printf("[bench]   %-30s p50=%7.2f us  p99=%7.2f us  max=%7.2f us\n",
                    order[c]->get_component_name().c_str(), p50, p99, max);
        EXPECT_LT(max, 10000.0);
    }
}

TEST(BalanceBench, SolverWorstPath) {
    Noise noise;

    // NLMPC：腿长每 7 拍跨一次 2mm 阈值，强制 Riccati 重建混入统计。
    hcs::BalanceNlmpc nlmpc;
    nlmpc.reset();
    std::vector<double> nlmpc_us;
    int solved = 0;
    for (int i = 0; i < 20000; ++i) {
        const double left = 0.19 + 0.01 * std::sin(0.05 * i);
        const double right = left + ((i % 7 == 0) ? 0.004 : 0.0005 * std::sin(0.3 * i));
        double ref[hcs::BalanceNlmpc::kStateSize];
        for (auto& v : ref)
            v = 0.3 * noise.next();
        double out[hcs::BalanceNlmpc::kControlSize]{};

        const auto t0 = Clock::now();
        const bool ok = nlmpc.solve(
            left, right, ref, 0.2 * std::sin(0.02 * i), 0.2 * std::cos(0.03 * i), out);
        const auto t1 = Clock::now();
        if (ok)
            ++solved;
        nlmpc_us.push_back(std::chrono::duration<double>(t1 - t0).count());
        for (double v : out)
            EXPECT_TRUE(std::isfinite(v));
    }

    // LQR 拟合表回退路径：lqr_k + lqr_torques 一起，对应控制器里 NLMPC 失败的整链。
    std::vector<double> lqr_us;
    double k_matrix[4][10];
    for (int i = 0; i < 20000; ++i) {
        const double left = 0.19 + 0.01 * std::sin(0.05 * i);
        const double right = left + 0.004 * std::sin(0.11 * i);
        double state[hcs::kStateSize];
        for (auto& v : state)
            v = noise.next();

        const auto t0 = Clock::now();
        hcs::lqr_k(left, right, hcs::kKOut, k_matrix);
        double wl, wr, hl, hr;
        hcs::lqr_torques(k_matrix, state, wl, wr, hl, hr, i % 3 == 0, i % 5 == 0, 0.1);
        const auto t1 = Clock::now();
        lqr_us.push_back(std::chrono::duration<double>(t1 - t0).count());
        for (double v : {wl, wr, hl, hr})
            EXPECT_TRUE(std::isfinite(v));
    }

    const auto [n50, n99, nmax] = percentiles_us(nlmpc_us);
    const auto [l50, l99, lmax] = percentiles_us(lqr_us);
    std::printf("\n[bench] 求解器最坏路径  n=20000  NLMPC 成功率 %.1f%%\n",
                100.0 * solved / 20000.0);
    std::printf("[bench]   NLMPC(含重建)  p50=%7.2f us  p99=%7.2f us  max=%7.2f us\n", n50, n99, nmax);
    std::printf("[bench]   LQR 表回退     p50=%7.2f us  p99=%7.2f us  max=%7.2f us\n", l50, l99, lmax);

    EXPECT_GT(solved, 0);
    EXPECT_LT(nmax, 10000.0);
    EXPECT_LT(lmax, 10000.0);
}

} // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    setenv("ROS_LOG_DIR", "/tmp/hcs_test_ros_logs", 0);
    std::vector<char*> arguments{argv, argv + argc};
    static const std::string params_path = BALANCE_GRAPH_TEST_YAML;
    arguments.push_back(const_cast<char*>("--ros-args"));
    arguments.push_back(const_cast<char*>("--params-file"));
    arguments.push_back(const_cast<char*>(params_path.c_str()));
    rclcpp::init(static_cast<int>(arguments.size()), arguments.data());
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
