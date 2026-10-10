// AutoAimLink 的端到端测试：真组件 + 真 socket + 真 memfd 通道，视觉由测试里的一个线程扮演。
//
// 走一遍完整生命周期：视觉连上 → 读到云台姿态 → 发命令 → 图输出跟上并外推 → 视觉走了 → 图输出停用。
// 周期域由测试主线程按 1 kHz 实时跑拍，和执行器一样先 latch 再按拓扑序 update。

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <hcs_executor/component.hpp>
#include <hcs_executor/wiring.hpp>
#include <hcs_link/autoaim.hpp>
#include <hcs_link/channel.hpp>
#include <hcs_link/rendezvous.hpp>
#include <hcs_base/channel/tick.hpp>

// 组件是单文件类（.cpp 里定义 + 底部 PLUGINLIB 导出），和 test_balance_graph 一样直接包含。
#include "controller/auto_aim/auto_aim_link.cpp"

namespace {

using namespace std::chrono_literals;
using hcs_executor::Component;
using hcs_link::autoaim::AimCommand;
using hcs_link::autoaim::GimbalState;

const std::string kEndpoint = "hcs-test/autoaim/" + std::to_string(::getpid());
constexpr double kImuYaw = 0.2;

std::int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               hcs_sync::Clock::now().time_since_epoch())
        .count();
}

/// 云台 IMU：绕 z 转了 kImuYaw，以 1 rad/s 转着。
class FakeImu : public Component {
public:
    FakeImu() {
        register_output("/gimbal/imu/quaternion", quaternion_,
                        Eigen::Quaterniond{Eigen::AngleAxisd{kImuYaw, Eigen::Vector3d::UnitZ()}});
        register_output("/gimbal/imu/angular_velocity", angular_velocity_, Eigen::Vector3d::UnitZ());
        register_output("/gimbal/imu/online", online_, true);
    }
    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {}

private:
    OutputInterface<Eigen::Quaterniond> quaternion_;
    OutputInterface<Eigen::Vector3d> angular_velocity_;
    OutputInterface<bool> online_;
};

/// 下游消费者的替身：记下每拍看到的 /auto_aim/*。
class Probe : public Component {
public:
    Probe() {
        register_input("/auto_aim/should_control", should_control_);
        register_input("/auto_aim/control_direction", direction_);
        register_input("/auto_aim/should_shoot", should_shoot_);
    }
    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        control = *should_control_;
        shoot = *should_shoot_;
        direction = direction_->vector;
    }

    bool control = false;
    bool shoot = false;
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();

private:
    InputInterface<bool> should_control_;
    InputInterface<hcs_description::OdomImu::DirectionVector> direction_;
    InputInterface<bool> should_shoot_;
};

/// 视觉进程的替身：连上、互换通道、读姿态、按 200 Hz 发命令，直到被叫停。
class FakeVision {
public:
    FakeVision()
        : thread_{[this](const std::stop_token& stop) { run(stop); }} {}

    std::atomic<bool> connected{false};
    std::atomic<bool> saw_state{false};
    std::atomic<double> state_w{0.0};
    std::atomic<double> state_z{0.0};

private:
    void run(const std::stop_token& stop) {
        // 组件的监听是在它自己的线程里起的，先重试着连。
        hcs_link::UniqueFd socket;
        while (!stop.stop_requested() && !socket) {
            if (auto connected_socket = hcs_link::connect(kEndpoint))
                socket = std::move(*connected_socket);
            else
                std::this_thread::sleep_for(5ms);
        }
        if (!socket)
            return;

        auto commands = hcs_link::Writer<AimCommand>::create({.capacity = 64, .lock_memory = false, .name = "vision"});
        ASSERT_TRUE(commands.has_value());
        const std::array fds{commands->fd()};
        auto received = hcs_link::exchange(socket.get(), fds, 2s);
        ASSERT_TRUE(received.has_value()) << received.error().message();
        ASSERT_EQ(received->count, 1U);
        auto state = hcs_link::Reader<GimbalState>::attach(std::move(received->fds[0]), {.lock_memory = false});
        ASSERT_TRUE(state.has_value()) << state.error().message();
        connected = true;

        std::uint64_t frame = 0;
        while (!stop.stop_requested()) {
            if (const auto sample = state->latest()) {
                state_w = sample->value.quaternion[0];
                state_z = sample->value.quaternion[3];
                saw_state = true;
            }
            const std::int64_t now = now_ns();
            commands->publish(AimCommand{
                .t_ref_ns = now,
                .frame_id = frame++,
                .azimuth = 0.4,
                .elevation = 0.05,
                .azimuth_rate = 0.0,
                .elevation_rate = 0.0,
                .azimuth_acceleration = 0.0,
                .elevation_acceleration = 0.0,
                .fire_from_ns = now,
                .fire_until_ns = now + 100'000'000,
                .flags = AimCommand::kHasTarget | AimCommand::kFire,
                .reserved = 0,
            });
            std::this_thread::sleep_for(5ms);
        }
    }

    std::jthread thread_;
};

class AutoAimLinkTest : public ::testing::Test {
protected:
    template <typename T>
    std::shared_ptr<T> add(const std::string& name) {
        const Component::NameScope scope{name};
        auto component = std::make_shared<T>();
        components_.push_back(component);
        return component;
    }

    /// 按 1 kHz 实时跑拍，直到 done() 为真或超时；返回是否等到了。
    template <typename Predicate>
    bool tick_until(const hcs_executor::Wiring& wiring, Predicate done, std::chrono::milliseconds timeout) {
        const auto deadline = hcs_sync::Clock::now() + timeout;
        while (hcs_sync::Clock::now() < deadline) {
            for (const auto& entry : wiring.latches)
                entry.latch(entry.interface);
            const hcs_sync::Tick tick{
                .scheduled = start_ + std::chrono::milliseconds{sequence_},
                .dt = std::chrono::milliseconds{1},
                .sequence = sequence_};
            ++sequence_;
            for (auto* component : wiring.updating_order) {
                component->update(tick);
                EXPECT_FALSE(component->failed());
            }
            if (done())
                return true;
            std::this_thread::sleep_until(start_ + std::chrono::milliseconds{sequence_});
        }
        return false;
    }

    std::vector<std::shared_ptr<Component>> components_;
    hcs_sync::Timestamp start_ = hcs_sync::Clock::now();
    std::uint64_t sequence_ = 0;
};

TEST_F(AutoAimLinkTest, VisionDrivesTheGimbalOutputsThroughTheChannels) {
    add<FakeImu>("fake_imu");
    add<hcs_core::controller::auto_aim::AutoAimLink>("auto_aim_link");
    const auto probe = add<Probe>("probe");

    auto wiring = hcs_executor::Linker::link(components_);
    ASSERT_TRUE(wiring.has_value()) << wiring.error().message;

    EXPECT_TRUE(tick_until(*wiring, [&] { return !probe->control; }, 50ms));

    auto vision = std::make_unique<FakeVision>();
    ASSERT_TRUE(tick_until(*wiring, [&] { return probe->control; }, 3s))
        << "graph never picked up the vision's command";

    // 视觉给的是 (0.4, 0.05)，零角速度：外推后方向不变。
    EXPECT_NEAR(probe->direction.x(), std::cos(0.05) * std::cos(0.4), 1e-9);
    EXPECT_NEAR(probe->direction.y(), std::cos(0.05) * std::sin(0.4), 1e-9);
    EXPECT_NEAR(probe->direction.z(), std::sin(0.05), 1e-9);
    EXPECT_TRUE(probe->shoot);

    // 反方向：视觉读到了控制写的云台姿态。
    ASSERT_TRUE(tick_until(*wiring, [&] { return vision->saw_state.load(); }, 1s));
    EXPECT_NEAR(vision->state_w.load(), std::cos(kImuYaw / 2), 1e-12);
    EXPECT_NEAR(vision->state_z.load(), std::sin(kImuYaw / 2), 1e-12);

    // 视觉退出：不再有新命令，stale_ticks 之后停用。
    vision.reset();
    EXPECT_TRUE(tick_until(*wiring, [&] { return !probe->control; }, 1s));
    EXPECT_FALSE(probe->shoot);

    // 视觉重启：新会话接上，重新跟。
    vision = std::make_unique<FakeVision>();
    EXPECT_TRUE(tick_until(*wiring, [&] { return probe->control; }, 3s));
}

} // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    setenv("ROS_LOG_DIR", "/tmp/hcs_test_ros_logs", 0);
    // 每个进程一个独立的 socket 名；单测进程不一定有足够的 RLIMIT_MEMLOCK。
    static const std::string endpoint = "endpoint:=" + kEndpoint;
    std::vector<char*> arguments{argv, argv + argc};
    for (const char* argument : {"--ros-args", "-p", endpoint.c_str(), "-p", "stale_ticks:=20", "-p",
                                 "lock_memory:=false", "-p", "state_capacity:=64"})
        arguments.push_back(const_cast<char*>(argument));
    rclcpp::init(static_cast<int>(arguments.size()), arguments.data());
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
