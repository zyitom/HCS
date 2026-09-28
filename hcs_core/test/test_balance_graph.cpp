// 平衡组件图的构建测试：假硬件 + 全部平衡组件跑一次 Linker::link，
// 确认无环、必需输入都有提供者，然后空跑 100 拍确认没有任何组件被隔离。
//
// 组件构造需要 ROS 参数：gtest main 里 rclcpp::init 时挂上
// test/balance_graph_test.yaml（节点名与测试里的组件实例名一致）。

#include <array>
#include <chrono>
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

// 组件类是单文件类（.cpp 里定义 + 底部 PLUGINLIB 导出），测试直接包含编译单元。
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
using hcs_executor::Wiring;

/// 假硬件（状态侧）：注册整车硬件组件对外提供的全部反馈输出，没有输入
/// ——与真实 BalanceInfantry 的 update() 相同的图语义。
class FakeBalanceHardware : public Component {
public:
    FakeBalanceHardware() {
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


        register_output("/chassis/imu/pitch", imu_pitch_, 0.0);
        register_output("/chassis/imu/roll", imu_roll_, 0.0);
        register_output("/chassis/imu/yaw", imu_yaw_, 0.0);
        register_output("/chassis/imu/pitch_rate", imu_pitch_rate_, 0.0);
        register_output("/chassis/imu/roll_rate", imu_roll_rate_, 0.0);
        register_output("/chassis/imu/yaw_rate", imu_yaw_rate_, 0.0);
        register_output("/chassis/imu/acceleration", imu_acceleration_, Eigen::Vector3d::Zero());
    }

    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {}

private:
    void register_joint_interfaces(const std::string& joint) {
        const std::string prefix = "/chassis/" + joint + "_joint";
        const auto index = joint_count_++;
        register_output(prefix + "/angle", joint_angles_[index], 1.5);
        register_output(prefix + "/velocity", joint_velocities_[index], 0.0);
        register_output(prefix + "/torque", joint_torque_feedbacks_[index], 0.0);
    }

    std::size_t joint_count_ = 0;

    OutputInterface<Eigen::Vector2d> joystick_;
    OutputInterface<hcs_msgs::Keyboard> keyboard_;
    OutputInterface<hcs_msgs::Switch> switch_left_;
    OutputInterface<double> gimbal_yaw_angle_;
    std::array<OutputInterface<double>, 4> joint_angles_{};
    std::array<OutputInterface<double>, 4> joint_velocities_{};
    std::array<OutputInterface<double>, 4> joint_torque_feedbacks_{};
    std::array<InputInterface<double>, 4> joint_torques_{};
    OutputInterface<double> left_wheel_velocity_;
    OutputInterface<double> right_wheel_velocity_;
    OutputInterface<double> imu_pitch_;
    OutputInterface<double> imu_roll_;
    OutputInterface<double> imu_yaw_;
    OutputInterface<double> imu_pitch_rate_;
    OutputInterface<double> imu_roll_rate_;
    OutputInterface<double> imu_yaw_rate_;
    OutputInterface<Eigen::Vector3d> imu_acceleration_;
    InputInterface<double> left_wheel_torque_;
    InputInterface<double> right_wheel_torque_;
};

/// 假硬件（指令侧）：对应真实的 Command 伙伴，消费全部控制力矩。
class FakeBalanceCommand : public Component {
public:
    FakeBalanceCommand() {
        register_input("/chassis/left_wheel/control_torque", left_wheel_torque_, false);
        register_input("/chassis/right_wheel/control_torque", right_wheel_torque_, false);
        register_input("/chassis/left_front_joint/control_torque", left_front_torque_, false);
        register_input("/chassis/left_back_joint/control_torque", left_back_torque_, false);
        register_input("/chassis/right_front_joint/control_torque", right_front_torque_, false);
        register_input("/chassis/right_back_joint/control_torque", right_back_torque_, false);
    }

    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        // 力矩收进来丢弃：链路打通即可。
        (void)left_wheel_torque_;
        (void)right_wheel_torque_;
        (void)left_front_torque_;
        (void)left_back_torque_;
        (void)right_front_torque_;
        (void)right_back_torque_;
    }

private:
    InputInterface<double> left_wheel_torque_;
    InputInterface<double> right_wheel_torque_;
    InputInterface<double> left_front_torque_;
    InputInterface<double> left_back_torque_;
    InputInterface<double> right_front_torque_;
    InputInterface<double> right_back_torque_;
};

class BalanceGraphTest : public ::testing::Test {
protected:
    template <typename T>
    std::shared_ptr<T> add(const std::string& name) {
        const Component::NameScope scope{name};
        auto component = std::make_shared<T>();
        components_.push_back(component);
        return component;
    }

    std::vector<std::shared_ptr<Component>> components_;
};

TEST_F(BalanceGraphTest, links_without_cycles_and_missing_inputs) {
    add<FakeBalanceHardware>("balance_hardware");
    add<FakeBalanceCommand>("balance_hardware_command");
    add<hcs::BalanceStateEstimator>("balance_state_estimator");
    add<hcs::BalanceChassisController>("balance_chassis_controller");
    add<hcs::BalanceModeManager>("balance_mode_manager");
    add<hcs::BalanceLqrController>("balance_lqr_controller");
    add<hcs::LegForceController>("leg_force_controller");
    add<hcs::LegJointController>("left_leg_joint_controller");
    add<hcs::LegJointController>("right_leg_joint_controller");

    auto wiring = Linker::link(components_);
    ASSERT_TRUE(wiring.has_value()) << wiring.error().message;

    // 轮/关节力矩输出都有人消费（假硬件收尾）。
    for (const auto* component : wiring->updating_order)
        EXPECT_FALSE(component->failed());
}

TEST_F(BalanceGraphTest, hundred_ticks_without_isolation) {
    add<FakeBalanceHardware>("balance_hardware");
    add<FakeBalanceCommand>("balance_hardware_command");
    add<hcs::BalanceStateEstimator>("balance_state_estimator");
    add<hcs::BalanceChassisController>("balance_chassis_controller");
    add<hcs::BalanceModeManager>("balance_mode_manager");
    add<hcs::BalanceLqrController>("balance_lqr_controller");
    add<hcs::LegForceController>("leg_force_controller");
    add<hcs::LegJointController>("left_leg_joint_controller");
    add<hcs::LegJointController>("right_leg_joint_controller");

    auto wiring = Linker::link(components_);
    ASSERT_TRUE(wiring.has_value()) << wiring.error().message;

    hcs_sync::Timestamp start = hcs_sync::Clock::now();
    std::uint64_t sequence = 0;
    for (int i = 0; i < 100; ++i) {
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

        for (const auto* component : wiring->updating_order)
            ASSERT_FALSE(component->failed()) << "component " << component->get_component_name()
                                              << " threw at tick " << sequence;
        start = hcs_sync::Clock::now();
    }
}

} // namespace

// 组件构造依赖 ROS 参数：挂上与本包一起维护的测试参数文件。
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // 沙箱/CI 里 ~/.ros 常不可写，缺省值给 /tmp。
    setenv("ROS_LOG_DIR", "/tmp/hcs_test_ros_logs", 0);
    std::vector<char*> arguments{argv, argv + argc};
    static const char params_flag[] = "--params-file";
    static const std::string params_path = BALANCE_GRAPH_TEST_YAML;
    arguments.push_back(const_cast<char*>("--ros-args"));
    arguments.push_back(const_cast<char*>(params_flag));
    arguments.push_back(const_cast<char*>(params_path.c_str()));
    rclcpp::init(static_cast<int>(arguments.size()), arguments.data());
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
