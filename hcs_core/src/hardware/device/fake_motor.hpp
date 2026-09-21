#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <string>

#include <rmcs_executor/component.hpp>

namespace rmcs_demo::hardware::device {

// 设备类的标准写法，对应真实工程里的 device::DjiMotor / device::LkMotor：
// 设备本身不是 Component，构造时同时向 status 组件注册 output、
// 向 command 组件注册 input，把「读」和「写」拆到调度图的两端。
class FakeMotor {
public:
    struct Config {
        double max_torque = 3.0;
        double inertia    = 0.01;
        double damping    = 0.02;

        Config& set_max_torque(double v) { return max_torque = v, *this; }
        Config& set_inertia(double v) { return inertia = v, *this; }
        Config& set_damping(double v) { return damping = v, *this; }
    };

    FakeMotor(
        rmcs_executor::Component& status_component,
        rmcs_executor::Component& command_component, const std::string& name_prefix) {

        status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);

        // required = false：控制器没接上时不算错误，对应真机「硬件先跑起来」的调试流程
        command_component.register_input(
            name_prefix + "/control_torque", control_torque_, false);
    }

    FakeMotor(const FakeMotor&)            = delete;
    FakeMotor& operator=(const FakeMotor&) = delete;
    FakeMotor(FakeMotor&&)                 = delete;
    FakeMotor& operator=(FakeMotor&&)      = delete;

    void configure(const Config& config) { config_ = config; }

    // status 侧调用：推进一步仿真并写出反馈。dt 由调用方从 tick.dt_seconds() 传入。
    // 用的是上一周期 command 侧锁存的力矩 —— 真机上指令发出去也要下一帧才收得到反馈。
    void update_status(double dt) {
        const double torque = latched_torque_.load(std::memory_order::relaxed);

        velocity_ += (torque - config_.damping * velocity_) / config_.inertia * dt;
        angle_ += velocity_ * dt;

        constexpr double kTwoPi = 2 * 3.14159265358979323846;
        angle_ -= kTwoPi * std::floor((angle_ + kTwoPi / 2) / kTwoPi);

        *angle_output_      = angle_;
        *velocity_output_   = velocity_;
        *max_torque_output_ = config_.max_torque;
    }

    // command 侧调用：读控制量并「发给硬件」。真机上这里是往 CAN builder 里塞一帧。
    void generate_command() {
        double torque = *control_torque_;
        if (!std::isfinite(torque))
            torque = 0.0;
        latched_torque_.store(
            std::clamp(torque, -config_.max_torque, config_.max_torque),
            std::memory_order::relaxed);
    }

    [[nodiscard]] double velocity() const { return velocity_; }

private:
    Config config_{};

    double angle_    = 0.0;
    double velocity_ = 0.0;

    // 本示例里 status 和 command 都跑在周期域同一条线程上，这个 atomic 是多余的 ——
    // 白付一次总线开销，只为了让示例和真机同构，所以留着。
    // 真机上跨域（USB 事件线程写、执行器线程读）的正解是 Snapshot<T>，不是散落的 atomic：
    // std::atomic 出现在控制律或设备类里是一个信号（说明你正在跨域），不是一个解法。
    std::atomic<double> latched_torque_{0.0};

    rmcs_executor::Component::OutputInterface<double> angle_output_;
    rmcs_executor::Component::OutputInterface<double> velocity_output_;
    rmcs_executor::Component::OutputInterface<double> max_torque_output_;

    rmcs_executor::Component::InputInterface<double> control_torque_;
};

} // namespace rmcs_demo::hardware::device
