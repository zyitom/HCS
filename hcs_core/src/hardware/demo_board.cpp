#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/timer.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_sync/snapshot.hpp>
#include <hcs_sync/tick.hpp>
#include <hcs_utility/rt_attributes.hpp>

#include "hardware/device/fake_motor.hpp"

namespace hcs_demo::hardware {

// ============================================================================
// 硬件层组件，对应真实工程里的 hcs_core::hardware::OmniInfantry 等。
//
// 一个硬件包 = 两个 Component：
//   DemoBoard         无 input，dependency_count == 0，最先跑：读硬件 -> 写反馈
//   DemoBoard::Command 只有 input，依赖所有控制器，最后跑：读控制量 -> 写硬件
//
// 不拆的话「电机反馈 -> PID -> 电机指令」在拓扑排序里就是一个环，启动即 fatal。
// ============================================================================
class DemoBoard
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    DemoBoard()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , demo_command_(
              create_partner_component<DemoCommand>(get_component_name() + "_command", *this))
        , motor_(*this, *demo_command_, "/demo/motor") {

        motor_.configure(
            device::FakeMotor::Config{}
                .set_max_torque(get_parameter("motor_max_torque").as_double())
                .set_inertia(get_parameter("motor_inertia").as_double())
                .set_damping(get_parameter("motor_damping").as_double()));

        setpoint_frequency_ = get_parameter("setpoint_frequency").as_double();
        setpoint_amplitude_ = get_parameter("setpoint_amplitude").as_double();

        // 遥测要报角度，而角度是 motor_ 注册在本组件上的 output；
        // 自己读自己的 output 会在拓扑排序里变成自环，所以从 command 侧读。
        demo_command_->register_input("/demo/motor/angle", motor_angle_);

        // 假装这是遥控器摇杆算出来的目标速度
        register_output("/demo/motor/velocity_setpoint", velocity_setpoint_, 0.0);

        // 遥测日志的读侧。定时器跑在 rcl_executor.spin() 那条线程上，也就是尽力域（D3）：
        // 这里分配、格式化、写 stdout 都是允许的，热路径里一样都不允许。
        telemetry_timer_ = create_wall_timer(std::chrono::seconds{1}, [this] {
            const auto reading = telemetry_.read(hcs_sync::Clock::now());
            if (!reading.valid)
                return;

            RCLCPP_INFO(
                get_logger(), "[%s] setpoint=% .3f  velocity=% .3f rad/s  angle=% .3f rad",
                get_component_name().c_str(), reading.value.setpoint, reading.value.velocity,
                reading.value.angle);
        });
    }

    ~DemoBoard() override = default;

    // 主循环启动前调用一次。真机上用来做握手 / 请求首次同步读。
    //
    // dt 现在从 update() 的 Tick 参数来，不再需要在这里预取 /predefined/update_rate ——
    // 「此刻 update_rate 还是 0」那个坑（ARCH-9）随之消失。
    void before_updating() override {
        RCLCPP_INFO(get_logger(), "[%s] hardware ready", get_component_name().c_str());
    }

    // status 侧：每周期最先执行
    //
    // 周期域的两条硬规矩，这个函数是它们的样板：
    //   1. dt 只许来自 tick.dt，禁止 1.0 / update_rate —— update_rate 是名义值，
    //      跳拍时它不变，dt 变。
    //   2. 参考轨迹只许用 tick.scheduled 推，禁止用「计数 × dt」——
    //      跳一拍，整条正弦就被静默地拉长一个周期，日志里只看到 skipped 涨了，
    //      波形上只是「慢了一点点」，根本看不出来。
    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        // 相位原点只能在首拍锁定：before_updating() 时还没有任何 Tick。
        if (!started_) {
            start_time_ = tick.scheduled;
            started_    = true;
        }

        motor_.update_status(tick.dt_seconds());

        const double t = std::chrono::duration<double>(tick.scheduled - start_time_).count();
        *velocity_setpoint_ =
            setpoint_amplitude_ * std::sin(2 * std::numbers::pi * setpoint_frequency_ * t);
    }

private:
    struct Telemetry {
        double setpoint, velocity, angle;
    };

    // command 侧：每周期最后执行
    void command_update(const hcs_sync::Tick& tick) {
        motor_.generate_command();

        // 三个原语怎么用的最小示范：这就是一条跨域通道。
        // 周期域（D2）写，尽力域（D3）读，两侧都 wait-free ——
        // 写侧不会被读侧拖住，读侧永远有值、永远不阻塞、永远返回（可能是上一拍的值，这是设计）。
        // 反过来直接在这里 RCLCPP_INFO 就是错的：格式化 + 分配 + 可能的 write 系统调用，
        // 标了 HCS_NONBLOCKING 之后 clang 会当场把它报出来。
        telemetry_.publish(
            Telemetry{*velocity_setpoint_, motor_.velocity(), *motor_angle_}, tick.scheduled);
    }

    class DemoCommand : public hcs_executor::Component {
    public:
        explicit DemoCommand(DemoBoard& board)
            : board_(board) {}

        void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
            board_.command_update(tick);
        }

    private:
        DemoBoard& board_;
    };

    double setpoint_frequency_ = 0.2;
    double setpoint_amplitude_ = 30.0;

    hcs_sync::Timestamp start_time_{};
    bool started_ = false;

    // 声明顺序即构造顺序：demo_command_ 必须在 motor_ 之前，
    // 因为 motor_ 的构造函数要拿 *demo_command_ 去注册 input。
    std::shared_ptr<DemoCommand> demo_command_;
    device::FakeMotor motor_;

    InputInterface<double> motor_angle_;

    OutputInterface<double> velocity_setpoint_;

    // telemetry_ 必须声明在 telemetry_timer_ 之前：析构是逆序，定时器先死，
    // 回调不会再碰到已经析构的 telemetry_。
    hcs_sync::Snapshot<Telemetry> telemetry_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
};

} // namespace hcs_demo::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_demo::hardware::DemoBoard, hcs_executor::Component)
