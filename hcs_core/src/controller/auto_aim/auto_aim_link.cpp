#include <pthread.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <thread>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <hcs_description/tf_description.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_link/autoaim.hpp>
#include <hcs_link/channel.hpp>
#include <hcs_link/quiescent_cell.hpp>
#include <hcs_link/rendezvous.hpp>
#include <hcs_sync/tick.hpp>
#include <hcs_utility/rt_attributes.hpp>

#include "controller/auto_aim/aim_follower.hpp"

namespace hcs_core::controller::auto_aim {

// 自瞄进程与控制图之间的桥。设计见 docs/autoaim-ipc-design.md。
//
// 两条通道，各由写它的一方建、封好再交出去，另一方只拿到只读视图：
//   GimbalState（本组件写）：每拍一条云台姿态；
//   AimCommand（视觉写）  ：本组件每拍取最新一条，外推到本拍，写成 /auto_aim/* 图输出，
//                          SimpleGimbalController 早就注册了这几个可选输入。
//
// 周期域只做通道读写（不分配、不加锁、不进内核）。接连接、互换 fd、换会话都在
// hcs-autoaim 线程里；新会话经 QuiescentCell 交给周期域，旧的等本拍结束才释放。
class AutoAimLink
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    AutoAimLink()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , endpoint_{parameter<std::string>("endpoint", std::string{hcs_link::autoaim::kEndpoint})}
        , follower_{AimFollower::Config{
              .stale_ticks = static_cast<std::uint32_t>(parameter<std::int64_t>("stale_ticks", 50)),
              .max_extrapolation_ns = parameter<std::int64_t>("max_extrapolation_ms", 60) * 1'000'000,
          }} {
        register_input("/gimbal/imu/quaternion", quaternion_);
        register_input("/gimbal/imu/angular_velocity", angular_velocity_);
        register_input("/gimbal/imu/online", imu_online_);

        register_output("/auto_aim/should_control", should_control_, false);
        register_output("/auto_aim/control_direction", control_direction_, kNoDirection);
        register_output("/auto_aim/should_shoot", should_shoot_, false);

        auto state = hcs_link::Writer<hcs_link::autoaim::GimbalState>::create({
            .capacity = static_cast<std::uint64_t>(parameter<std::int64_t>("state_capacity", 1024)),
            .lock_memory = parameter<bool>("lock_memory", true),
            .name = "hcs-autoaim-state",
        });
        if (!state)
            throw std::runtime_error{"AutoAimLink: cannot create the state channel: " + state.error().message()};
        state_.emplace(std::move(*state));

        link_thread_ = std::jthread{[this](const std::stop_token& stop) { serve(stop); }};
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        const std::int64_t now_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(tick.scheduled.time_since_epoch()).count();

        const Eigen::Quaterniond& quaternion = *quaternion_;
        const Eigen::Vector3d& angular_velocity = *angular_velocity_;
        state_->publish(hcs_link::autoaim::GimbalState{
            .tick_ns = now_ns,
            .tick_sequence = tick.sequence,
            .quaternion = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()},
            .angular_velocity = {angular_velocity.x(), angular_velocity.y(), angular_velocity.z()},
            .imu_online = *imu_online_ ? 1U : 0U,
            .reserved = 0,
        });

        const auto decision = follower_.update(commands_.acquire(), now_ns);
        commands_.quiescent(); // 此后不再碰本拍 acquire 到的 Reader

        *should_control_ = decision.control;
        *should_shoot_ = decision.shoot;
        *control_direction_ =
            decision.control
                ? OdomImu::DirectionVector{decision.direction[0], decision.direction[1], decision.direction[2]}
                : kNoDirection;
    }

private:
    using OdomImu = hcs_description::OdomImu;
    using CommandReader = AimFollower::CommandReader;

    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    inline static const OdomImu::DirectionVector kNoDirection{kNan, kNan, kNan};

    template <typename T>
    T parameter(const std::string& name, const T& fallback) {
        T value{};
        get_parameter_or(name, value, fallback);
        return value;
    }

    /// hcs-autoaim 线程：等视觉连上来，互换通道，把新会话交给周期域。新连接直接顶掉旧的——
    /// 视觉 fork 过的话旧连接的 EOF 可能永远不来，只看"有人来连"最简单也最可靠。
    void serve(const std::stop_token& stop) {
        ::pthread_setname_np(::pthread_self(), "hcs-autoaim");

        auto listener = hcs_link::Listener::bind(endpoint_);
        if (!listener) {
            RCLCPP_ERROR(get_logger(), "auto-aim link disabled, cannot listen on '@%s': %s",
                         endpoint_.c_str(), listener.error().message().c_str());
            return;
        }

        hcs_link::UniqueFd session;
        while (!stop.stop_requested()) {
            if (session && hcs_link::peer_closed(session.get())) {
                RCLCPP_WARN(get_logger(), "vision disconnected");
                session.reset();
                install(nullptr);
            }

            auto peer = listener->accept(std::chrono::milliseconds{100});
            if (!peer) {
                if (peer.error().code != hcs_link::ErrorCode::kTimeout)
                    RCLCPP_WARN(get_logger(), "accept: %s", peer.error().message().c_str());
                continue;
            }

            const std::array fds{state_->fd()};
            auto received = hcs_link::exchange(peer->get(), fds, std::chrono::seconds{1});
            if (!received || received->count != 1) {
                RCLCPP_WARN(get_logger(), "handshake failed: %s",
                            received ? "vision sent no command channel" : received.error().message().c_str());
                continue;
            }

            auto reader = CommandReader::attach(std::move(received->fds[0]));
            if (!reader) {
                RCLCPP_WARN(get_logger(), "rejected the command channel: %s", reader.error().message().c_str());
                continue;
            }

            install(std::make_unique<CommandReader>(std::move(*reader)));
            session = std::move(*peer);
            RCLCPP_INFO(get_logger(), "vision connected on '@%s'", endpoint_.c_str());
        }
    }

    void install(std::unique_ptr<CommandReader> reader) {
        if (!commands_.replace(std::move(reader), std::chrono::milliseconds{200}))
            RCLCPP_WARN(get_logger(), "control loop is not ticking; the previous command channel is leaked");
    }

    std::string endpoint_;
    AimFollower follower_;

    InputInterface<Eigen::Quaterniond> quaternion_;
    InputInterface<Eigen::Vector3d> angular_velocity_;
    InputInterface<bool> imu_online_;

    OutputInterface<bool> should_control_;
    OutputInterface<OdomImu::DirectionVector> control_direction_;
    OutputInterface<bool> should_shoot_;

    std::optional<hcs_link::Writer<hcs_link::autoaim::GimbalState>> state_;
    hcs_link::QuiescentCell<CommandReader> commands_;

    // 必须是最后一个成员：析构时最先停下，之后才轮到它用到的 state_ 和 commands_。
    std::jthread link_thread_;
};

} // namespace hcs_core::controller::auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::controller::auto_aim::AutoAimLink, hcs_executor::Component)
