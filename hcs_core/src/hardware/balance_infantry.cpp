#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#include <libhcs/board/hcs_can_port.hpp>
#include <libhcs/board/hpm5321.hpp>
#include <libhcs/data/datas.hpp>
#include <libhcs/protocol/handler.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/timer.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_sync/tick.hpp>
#include <hcs_utility/machine_guard.hpp>
#include <hcs_utility/rt_attributes.hpp>
#include <hcs_utility/thread_config.hpp>

#include <hcs_description/tf_description.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/hipnuc.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/vt13.hpp"
#include "hardware/util/board_transmitter.hpp"
#include "hardware/util/safety_latch.hpp"

namespace hcs_core::hardware {
namespace {

using libhcs::board::hcs::CanPort;

// 板卡下标 = 发送线程板表下标
enum class Board : std::uint8_t { kGimbal, kChassis, kAux };

// 每块板的名字与 UART0 外设波特率（硬件事实，改接线才动）
struct BoardSpec {
    const char* name;
    std::uint32_t uart0_baudrate;
};
constexpr std::array<BoardSpec, 3> kBoards{{
    {"gimbal", 921600},  // 云台 CH040
    {"chassis", 921600}, // 底盘 CH040
    {"aux", 115200},     // VT13（待上台架核对）
}};

constexpr std::size_t board_index(Board board) { return std::to_underlying(board); }

struct Bus {
    Board board;
    CanPort port;
    bool operator==(const Bus&) const = default;
};

constexpr Bus kGimbalCan1{Board::kGimbal, CanPort::kCan1};
constexpr Bus kGimbalCan2{Board::kGimbal, CanPort::kCan2};
constexpr Bus kChassisCan1{Board::kChassis, CanPort::kCan1};
constexpr Bus kChassisCan2{Board::kChassis, CanPort::kCan2};
constexpr Bus kAuxCan1{Board::kAux, CanPort::kCan1};

// 腿关节 DM：四条腿刷成同一组 PMAX/VMAX/TMAX（寄存器镜像，与实物不符会静默缩放，上车前读回核对）。
// 驱动层不反转（左腿在估计器取负、右腿在 LegJointController 取负），多圈对齐 Helios MTTurnProc
device::DmMotor::Config leg_joint(std::uint32_t esc_id, std::uint32_t master_id, double zero) {
    device::DmMotor::Config config{device::DmMotor::Type::kJ4310, esc_id, master_id};
    config.set_position_max(6.283185)
        .set_velocity_max(45.0)
        .set_torque_max(40.0)
        .set_zero_angle(zero) // Helios org_pos
        .enable_multi_turn_angle();
    return config;
}

using ImuFrame = device::Hipnuc::Config::ModuleFrame;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

// IMU 标量话题表，一行一个话题：下游 PID / 估计器只吃 double，这里写明取 Hipnuc 的
// 哪个量、哪个分量。Hipnuc 输出已是 FLU，欧拉角分量顺序是 roll/pitch/yaw。
struct ImuScalar {
    const char* topic;
    const Eigen::Vector3d& (device::Hipnuc::*vector)() const;
    Eigen::Index axis; // 0/1/2 = x/y/z = roll/pitch/yaw
};

constexpr auto kEuler = &device::Hipnuc::euler_angles;
constexpr auto kGyro = &device::Hipnuc::angular_velocity;

constexpr std::array kGimbalImuScalars{
    ImuScalar{"/gimbal/yaw/velocity_imu", kGyro, 2},
    ImuScalar{"/gimbal/pitch/velocity_imu", kGyro, 1},
};

constexpr std::array kChassisImuScalars{
    ImuScalar{"/chassis/imu/roll", kEuler, 0},
    ImuScalar{"/chassis/imu/pitch", kEuler, 1},
    ImuScalar{"/chassis/imu/yaw", kEuler, 2},
    ImuScalar{"/chassis/imu/roll_rate", kGyro, 0},
    ImuScalar{"/chassis/imu/pitch_rate", kGyro, 1},
    ImuScalar{"/chassis/imu/yaw_rate", kGyro, 2},
};

// 适配器：按表把一个 Hipnuc 的向量量拆成标量输出，注册和发布走同一张表。
// 初值 NaN 即组件隔离时的复位值。
template <const auto& kScalars>
class ImuScalarOutputs {
public:
    ImuScalarOutputs(hcs_executor::Component& component, const device::Hipnuc& imu)
        : imu_{imu} {
        for (auto&& [scalar, output] : std::views::zip(kScalars, outputs_))
            component.register_output(scalar.topic, output, kNan);
    }

    void publish() HCS_NONBLOCKING {
        for (auto&& [scalar, output] : std::views::zip(kScalars, outputs_))
            *output = std::invoke(scalar.vector, imu_)[scalar.axis];
    }

private:
    const device::Hipnuc& imu_;
    std::array<hcs_executor::Component::OutputInterface<double>, kScalars.size()> outputs_;
};

} // namespace

// ============================================================================
// 平衡步兵整车硬件组件：三块 Hpm5321，接线表见类尾（一台电机一行）。
// 分发、打包、全失能批次、CAN 路数与负载估算都从接线表推出，
// 新增电机只改接线表和 motors() 两处。yaml 只放部署项（板卡使能/串口/线程）。
// 发送与安全：Command 伙伴打包定长批次交给三板共用发送线程；DM 遇 NaN 发 0xFD，
// LK 遇 NaN 自动失能，DJI 遇 NaN 发 0 电流；隔离后只补发一次全失能批次。
// 关键设备（腿、轮、底盘 IMU）上过线后掉线或报故障 → 锁存全失能，左拨杆 DOWN 再拨离复位。
// ============================================================================

class BalanceInfantry
    : public hcs_executor::Component
    , public rclcpp::Node {
    // 接线表里的全部电机（新增电机在这里加一项）
    auto motors(this auto& self) {
        return std::tie(
            self.yaw_, self.dial_, self.pitch_, self.friction_left_, self.friction_right_,
            self.left_front_joint_, self.left_back_joint_, self.right_front_joint_,
            self.right_back_joint_, self.left_wheel_, self.right_wheel_);
    }

    void for_each_motor(this auto& self, auto&& f) {
        std::apply([&](auto&... motor) { (f(motor), ...); }, self.motors());
    }

    // 失效即无法平衡的设备，顺序即 kCriticalNames 与 SafetyLatch::tripped_device() 的下标。
    // 云台、拨盘、摩擦轮不在此列：它们坏了车仍能站住，交给各自的控制器处理。
    static constexpr std::array kCriticalNames{
        "/chassis/left_front_joint", "/chassis/left_back_joint", "/chassis/right_front_joint",
        "/chassis/right_back_joint", "/chassis/left_wheel",      "/chassis/right_wheel",
        "/chassis/imu",
    };

    template <class Device>
    static util::DeviceHealth health(const Device& device) {
        return {.received = device.received(), .online = device.online(), .faulted = device.faulted()};
    }

    void update_critical_health() {
        const util::DeviceHealth devices[] = {
            health(left_front_joint_), health(left_back_joint_),  health(right_front_joint_),
            health(right_back_joint_), health(left_wheel_),       health(right_wheel_),
            {.received = chassis_imu_.received(), .online = chassis_imu_.online(), .faulted = false},
        };
        static_assert(std::size(devices) == kCriticalNames.size());
        safety_latch_.update(devices, remote_control_.switch_left());
    }

public:
    BalanceInfantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        remote_control_.register_vt13(&vt13_);
        for_each_motor([this]<class Driver>(const OnBus<Driver>& motor) {
            if constexpr (std::same_as<Driver, device::DmMotor>)
                log_dm_register_mirror(motor);
        });

        register_output("/tf", tf_);

        // 板卡构造期就可能回调，放在所有它会碰的数据都就位之后
        for (const auto board : {Board::kGimbal, Board::kChassis, Board::kAux})
            create_board(board);
        check_classic_can();

        transmitter_ = std::make_unique<util::BoardTransmitter>(
            std::array{boards_[0].get(), boards_[1].get(), boards_[2].get()},
            hcs_utility::ThreadConfig{
                param<std::string>("tx_thread_config", ""), get_component_name() + "-tx"},
            std::chrono::milliseconds{param<std::int64_t>("tx_stale_after_ms", 10)});
        transmitter_->set_safe_batch(build_safe_batch());
        report_bus_load();

        static std::once_flag machine_guard_once;
        std::call_once(machine_guard_once, [this]() {
            for (const auto& finding : hcs_utility::MachineGuard::run()) {
                const char* text = finding.text.c_str();
                switch (finding.level) {
                case hcs_utility::MachineGuard::Level::kCritical:
                    RCLCPP_ERROR(get_logger(), "[machine] %s", text);
                    break;
                case hcs_utility::MachineGuard::Level::kWarn:
                    RCLCPP_WARN(get_logger(), "[machine] %s", text);
                    break;
                default: RCLCPP_INFO(get_logger(), "[machine] %s", text); break;
                }
            }
        });

        report_timer_ = create_wall_timer(std::chrono::seconds{1}, [this] { report(); });
    }

    hcs_utility::Doorbell* tick_end_doorbell() override {
        return transmitter_ ? &transmitter_->doorbell() : nullptr;
    }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        // 回调线程只存原始帧，这里解成物理量写进输出
        for_each_motor([](auto& motor) { motor.update_status(); });
        gimbal_imu_.update_status();
        chassis_imu_.update_status();
        vt13_.update_status(tick.scheduled);

        remote_control_.update();
        update_imu();
        update_critical_health();
    }

private:
    // 接线表的一行：驱动 + 所在总线 + 话题名 + 构造期配置（留给日志）
    template <class Driver>
    struct OnBus : Driver {
        OnBus(
            BalanceInfantry& owner, Bus bus, const char* name,
            const typename Driver::Config& config)
            : Driver(owner, *owner.command_, name, config)
            , bus(bus)
            , name(name)
            , config(config) {}

        const Bus bus;
        const char* const name;
        const typename Driver::Config config;
    };

    class BalanceInfantryCommand : public hcs_executor::Component {
    public:
        explicit BalanceInfantryCommand(BalanceInfantry& owner)
            : owner_(owner) {}

        void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
            owner_.command_update(tick);
        }

    private:
        BalanceInfantry& owner_;
    };

    class BoardCallback final : public libhcs::board::Hpm5321::Callback {
    public:
        BoardCallback(BalanceInfantry& owner, Board board)
            : owner_(owner)
            , board_(board) {}

        void can_receive(CanPort port, const libhcs::data::CanDataView& data) override {
            owner_.on_can_receive(board_, port, data);
        }

        void uart0_receive_callback(const libhcs::data::UartDataView& data) override {
            owner_.on_uart0_receive(board_, data);
        }

    private:
        BalanceInfantry& owner_;
        Board board_;
    };

    // ── 反馈分发（libhcs IO 线程，事件域）──────────────────────────────────
    void on_can_receive(
        Board board, CanPort port, const libhcs::data::CanDataView& data) noexcept {
        if (data.is_extended_can_id || data.is_remote_transmission)
            return;
        const Bus bus{board, port};
        std::apply(
            [&](auto&... motor) {
                (void)((motor.bus == bus && motor.match_then_store_status(data.can_id, data.can_data))
                       || ...);
            },
            motors());
    }

    void on_uart0_receive(Board board, const libhcs::data::UartDataView& data) noexcept {
        switch (board) {
        case Board::kGimbal: gimbal_imu_.store_status(data.uart_data); break;
        case Board::kChassis: chassis_imu_.store_status(data.uart_data); break;
        case Board::kAux: vt13_.store_status(data.uart_data); break;
        }
    }

    // ── 指令侧（Command 伙伴，周期域）────────────────────────────────────
    // 锁存期间整拍改发全失能帧，与构造期安全批次同一套 command_frame(safe=true)。
    // 仍每拍发：发送线程只在 fresh 时发，停发就回到"不发任何指令"，而 DM 失能帧
    // 本身也是它回反馈的唯一理由——停发之后连复位要看的 online 都拿不到了。
    void command_update(const hcs_sync::Tick& tick) HCS_NONBLOCKING {
        util::TransmitBatch batch;
        batch.sequence = ++batch_sequence_;
        pack_frames(batch, {.safe = safety_latch_.latched(), .all_boards = false});
        transmitter_->publish(batch, tick.scheduled);
    }

    struct PackOptions {
        bool safe;       ///< 全失能帧
        bool all_boards; ///< 连禁用板也装（构造期安全批次、负载估算：按整张接线表算）
    };

    // 按接线表打一拍的帧
    void pack_frames(util::TransmitBatch& batch, PackOptions options) {
        const bool safe = options.safe;
        struct DjiFrame {
            Bus bus;
            std::uint32_t can_id;
            device::CanPacket8 packet;
        };
        std::array<DjiFrame, 8> dji_frames{}; // 上限 = DJI 电机数
        std::size_t dji_count = 0;

        for_each_motor([&]<class Driver>(OnBus<Driver>& motor) {
            if (!options.all_boards && !boards_[board_index(motor.bus.board)])
                return;
            if constexpr (std::same_as<Driver, device::DjiMotor>) {
                // 同总线同 send_id 的 DJI 合成一帧四合一，槽位由驱动按 id 落
                const auto used = std::span{dji_frames}.first(dji_count);
                auto* frame = std::to_address(std::ranges::find_if(used, [&](const DjiFrame& f) {
                    return f.bus == motor.bus && f.can_id == motor.send_id();
                }));
                if (frame == used.data() + used.size()) {
                    frame = &dji_frames[dji_count++];
                    *frame = {motor.bus, motor.send_id(), device::CanPacket8{std::uint64_t{0}}};
                }
                if (!safe)
                    frame->packet << motor;
            } else {
                util::push_frame(
                    batch, std::to_underlying(motor.bus.board), motor.bus.port, motor.send_id(),
                    command_frame(motor, safe));
            }
        });

        for (const auto& frame : std::span{dji_frames}.first(dji_count))
            util::push_frame(
                batch, std::to_underlying(frame.bus.board), frame.bus.port, frame.can_id,
                frame.packet);
    }

    // DM：torque 为 NaN（失能/摔倒/隔离复位）发 0xFD，否则交给驱动使能与清错
    static device::CanPacket8 command_frame(device::DmMotor& motor, bool safe) {
        if (safe || std::isnan(motor.control_torque()))
            return device::DmMotor::generate_disable_command();
        return motor.generate_command();
    }

    // LK：只发力矩帧（0xA1），NaN 时驱动自己发 0 电流。不走 generate_command()：
    // yaw 同时接了 control_velocity（角度环输出），它会按优先级发 0xA2 速度闭环，
    // 把速度环交给电机、把 control_torque 当成电流限幅，绕过我们的速度 PID。
    static device::CanPacket8 command_frame(device::LkMotor& motor, bool safe) {
        return safe ? device::LkMotor::generate_disable_command() : motor.generate_torque_command();
    }

    util::TransmitBatch build_safe_batch() {
        util::TransmitBatch batch;
        pack_frames(batch, {.safe = true, .all_boards = true});
        return batch;
    }

    // ── IMU 桥接 ─────────────────────────────────────────────────────────
    void update_imu() HCS_NONBLOCKING {
        gimbal_imu_scalars_.publish();
        chassis_imu_scalars_.publish();

        tf_->set_state<hcs_description::GimbalCenterLink, hcs_description::YawLink>(yaw_.angle());
        tf_->set_state<hcs_description::YawLink, hcs_description::PitchLink>(pitch_.angle());
        tf_->set_transform<hcs_description::PitchLink, hcs_description::OdomImu>(
            gimbal_imu_.quaternion());
    }

    // DM 的 PMAX/VMAX/TMAX 与出厂默认对照打印；不一致 = 被刷过，上车须读回（0x7FF/RID 0x33）
    void log_dm_register_mirror(const OnBus<device::DmMotor>& motor) const {
        const auto& config = motor.config;
        const device::DmMotor::Config factory{config.motor_type};
        const bool reflashed = std::abs(config.position_max - factory.position_max) > 1e-9
                            || std::abs(config.velocity_max - factory.velocity_max) > 1e-9
                            || std::abs(config.torque_max - factory.torque_max) > 1e-9;
        RCLCPP_INFO(
            get_logger(), "[%s] DM %s: PMAX=%.6g VMAX=%.6g TMAX=%.6g (factory %.6g/%.6g/%.6g)%s",
            get_component_name().c_str(), motor.name, config.position_max, config.velocity_max,
            config.torque_max, factory.position_max, factory.velocity_max, factory.torque_max,
            reflashed ? " -- RE-FLASHED: read back registers (0x7FF/RID 0x33) at bringup"
                      : " (stock, factory defaults)");
    }

    void create_board(Board board) {
        const auto [name, uart0_baudrate] = kBoards[board_index(board)];
        const auto key = [name](std::string_view field) { return std::format("{}_{}", name, field); };
        if (!param<bool>(key("enabled"), false)) {
            RCLCPP_WARN(
                get_logger(), "[%s] board %s disabled by parameter; its devices stay offline",
                get_component_name().c_str(), name);
            return;
        }

        auto options = libhcs::board::AdvancedOptions{};
        options.dangerously_skip_version_checks =
            param<bool>("dangerously_skip_version_checks", false);
        const auto io_cpu = param<std::int64_t>(key("io_thread_cpu"), -1);
        const auto io_priority = param<std::int64_t>(key("io_thread_rt_priority"), 0);
        if (io_cpu >= 0)
            options.set_io_thread_affinity(static_cast<int>(io_cpu), static_cast<int>(io_priority));

        // 波特率走构造期 Configuration：before-session 同步 apply 并读回，重连自动重发
        libhcs::board::hcs::Configuration config;
        config.uart_baudrate[0] = uart0_baudrate;
        const auto serial_filter = param<std::string>(key("serial_filter"), "");
        auto& slot = boards_[board_index(board)];
        slot = std::make_unique<libhcs::board::Hpm5321>(
            callbacks_[board_index(board)], serial_filter, options, config);

        using LinkState = libhcs::host::protocol::Handler::LinkState;
        if (slot->link_state() == LinkState::kFaulted)
            throw std::runtime_error(
                std::format("{}: board {} faulted at construction", get_component_name(), name));

        // 再读回一道：失配超 5% 起不来
        const std::uint32_t effective = slot->uart0_baudrate();
        const auto drift = effective > uart0_baudrate ? effective - uart0_baudrate
                                                      : uart0_baudrate - effective;
        if (std::uint64_t{drift} * 100U > std::uint64_t{uart0_baudrate} * 5U)
            throw std::runtime_error(std::format(
                "{}: board {} uart0 asked {} baud but reads back {}; do not trust this link",
                get_component_name(), name, uart0_baudrate, effective));

        RCLCPP_INFO(
            get_logger(), "[%s] board %s: serial_filter='%s' io_cpu=%lld io_prio=%lld uart0=%u",
            get_component_name().c_str(), name, serial_filter.c_str(),
            static_cast<long long>(io_cpu), static_cast<long long>(io_priority), effective);
    }

    // CAN 路数够用（按接线表推出）且全为经典 CAN 2.0
    void check_classic_can() const {
        std::array<std::uint8_t, kBoards.size()> required{};
        for_each_motor([&](const auto& motor) {
            auto& count = required[board_index(motor.bus.board)];
            count = std::max(count, std::to_underlying(motor.bus.port));
        });
        for (std::size_t index = 0; index < boards_.size(); ++index) {
            if (!boards_[index])
                continue;
            const auto name = kBoards[index].name;
            const auto interface = boards_[index]->interface();
            if (interface.can_count < required[index])
                throw std::runtime_error(std::format(
                    "{}: board {} carries CAN1..CAN{} but the wiring needs {} buses",
                    get_component_name(), name, interface.can_count, required[index]));
            if (interface.can_fd_mask != 0)
                throw std::runtime_error(std::format(
                    "{}: board {} reports CAN FD (mask=0x{:x}) but all motors are classic "
                    "CAN 2.0; reflash the firmware",
                    get_component_name(), name, interface.can_fd_mask));
        }
    }

    // 每路总线负载 = 帧数/拍 × 拍率 × 130 µs（经典 CAN 1 Mbit/s 8 字节帧上界），>70% 告警
    void report_bus_load() {
        std::array<std::array<unsigned, 2>, kBoards.size()> frames{};
        const auto batch = build_safe_batch();
        for (const auto& frame : std::span{batch.frames}.first(batch.frame_count))
            ++frames.at(frame.board).at(frame.port - 1);

        const double update_rate = param<double>("expected_update_rate", 1000.0);
        std::string summary;
        for (std::size_t board = 0; board < frames.size(); ++board) {
            for (std::size_t port = 0; port < frames[board].size(); ++port) {
                if (frames[board][port] == 0)
                    continue;
                const double load = frames[board][port] * update_rate * 130e-6 * 100.0;
                summary += std::format(" {}/can{}={:.0f}%", kBoards[board].name, port + 1, load);
                if (load > 70.0)
                    RCLCPP_WARN(
                        get_logger(), "[%s] bus %s/can%zu at %.0f%% estimated load (>70%%)",
                        get_component_name().c_str(), kBoards[board].name, port + 1, load);
            }
        }
        RCLCPP_INFO(
            get_logger(), "[%s] bus load estimate at %.0f Hz:%s", get_component_name().c_str(),
            update_rate, summary.c_str());
    }

    // ── 尽力域：板卡故障监控 ──────────────────────────────────────────────
    void report() {
        report_safety();
        using LinkState = libhcs::host::protocol::Handler::LinkState;
        for (std::size_t index = 0; index < boards_.size(); ++index) {
            auto& board = boards_[index];
            const auto name = kBoards[index].name;
            if (!board)
                continue;
            if (board->link_state() == LinkState::kFaulted) {
                if (param<bool>("exit_on_board_fault", true)) {
                    RCLCPP_FATAL(
                        get_logger(), "[%s] board %s faulted (link lost); shutting down",
                        get_component_name().c_str(), name);
                    rclcpp::shutdown();
                    return;
                }
                RCLCPP_ERROR(
                    get_logger(), "[%s] board %s faulted (link lost); its devices are offline",
                    get_component_name().c_str(), name);
                continue;
            }
            report_can_status(*board, name);
        }
    }

    // 锁存与 DM 反馈串号的日志：周期域不许打日志，这里 1 Hz 读一次计数。
    // 读的是周期域在写的普通成员，是良性的撕读：最坏把同一次锁存晚一秒报、或把原因报成上一次的。
    void report_safety() {
        using Reason = util::SafetyLatch::Reason;
        if (const auto trips = safety_latch_.trip_count(); trips != reported_trips_) {
            reported_trips_ = trips;
            const auto reason = safety_latch_.reason();
            RCLCPP_ERROR(
                get_logger(),
                "[%s] SAFETY LATCHED: %s %s; all motors disabled. "
                "Fix it, then flip left switch DOWN and back to re-arm",
                get_component_name().c_str(), kCriticalNames.at(safety_latch_.tripped_device()),
                reason == Reason::kFaulted ? "reported a fault" : "went offline");
        }

        for_each_motor([this]<class Driver>(const OnBus<Driver>& motor) {
            if constexpr (std::same_as<Driver, device::DmMotor>) {
                if (const auto count = motor.foreign_frame_count(); count != 0)
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 5000,
                        "[%s] DM %s: %u frames on feedback id 0x%x carried another motor's id; "
                        "two drivers share one MST_ID, fix the register",
                        get_component_name().c_str(), motor.name, count, motor.recv_id());
            }
        });
    }

    // CAN 控制器健康检查（1Hz）：EP0 查询会抛，定时器里兜住
    void report_can_status(libhcs::board::Hpm5321& board, const char* name) {
        namespace hcs = libhcs::board::hcs;
        const auto can_count = board.interface().can_count;
        for (std::uint8_t bus = 0; bus < can_count; ++bus) {
            const auto port = static_cast<hcs::CanPort>(bus + 1);
            hcs::vc::CanStatusPayload status;
            try {
                status = board.can_status(port);
            } catch (const std::exception& error) {
                RCLCPP_WARN(
                    get_logger(), "[%s] board %s can%u status read failed: %s",
                    get_component_name().c_str(), name, static_cast<unsigned>(bus) + 1,
                    error.what());
                return;
            }

            const unsigned flags = status.flags;
            const bool flagged =
                (flags & (hcs::vc::kCanErrorPassive | hcs::vc::kCanWarning | hcs::vc::kCanBusOff))
                != 0;
            if (!flagged && status.tec == 0 && status.rec == 0 && status.rx_fifo_level == 0)
                continue;
            RCLCPP_WARN(
                get_logger(),
                "[%s] board %s can%u: tec=%u rec=%u last=%s flags=0x%02x rx_backlog=%u",
                get_component_name().c_str(), name, static_cast<unsigned>(bus) + 1,
                static_cast<unsigned>(status.tec), static_cast<unsigned>(status.rec),
                hcs::last_error_name(status.last_error), flags, status.rx_fifo_level);
        }
    }

    // ── 参数：只剩部署项，一个模板读全部类型 ──────────────────────────────
    template <class T>
    T param(const std::string& name, std::type_identity_t<T> fallback) const {
        T value = fallback;
        get_parameter_or(name, value, value);
        return value;
    }

    // ── 成员（声明序 = 构造序，析构逆序：发送线程 → 板卡 → 回调 → 设备）──
    std::shared_ptr<BalanceInfantryCommand> command_ =
        create_partner_component<BalanceInfantryCommand>(get_component_name() + "_command", *this);
    std::uint32_t batch_sequence_ = 0; // 指令侧批次序号（周期域）
    util::SafetyLatch safety_latch_;   // 状态侧写、指令侧读，两半同一线程顺序执行
    std::uint32_t reported_trips_ = 0; // 尽力域：已打过日志的锁存次数

    // ── 接线表：一台电机一行（上车前由用户核对）──────────────────────────
    // gimbal CAN1：LK yaw / 拨盘
    OnBus<device::LkMotor> yaw_{
        *this, kGimbalCan1, "/gimbal/yaw",
        device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei10, 0x145}
            .set_encoder_zero_point(9051) // org_pos 0.867753744
            .set_reversed()};             // dir_yaw = -1
    OnBus<device::LkMotor> dial_{
        *this, kGimbalCan1, "/gimbal/dial",
        device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei10, 0x144}};
    // gimbal CAN2：DM pitch（指令 0x04 / 反馈 0x03，寄存器被刷过）+ 3508 摩擦轮
    OnBus<device::DmMotor> pitch_{
        *this, kGimbalCan2, "/gimbal/pitch",
        device::DmMotor::Config{device::DmMotor::Type::kJ4310, 0x04, 0x03}
            .set_position_max(12.566)
            .set_velocity_max(30.0)
            .set_torque_max(40.0)
            .set_zero_angle(0.0285701752) // Helios org_pos
            .set_reversed()};             // dir_pitch = -1
    OnBus<device::DjiMotor> friction_left_{
        *this, kGimbalCan2, "/gimbal/left_friction_wheel",
        device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}};
    OnBus<device::DjiMotor> friction_right_{
        *this, kGimbalCan2, "/gimbal/right_friction_wheel",
        device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}};
    // chassis CAN1/CAN2：左/右腿 DM（Helios bl0/br0 = front，bl1/br1 = back）
    OnBus<device::DmMotor> left_front_joint_{
        *this, kChassisCan1, "/chassis/left_front_joint", leg_joint(0x02, 0x06, 0.717242718)};
    OnBus<device::DmMotor> left_back_joint_{
        *this, kChassisCan1, "/chassis/left_back_joint", leg_joint(0x01, 0x05, 1.938694)};
    OnBus<device::DmMotor> right_front_joint_{
        *this, kChassisCan2, "/chassis/right_front_joint", leg_joint(0x04, 0x08, 1.39220476)};
    OnBus<device::DmMotor> right_back_joint_{
        *this, kChassisCan2, "/chassis/right_back_joint", leg_joint(0x03, 0x07, -1.37379646)};
    // aux CAN1：3508 轮毂，转子到轮 13.94（Helios REDUCTION_RATIO_WHEEL，替换 3508 默认的 19.2）；
    // 右轮速度与电流一起取负 = 驱动层一次反转
    OnBus<device::DjiMotor> left_wheel_{
        *this, kAuxCan1, "/chassis/left_wheel",
        device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}.set_reduction_ratio(13.94)};
    OnBus<device::DjiMotor> right_wheel_{
        *this, kAuxCan1, "/chassis/right_wheel",
        device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}
            .set_reduction_ratio(13.94)
            .set_reversed()};

    // IMU 模块 flash 坐标系：云台 NWU（直通 FLU），底盘 ENU（驱动把 RFU 转 FLU）
    device::Hipnuc gimbal_imu_{*this, "/gimbal/imu", device::Hipnuc::Config{}.set_module_frame(ImuFrame::kNwu)};
    device::Hipnuc chassis_imu_{*this, "/chassis/imu", device::Hipnuc::Config{}.set_module_frame(ImuFrame::kEnu)};
    ImuScalarOutputs<kGimbalImuScalars> gimbal_imu_scalars_{*this, gimbal_imu_};
    ImuScalarOutputs<kChassisImuScalars> chassis_imu_scalars_{*this, chassis_imu_};
    device::Vt13 vt13_;
    device::RemoteControl remote_control_{*this};

    // 板卡构造期就可能进回调，引用的都是上面的设备
    std::array<BoardCallback, kBoards.size()> callbacks_{{
        {*this, Board::kGimbal},
        {*this, Board::kChassis},
        {*this, Board::kAux},
    }};
    std::array<std::unique_ptr<libhcs::board::Hpm5321>, kBoards.size()> boards_;
    std::unique_ptr<util::BoardTransmitter> transmitter_;

    OutputInterface<hcs_description::Tf> tf_;

    rclcpp::TimerBase::SharedPtr report_timer_;
};

} // namespace hcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::hardware::BalanceInfantry, hcs_executor::Component)
