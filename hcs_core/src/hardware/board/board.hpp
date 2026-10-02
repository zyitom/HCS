#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hcs_can_port.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/data/datas.hpp>
#include <libhcs/protocol/handler.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/realtime_scope.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_executor/component.hpp>

#include "hardware/board/device.hpp"
#include "hardware/util/board_transmitter.hpp"

namespace hcs_core::hardware::board {

// ============================================================================
// 一块板 = 一个执行器组件：一块 libhcs 板、板上的端口、端口上的设备。
//
//   struct Gimbal : Board<libhcs::board::Hpm5321> {
//       CanBus can1{*this, Spec::kCans.kCan1, kClassic1M};
//       SerialPort uart0{*this, Spec::kUarts.kUart0};
//       DmMotor pitch{can1, "/gimbal/pitch", {...}};
//       Hipnuc imu{uart0, "/gimbal/imu", {.baudrate = 921600, ...}};
//   };
//   using GimbalBoard = BoardComponent<Gimbal>;   // 导出为插件
//
// 板组件自己收、自己发：
//   事件域  libhcs 回调 → 端口 → 设备（存原始帧）
//   周期域  update()：本板设备解码写输出；Command 伙伴：本板设备打帧 → 本板的发送 Lane
//   发送    几块板共用的发送线程（TransmitThread 组件，经输入 transmitter 拿到）
// 跨板的事（安全锁存、TF）是图里别的组件，板组件只读 /hardware/safe 这一个量。
//
// 显式配置：CAN 总线的帧型与速率写在 CanBus 上，设备登记时核对自己支持；串口的速率与
// 帧格式由设备给出（serial_line()）。用到的口一律经 EP0 显式下发，没有"沿用固件当前"。
//
// 参数（本组件名下）：
//   enabled                 默认 false：不打开板卡，设备保持离线（台架缺板照样能跑图）
//   serial_filter           USB 序列号子串，同 PID 多块板时必须填
//   io_thread_cpu / io_thread_rt_priority   libhcs 事件线程
//   dangerously_skip_version_checks
//   exit_on_board_fault     链路永久失效时整体退出（默认 true）
//   expected_update_rate    总线负载估算用（默认 1000）
//   transmitter             发送线程的输入名（默认 /hardware/transmitter）
//   safety                  安全量的输入名（默认 /hardware/safe；空 = 不接锁存）
// ============================================================================

class BoardCore;
template <class Sdk>
class Board;

/// 一路 CAN：声明帧型与速率；按 CAN id 查表分发反馈；登记时检查撞号与速率。
class CanBus {
public:
    /// @param can 本板型的 CAN 描述符（Spec::kCans.kCanN）：别的板型的描述符编译不过。
    /// @param setting 这路总线的帧型与速率（如 kClassic1M），构造期经 EP0 下发。
    template <class Sdk>
    CanBus(
        Board<Sdk>& board, const typename Board<Sdk>::Spec::Can& can,
        const libhcs::board::hcs::CanSetting& setting)
        : CanBus(
              board, Board<Sdk>::Spec::kCans.index_of(can),
              libhcs::board::hcs::can_port(can.data_id), setting) {}

    CanBus(const CanBus&) = delete;
    CanBus& operator=(const CanBus&) = delete;

    [[nodiscard]] BoardCore& board() const noexcept { return board_; }
    /// 在板型端口表里的下标，即 EP0 配置里的总线号。
    [[nodiscard]] std::size_t index() const noexcept { return index_; }
    [[nodiscard]] libhcs::board::hcs::CanPort port() const noexcept { return port_; }
    [[nodiscard]] const libhcs::board::hcs::CanSetting& setting() const noexcept {
        return setting_;
    }
    /// "gimbal_board/can2"
    [[nodiscard]] std::string label() const;
    [[nodiscard]] std::span<CanDevice* const> devices() const noexcept { return devices_; }

    /// 设备构造时调用。总线速率/帧型与设备不符、反馈 id 撞号、指令 id 与别人的反馈 id 相同、
    /// 独占指令 id 重复、共享帧槽位重复，都在这里抛 std::invalid_argument。
    void attach(CanDevice& device);

    /// libhcs IO 线程（事件域）：一次查表，一次虚调用。
    void receive(std::uint32_t can_id, std::span<const std::byte> data) noexcept HCS_NONBLOCKING {
        if (can_id < table_.size()) [[likely]]
            if (auto* device = table_[can_id])
                device->receive(can_id, data);
    }

private:
    CanBus(
        BoardCore& board, std::size_t index, libhcs::board::hcs::CanPort port,
        const libhcs::board::hcs::CanSetting& setting);

    BoardCore& board_;
    std::size_t index_;
    libhcs::board::hcs::CanPort port_;
    libhcs::board::hcs::CanSetting setting_;
    std::vector<CanDevice*> devices_;
    std::array<CanDevice*, 0x800> table_{}; ///< 标准帧 11 位 id → 设备
};

/// 一路串口。一个口只接一个设备（串口是字节流，没有 id 可分），设置由这个设备给出。
class SerialPort {
public:
    /// @param uart 本板型的串口描述符（Spec::kUarts.kUart0、Spec::kUarts.kDbus……）
    template <class Sdk>
    SerialPort(Board<Sdk>& board, const typename Board<Sdk>::Spec::Uart& uart)
        : SerialPort(board, Board<Sdk>::Spec::kUarts.index_of(uart), uart.data_id) {}

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    [[nodiscard]] BoardCore& board() const noexcept { return board_; }
    [[nodiscard]] std::size_t index() const noexcept { return index_; }
    /// "aux_board/dbus"
    [[nodiscard]] std::string label() const;
    [[nodiscard]] SerialDevice* device() const noexcept { return device_; }
    /// 经 EP0 下发的设置：由挂在上面的设备决定；没挂设备时为空（不配置）。
    [[nodiscard]] std::optional<libhcs::board::hcs::UartSetting> setting() const;

    void attach(SerialDevice& device);

    /// libhcs IO 线程（事件域）。
    void receive(std::span<const std::byte> data) noexcept HCS_NONBLOCKING {
        if (device_)
            device_->receive(data);
    }

private:
    SerialPort(BoardCore& board, std::size_t index, libhcs::data::DataId data_id);

    BoardCore& board_;
    std::size_t index_;
    libhcs::data::DataId data_id_;
    SerialDevice* device_ = nullptr;
};

/// 板载 IMU：板子自己焊着的那一颗（BMI088），加速度计和陀螺仪的样本经 USB 直接上来。
/// 一块板只有一个，也只接一个设备。它没有可配置的线路参数，所以不出现在 EP0 配置里。
class ImuPort {
public:
    template <class Sdk>
    explicit ImuPort(Board<Sdk>& board)
        : ImuPort(static_cast<BoardCore&>(board)) {}

    ImuPort(const ImuPort&) = delete;
    ImuPort& operator=(const ImuPort&) = delete;

    [[nodiscard]] BoardCore& board() const noexcept { return board_; }
    /// "gimbal_board/imu"
    [[nodiscard]] std::string label() const;
    [[nodiscard]] ImuDevice* device() const noexcept { return device_; }

    void attach(ImuDevice& device);

    /// libhcs IO 线程（事件域）。
    void receive(const device::ImuSample& sample) noexcept HCS_NONBLOCKING {
        if (device_)
            device_->receive(sample);
    }

private:
    explicit ImuPort(BoardCore& board);

    BoardCore& board_;
    ImuDevice* device_ = nullptr;
};

/// 板组件与型号无关的部分：生命周期、参数、周期域的收发、尽力域的上报。
///
/// 生命周期：构造（端口、设备登记）→ before_updating()（读参数、打开板卡、登记到发送线程）
/// → shutdown()（离开发送线程、停 IO 线程）→ 析构。shutdown() 必须在派生类成员（设备）析构
/// 之前，否则 IO 线程可能回调进已经析构的设备，所以由 BoardComponent<> 的析构函数调用。
class BoardCore
    : public hcs_executor::Component
    , public rclcpp::Node
    , public util::BatchSink {
public:
    ~BoardCore() override;

    void before_updating() override;
    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override;

    /// 指令侧伙伴：设备的 /control_* 输入注册在它身上，它排在所有控制器之后。
    [[nodiscard]] hcs_executor::Component& command() noexcept;

    [[nodiscard]] std::span<CanBus* const> can_buses() const noexcept { return can_buses_; }
    [[nodiscard]] std::span<SerialPort* const> serial_ports() const noexcept {
        return serial_ports_;
    }
    /// 所有端口上的设备，按声明顺序。
    [[nodiscard]] std::vector<Device*> devices() const;

    /// 板卡已打开（enabled 且构造成功）。
    [[nodiscard]] virtual bool opened() const noexcept HCS_NONBLOCKING = 0;

    /// 离开发送线程、关闭板卡（停 IO 线程）。之后设备才可以析构。可重复调用。
    void shutdown() noexcept;

protected:
    BoardCore();

    /// 型号相关：打开 libhcs 板卡；关闭；链路状态；板上 CAN 路数；CAN 控制器状态上报。
    virtual void open_sdk(
        const std::string& serial_filter, const libhcs::board::AdvancedOptions& options,
        const libhcs::board::hcs::Configuration& configuration) = 0;
    virtual void close_sdk() noexcept = 0;
    [[nodiscard]] virtual bool link_faulted() const noexcept = 0;
    [[nodiscard]] virtual std::size_t can_count() const = 0;
    virtual void report_can_health() = 0;

    /// 1 Hz 上报用：CAN 控制器状态有异常就打一行；EP0 查询失败打一行。
    void log_can_status(
        libhcs::board::hcs::CanPort port, const libhcs::board::hcs::vc::CanStatusPayload& status);
    void log_can_status_failure(libhcs::board::hcs::CanPort port, const std::exception& error);

    [[nodiscard]] CanBus* can_bus(std::size_t index) const noexcept HCS_NONBLOCKING {
        return index < can_by_index_.size() ? can_by_index_[index] : nullptr;
    }
    [[nodiscard]] SerialPort* serial_port(std::size_t index) const noexcept HCS_NONBLOCKING {
        return index < serial_by_index_.size() ? serial_by_index_[index] : nullptr;
    }
    [[nodiscard]] ImuPort* imu_port() const noexcept HCS_NONBLOCKING { return imu_port_; }

private:
    friend class CanBus;
    friend class SerialPort;
    friend class ImuPort;
    class Command;

    void add(CanBus& bus);
    void add(SerialPort& port);
    void add(ImuPort& port);
    void leave_transmitter() noexcept;

    [[nodiscard]] libhcs::board::hcs::Configuration configuration() const;
    void open();
    void command_update(const hcs_sync::Tick& tick) HCS_NONBLOCKING;
    void pack_frames(util::TransmitBatch& batch, bool safe) HCS_NONBLOCKING;
    void report_bus_load();
    void report();

    template <class T>
    T param(const std::string& name, T fallback) const {
        get_parameter_or(name, fallback, fallback);
        return fallback;
    }

    std::shared_ptr<Command> command_;
    InputInterface<std::shared_ptr<util::SharedTransmitter>> transmitter_;

    std::vector<CanBus*> can_buses_;
    std::vector<SerialPort*> serial_ports_;
    std::array<CanBus*, 8> can_by_index_{};        ///< 下标 = 板型端口表下标
    std::array<SerialPort*, 8> serial_by_index_{}; ///< 同上（EP0 配置最多 8 路）
    ImuPort* imu_port_ = nullptr;                  ///< 板载 IMU，一块板至多一个
    std::vector<Device*> devices_;                 ///< before_updating() 时收齐

    std::shared_ptr<util::SharedTransmitter> transmitter_ref_; ///< 打开之后持有，关闭时还要用
    util::SharedTransmitter::Lane* lane_ = nullptr;             ///< 打开之后才有
    std::uint32_t batch_sequence_ = 0;              ///< 指令侧批次序号（周期域）
    rclcpp::TimerBase::SharedPtr report_timer_;
    /// "链路断了"在不退出的配置下每 5 秒提醒一次，只在 report() 里用（spin 线程）。
    hcs_log::Throttle link_fault_report_{std::chrono::seconds{5}};
};

/// 一块 libhcs 板。Sdk 是 libhcs 的板卡类（libhcs::board::Hpm5321、Mc02……）：
/// 构造 (callback, serial_filter, options, configuration)，Callback 以描述符回调
/// （can_receive_callback(Spec::Can, ...) / uart_receive_callback(Spec::Uart, ...)），
/// 板载 IMU 走 accelerometer_receive_callback / gyroscope_receive_callback，
/// 另有 link_state()、interface()、can_status()、start_transmit().can_transmit()。
template <class Sdk>
class Board : public BoardCore {
public:
    /// 本板型的端口描述符：Spec::kCans.kCan1、Spec::kUarts.kUart0……
    using Spec = typename Sdk::Callback::Spec;

    [[nodiscard]] bool opened() const noexcept HCS_NONBLOCKING override { return sdk_ != nullptr; }

    /// 打开之后的 libhcs 板卡对象（未打开时为空）。
    [[nodiscard]] Sdk* sdk() noexcept { return sdk_.get(); }

    /// 发送线程：整批帧塞进同一个发送缓冲。
    void send(const util::TransmitBatch& batch) override {
        if (!sdk_)
            return;
        auto builder = sdk_->start_transmit();
        for (const auto& frame : std::span{batch.frames}.first(batch.frame_count))
            builder.can_transmit(
                static_cast<libhcs::board::hcs::CanPort>(frame.port),
                libhcs::data::CanDataView{.can_id = frame.can_id, .can_data = frame.data});
    }

protected:
    Board() = default;
    ~Board() override { close_sdk(); } // 兜底；正常路径已由 BoardComponent<> 先关

    void open_sdk(
        const std::string& serial_filter, const libhcs::board::AdvancedOptions& options,
        const libhcs::board::hcs::Configuration& configuration) override {
        sdk_ = std::make_unique<Sdk>(callback_, serial_filter, options, configuration);
    }

    void close_sdk() noexcept override { sdk_.reset(); }

    [[nodiscard]] bool link_faulted() const noexcept override {
        using LinkState = libhcs::host::protocol::Handler::LinkState;
        return sdk_ && sdk_->link_state() == LinkState::kFaulted;
    }

    [[nodiscard]] std::size_t can_count() const override {
        return sdk_ ? sdk_->interface().can_count : 0;
    }

    /// CAN 控制器健康检查（1 Hz）：EP0 查询会抛，这里兜住。
    void report_can_health() override {
        if (!sdk_)
            return;
        for (const auto& can : Spec::kCans) {
            if (Spec::kCans.index_of(can) >= can_count())
                continue;
            const auto port = libhcs::board::hcs::can_port(can.data_id);
            try {
                log_can_status(port, sdk_->can_status(port));
            } catch (const std::exception& error) {
                log_can_status_failure(port, error);
                return;
            }
        }
    }

private:
    /// libhcs 回调：按端口描述符转给对应的 CanBus / SerialPort。
    /// IO 线程（事件域）：只把原始帧存进设备——不分配、不加锁、不打日志。RealtimeScope
    /// 让 RTSan 运行期检查。外层 libhcs 的 REAPURB / 重新提交接收缓冲本来就要进内核，不在此列。
    class Callback final : public Sdk::Callback {
    public:
        using View = typename Sdk::Callback::View;

        explicit Callback(Board& board)
            : board_(board) {}

        void can_receive_callback(
            const typename Spec::Can& can, const typename View::Can& data) override {
            [[maybe_unused]] const hcs_utility::RealtimeScope realtime_scope;
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (auto* bus = board_.can_bus(Spec::kCans.index_of(can)))
                bus->receive(data.can_id, data.can_data);
        }

        void uart_receive_callback(
            const typename Spec::Uart& uart, const typename View::Uart& data) override {
            [[maybe_unused]] const hcs_utility::RealtimeScope realtime_scope;
            if (auto* port = board_.serial_port(Spec::kUarts.index_of(uart)))
                port->receive(data.uart_data);
        }

        void accelerometer_receive_callback(
            const libhcs::data::ImuAccelerometerDataView& data) override {
            [[maybe_unused]] const hcs_utility::RealtimeScope realtime_scope;
            if (auto* port = board_.imu_port())
                port->receive(
                    {.kind = device::ImuSample::Kind::kAccelerometer,
                     .x = data.x,
                     .y = data.y,
                     .z = data.z,
                     .timestamp_quarter_us = data.timestamp_quarter_us});
        }

        void gyroscope_receive_callback(const libhcs::data::ImuGyroscopeDataView& data) override {
            [[maybe_unused]] const hcs_utility::RealtimeScope realtime_scope;
            if (auto* port = board_.imu_port())
                port->receive(
                    {.kind = device::ImuSample::Kind::kGyroscope,
                     .x = data.x,
                     .y = data.y,
                     .z = data.z,
                     .timestamp_quarter_us = data.timestamp_quarter_us});
        }

    private:
        Board& board_;
    };

    Callback callback_{*this};
    std::unique_ptr<Sdk> sdk_; ///< 最后声明：先于 callback_ 析构
};

/// 导出为插件的板组件：在设备（派生类成员）析构之前先 shutdown()。
///
///   struct Gimbal : Board<libhcs::board::Hpm5321> { ... };
///   using GimbalBoard = BoardComponent<Gimbal>;
///   PLUGINLIB_EXPORT_CLASS(hcs_core::hardware::GimbalBoard, hcs_executor::Component)
template <class Wiring>
class BoardComponent final : public Wiring {
public:
    ~BoardComponent() override { this->shutdown(); }
};

using libhcs::board::hcs::kClassic1M;

} // namespace hcs_core::hardware::board
