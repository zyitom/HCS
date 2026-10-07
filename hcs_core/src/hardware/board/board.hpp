#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/data/datas.hpp>
#include <libhcs/protocol/handler.hpp>
#include <libhcs/spec/port.hpp>
#include <libhcs/time/sample_time.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <hcs_base/channel/snapshot.hpp>
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
// 板组件自己收、自己发，CAN 和串口两个方向都走同一条路：
//   事件域  libhcs 回调 → 端口 → 设备（存原始帧 / 字节）
//   周期域  update()：本板设备解码写输出；
//           Command 伙伴：本板设备打帧 → 本板的发送 Lane，串口设备的字节 → 各自的发件箱
//   发送域  几块板共用的发送线程（TransmitThread 组件，经输入 transmitter 拿到）：
//           一块板一拍一个发送缓冲，CAN 帧和串口字节都在里面
// 跨板的事（安全锁存、TF）是图里别的组件，板组件只读 /hardware/safe 这一个量。
//
// 显式配置：CAN 总线的帧型与速率写在 CanBus 上，设备登记时核对自己支持；串口的速率与
// 帧格式由设备给出（serial_line()）。用到的口一律经 EP0 显式下发，没有"沿用固件当前"。
// 这次下发同时是声明：接线表没写的口不出现在 EP0 配置里，板子就不启动它（libhcs 的
// "配置即声明"）——没声明的 CAN 不上总线，没声明的串口不收不发。往没声明的口发，libhcs 的
// 板卡类会抛 UndeclaredChannel；这一层发出去的帧和字节只来自声明过的 CanBus / SerialPort，
// 所以那个异常在这里是写不出来的情况，不需要、也没有对应的处理。
//
// 参数（本组件名下）：
//   enabled                 默认 false：不打开板卡，设备保持离线（台架缺板照样能跑图）
//   serial_filter           USB 序列号子串，同 PID 多块板时必须填
//   io_thread_cpu / io_thread_rt_priority   libhcs 事件线程
//   dangerously_skip_version_checks
//   time_sync               开共享时间基准（默认 false）：板卡给每帧 CAN 反馈打上硬件时间戳，
//                           SDK 在 IO 线程把它换算成 libhcs::time::SampleTime（样本时刻 + 来源
//                           + 板上时刻），CAN 设备与板载 IMU 收到的每一路反馈都带着它，经
//                           <设备名>/time 输出给控制环直接读、不算时间。不开时 CAN 的时刻退回
//                           到达时刻（看 source）。串口线上没有戳，这一层不取它的时刻。
//                           它是声明的一部分：板子接受了清单才开，没开的板子连 SOF 中断都不收；
//                           板子报告没有时间基准（c_board）时构造即抛。
//   exit_on_board_fault     链路永久失效时整体退出（默认 true）
//   expected_update_rate    总线负载估算用（默认 1000）
//   transmitter             发送线程的输入名（默认 /hardware/transmitter）
//   safety                  安全量的输入名（默认 /hardware/safe；空 = 不接锁存）
// ============================================================================

} // namespace hcs_core::hardware::board

namespace hcs_core::hardware::board {

/// 板层对 libhcs 板卡类的全部要求：Board<Sdk> 用它约束。真板卡类（libhcs::board::Hpm5321、
/// Mc02、CBoard、Hpm6e8y）与测试里的假 SDK 都满足它；写新后端（SocketCAN 之类）时对着它实现，
/// 接错时编译错误停在这里，而不是 Board 模板深处。只检查形状，零运行时开销。
///
///   - Spec：端口描述符。Spec::Can / Spec::Uart 由 DataId 构造、带回 data_id；Spec::kPorts 是
///     本板型全部口的清单（kind + data_id），发送时按它遍历串口。
///   - Callback：IO 线程回调的基类（虚析构），Board 派生它并覆盖下面这几个回调。
///   - 构造 (callback, serial_filter, options, configuration)：打开板子、经 EP0 下发声明。
///   - link_state() / port_status(port)：链路状态与板子推送的口状态，尽力域读。
///   - start_transmit()：一个发送缓冲，can_transmit / uart_transmit 往里塞，析构时发出。
template <class Sdk>
concept BoardSdk =
    requires {
        typename Sdk::Spec;
        typename Sdk::Spec::Can;
        typename Sdk::Spec::Uart;
        typename Sdk::Callback;
    }
    && std::constructible_from<typename Sdk::Spec::Can, libhcs::data::DataId>
    && std::constructible_from<typename Sdk::Spec::Uart, libhcs::data::DataId>
    && requires(const typename Sdk::Spec::Can& can, const typename Sdk::Spec::Uart& uart) {
           { can.data_id } -> std::convertible_to<libhcs::data::DataId>;
           { uart.data_id } -> std::convertible_to<libhcs::data::DataId>;
           { std::begin(Sdk::Spec::kPorts)->kind } -> std::convertible_to<libhcs::spec::PortKind>;
           { std::begin(Sdk::Spec::kPorts)->data_id } -> std::convertible_to<libhcs::data::DataId>;
       }
    && std::has_virtual_destructor_v<typename Sdk::Callback>
    && requires(
        typename Sdk::Callback& callback, typename Sdk::Spec::Can can,
        typename Sdk::Spec::Uart uart, const libhcs::data::CanDataView& can_data,
        const libhcs::data::UartDataView& uart_data,
        const libhcs::data::ImuAccelerometerDataView& accelerometer,
        const libhcs::data::ImuGyroscopeDataView& gyroscope,
        const libhcs::time::SampleTime& time) {
           callback.can_receive_callback(can, can_data, time);
           callback.uart_receive_callback(uart, uart_data, time);
           callback.accelerometer_receive_callback(accelerometer, time);
           callback.gyroscope_receive_callback(gyroscope, time);
       }
    && std::constructible_from<
        Sdk, typename Sdk::Callback&, const std::string&, const libhcs::board::AdvancedOptions&,
        const libhcs::board::hcs::Configuration&>
    && requires(const Sdk& sdk, libhcs::data::DataId port) {
           {
               sdk.link_state()
           } -> std::convertible_to<libhcs::host::protocol::Handler::LinkState>;
           { sdk.port_status(port) } -> std::convertible_to<libhcs::data::PortStatusVariant>;
       }
    && requires(
        Sdk& sdk, typename Sdk::Spec::Can can, typename Sdk::Spec::Uart uart,
        const libhcs::data::CanDataView& can_data, const libhcs::data::UartDataView& uart_data) {
           sdk.start_transmit().can_transmit(can, can_data);
           sdk.start_transmit().uart_transmit(uart, uart_data);
       };

class BoardCore;
template <BoardSdk Sdk>
class Board;

/// 一路 CAN：声明帧型与速率；按 CAN id 查表分发反馈；登记时检查撞号与速率。
/// 端口身份 = DataId（丝印号），从描述符一路带到发送批次，没有任何位置换算。
class CanBus {
public:
    /// @param can 本板型的 CAN 描述符（Spec::kCans.kCanN）：别的板型的描述符编译不过。
    /// @param setting 这路总线的帧型与速率（如 kClassic1M），构造期经 EP0 下发。
    template <class Sdk>
    CanBus(
        Board<Sdk>& board, const typename Board<Sdk>::Spec::Can& can,
        const libhcs::board::hcs::CanSetting& setting)
        : CanBus(board, can.data_id, setting) {}

    CanBus(const CanBus&) = delete;
    CanBus& operator=(const CanBus&) = delete;

    [[nodiscard]] BoardCore& board() const noexcept { return board_; }
    /// 这路总线的身份（丝印号），也是 EP0 清单里的寻址。
    [[nodiscard]] libhcs::data::DataId data_id() const noexcept { return data_id_; }
    [[nodiscard]] const libhcs::board::hcs::CanSetting& setting() const noexcept {
        return setting_;
    }
    /// "gimbal_board/can2"
    [[nodiscard]] std::string label() const;
    [[nodiscard]] std::span<CanDevice* const> devices() const noexcept { return devices_; }

    /// 设备构造时调用。总线速率/帧型与设备不符、反馈 id 撞号、指令 id 与别人的反馈 id 相同、
    /// 独占指令 id 重复、共享帧槽位重复，都在这里抛 std::invalid_argument。
    void attach(CanDevice& device);

    /// 尽力域（1 Hz 健康检查）：换上板子推送的最新状态，返回上次读到的；第一次读返回空，
    /// 只记基准（计数从板子上电起算，第一次的值不是"这一秒"的）。状态跟着总线走，不另开
    /// 一张按口编号索引的表。
    [[nodiscard]] std::optional<libhcs::data::CanStatusView>
        exchange_status(const libhcs::data::CanStatusView& now) noexcept {
        return std::exchange(status_seen_, now);
    }

    /// libhcs IO 线程（事件域）：一次查表，一次虚调用。
    void receive(std::uint32_t can_id, std::span<const std::byte> data,
        libhcs::time::SampleTime time) noexcept HCS_NONBLOCKING {
        if (can_id < table_.size()) [[likely]]
            if (auto* device = table_[can_id])
                device->receive(can_id, data, time);
    }

private:
    CanBus(
        BoardCore& board, libhcs::data::DataId data_id,
        const libhcs::board::hcs::CanSetting& setting);

    BoardCore& board_;
    libhcs::data::DataId data_id_;
    libhcs::board::hcs::CanSetting setting_;
    std::vector<CanDevice*> devices_;
    std::array<CanDevice*, 0x800> table_{}; ///< 标准帧 11 位 id → 设备
    std::optional<libhcs::data::CanStatusView> status_seen_; ///< 尽力域独占，见 exchange_status()
};

/// 一路串口。一个口只接一个设备（串口是字节流，没有 id 可分），设置由这个设备给出。
class SerialPort {
public:
    /// @param uart 本板型的串口描述符（Spec::kUarts.kUart0、Spec::kUarts.kDbus……）
    template <class Sdk>
    SerialPort(Board<Sdk>& board, const typename Board<Sdk>::Spec::Uart& uart)
        : SerialPort(board, uart.data_id) {}

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    [[nodiscard]] BoardCore& board() const noexcept { return board_; }
    /// 这一串口的身份（丝印号），也是 EP0 清单里的寻址。
    [[nodiscard]] libhcs::data::DataId data_id() const noexcept { return data_id_; }
    /// "mc02_board/dbus"
    [[nodiscard]] std::string label() const;
    [[nodiscard]] SerialDevice* device() const noexcept { return device_; }
    /// 经 EP0 下发的设置：由挂在上面的设备决定；没挂设备时为空（不配置）。
    [[nodiscard]] std::optional<libhcs::board::hcs::UartSetting> setting() const;
    /// 这一口往外发不发字节：由挂在上面的设备决定（attach 之后不再变）。
    [[nodiscard]] bool transmits() const noexcept { return transmits_; }

    void attach(SerialDevice& device);

    /// 尽力域（1 Hz 健康检查）：同 CanBus::exchange_status()。
    [[nodiscard]] std::optional<libhcs::data::UartStatusView>
        exchange_status(const libhcs::data::UartStatusView& now) noexcept {
        return std::exchange(status_seen_, now);
    }

    /// libhcs IO 线程（事件域）。
    void receive(std::span<const std::byte> data) noexcept HCS_NONBLOCKING {
        if (device_)
            device_->receive(data);
    }

    /// 发送线程（发送域）：取走这一口待发的字节，最多 out.size() 个。
    std::size_t take_transmit(std::span<std::byte> out) noexcept {
        return transmits_ ? device_->take_transmit(out) : 0;
    }

private:
    SerialPort(BoardCore& board, libhcs::data::DataId data_id);

    BoardCore& board_;
    libhcs::data::DataId data_id_;
    SerialDevice* device_ = nullptr;
    bool transmits_ = false;
    std::optional<libhcs::data::UartStatusView> status_seen_; ///< 尽力域独占，见 exchange_status()
};

/// 板载 IMU：板子自己焊着的那一颗（BMI088），加速度计和陀螺仪的样本经 USB 直接上来。
/// 一块板只有一个，也只接一个设备。和串口一样，它的设置（量程、速率）由挂在上面的设备给出、
/// 经 EP0 下发；这次下发同时是声明——接线表没写 ImuPort 的板，传感器根本不初始化。
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
    /// 经 EP0 下发的设置：由挂在上面的设备决定；没挂设备时为空（不声明）。
    [[nodiscard]] std::optional<libhcs::board::hcs::ImuSetting> setting() const;

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
/// 之前，否则 IO 线程和发送线程可能进到已经析构的设备里，所以由 BoardComponent<> 的析构函数
/// 调用。这一条由类型管着：接线结构体是抽象类（见 sealed_by_board_component()），只有套上
/// BoardComponent<> 才构造得出来、才导出得成插件。
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

    /// 型号相关：打开 libhcs 板卡；关闭；链路状态；一路 CAN / 串口的运行时状态（板子随
    /// keepalive 推送的快照，读它不发 USB 请求；板卡已打开时才调）。
    virtual void open_sdk(
        const std::string& serial_filter, const libhcs::board::AdvancedOptions& options,
        const libhcs::board::hcs::Configuration& configuration) = 0;
    virtual void close_sdk() noexcept = 0;
    [[nodiscard]] virtual bool link_faulted() const noexcept = 0;
    [[nodiscard]] virtual libhcs::data::PortStatusVariant
        port_status(libhcs::data::DataId port) const noexcept = 0;

    /// 同 port_status()，取成一种（这个口没报过这一种时为全零）。
    template <class View>
    [[nodiscard]] View status(libhcs::data::DataId port) const noexcept {
        const auto any = port_status(port);
        const auto* view = std::get_if<View>(&any);
        return view != nullptr ? *view : View{};
    }

    [[nodiscard]] CanBus* can_bus(libhcs::data::DataId port) const noexcept HCS_NONBLOCKING {
        const auto slot = std::to_underlying(port);
        return slot < can_by_id_.size() ? can_by_id_[slot] : nullptr;
    }
    [[nodiscard]] SerialPort* serial_port(libhcs::data::DataId port) const noexcept HCS_NONBLOCKING {
        const auto slot = std::to_underlying(port);
        return slot < serial_by_id_.size() ? serial_by_id_[slot] : nullptr;
    }
    [[nodiscard]] ImuPort* imu_port() const noexcept HCS_NONBLOCKING { return imu_port_; }

private:
    friend class CanBus;
    friend class SerialPort;
    friend class ImuPort;
    class Command;

    /// 只有 BoardComponent<> 实现它。接线结构体（struct X : Board<Sdk>）因此是抽象类：
    /// 不经 BoardComponent<X> 就导出或构造，编译不过——而不是在退出时让 IO 线程
    /// 回调进已经析构的设备。
    virtual void sealed_by_board_component() const noexcept = 0;

private:
    void add(CanBus& bus);
    void add(SerialPort& port);
    void add(ImuPort& port);
    void leave_transmitter() noexcept;

    [[nodiscard]] libhcs::board::hcs::Configuration configuration() const;
    void open();
    void command_update(const hcs_sync::Tick& tick) HCS_NONBLOCKING;
    void pack_frames(util::TransmitBatch& batch, bool safe) HCS_NONBLOCKING;
    void report_bus_load();
    void report_port_health();
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
    /// 端口按 DataId（丝印号）索引：DataId 的端口值都在 32 以内，一个口一个槽。
    static constexpr std::size_t kPortSlots = 32;
    std::array<CanBus*, kPortSlots> can_by_id_{};
    std::array<SerialPort*, kPortSlots> serial_by_id_{};
    ImuPort* imu_port_ = nullptr;                  ///< 板载 IMU，一块板至多一个
    std::vector<Device*> devices_;                 ///< before_updating() 时收齐

    std::shared_ptr<util::SharedTransmitter> transmitter_ref_; ///< 打开之后持有，关闭时还要用
    util::SharedTransmitter::Lane* lane_ = nullptr;             ///< 打开之后才有
    std::uint32_t batch_sequence_ = 0;              ///< 指令侧批次序号（周期域）
    rclcpp::TimerBase::SharedPtr report_timer_;
    /// "链路断了"在不退出的配置下每 5 秒提醒一次，只在 report() 里用（spin 线程）。
    hcs_log::Throttle link_fault_report_{std::chrono::seconds{5}};
    /// 链路本身（DataId::kSession）的状态，上次 report() 读到的（尽力域独占）；口的那些
    /// 跟着各自的 CanBus / SerialPort 走，链路不属于哪个口，记在板上。
    std::optional<libhcs::data::LinkStatusView> link_status_seen_;
};

/// 一块 libhcs 板。Sdk 是 libhcs 的板卡类（libhcs::board::Hpm5321、Mc02……），
/// 要求见 BoardSdk。
template <BoardSdk Sdk>
class Board : public BoardCore {
public:
    /// 本板型的端口描述符：Spec::kCans.kCan1、Spec::kUarts.kUart0……
    using Spec = typename Sdk::Spec;

    [[nodiscard]] bool opened() const noexcept HCS_NONBLOCKING override { return sdk_ != nullptr; }

    /// 打开之后的 libhcs 板卡对象（未打开时为空）。
    [[nodiscard]] Sdk* sdk() noexcept { return sdk_.get(); }

    /// 发送线程：整批帧塞进同一个发送缓冲，各串口发件箱里攒下的字节跟在后面。
    /// 帧的 port 字段与串口的查表键都是 DataId（丝印号）——清单里有什么口就发什么口，
    /// 编译期展开，没有下标换算。
    void send(const util::TransmitBatch& batch) override {
        if (!sdk_)
            return;
        auto builder = sdk_->start_transmit();
        for (const auto& frame : std::span{batch.frames}.first(batch.frame_count))
            builder.can_transmit(
                typename Spec::Can{frame.port},
                libhcs::data::CanDataView{.can_id = frame.can_id, .can_data = frame.data});

        for (const auto& descriptor : Spec::kPorts) {
            if (descriptor.kind != libhcs::spec::PortKind::kUart)
                continue;
            auto* port = serial_port(descriptor.data_id);
            if (port == nullptr || !port->transmits())
                continue;
            // 一次最多取一整个发件箱的量：驱动写得再快，这一次发送的工作量也有上界。
            std::array<std::byte, kSerialChunkBytes> chunk;
            for (std::size_t budget = util::SerialOutbox::kCapacity; budget != 0;) {
                const auto count =
                    port->take_transmit(std::span{chunk}.first(std::min(chunk.size(), budget)));
                if (count == 0)
                    break;
                builder.uart_transmit(
                    typename Spec::Uart{descriptor.data_id},
                    {.uart_data = std::span<const std::byte>{chunk}.first(count)});
                budget -= count;
            }
        }
    }

protected:
    Board() = default;
    ~Board() override = default; // 板卡已由 BoardComponent<> 的析构函数经 shutdown() 关掉

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

    [[nodiscard]] libhcs::data::PortStatusVariant
        port_status(libhcs::data::DataId port) const noexcept override {
        return sdk_->port_status(port);
    }

private:
    /// libhcs 回调：按端口描述符转给对应的 CanBus / SerialPort。
    /// IO 线程（事件域）：只把原始帧存进设备——不分配、不加锁、不打日志。RealtimeScope
    /// 让 RTSan 运行期检查。外层 libhcs 的 REAPURB / 重新提交接收缓冲本来就要进内核，不在此列。
    class Callback final : public Sdk::Callback {
    public:
        explicit Callback(Board& board)
            : board_(board) {}

        void can_receive_callback(
            typename Sdk::Spec::Can can, const libhcs::data::CanDataView& data,
            const libhcs::time::SampleTime& time) override {
            [[maybe_unused]] const hcs_utility::RealtimeScope realtime_scope;
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (auto* bus = board_.can_bus(can.data_id))
                bus->receive(data.can_id, data.can_data, time);
        }

        void uart_receive_callback(
            typename Sdk::Spec::Uart uart, const libhcs::data::UartDataView& data,
            const libhcs::time::SampleTime&) override {
            [[maybe_unused]] const hcs_utility::RealtimeScope realtime_scope;
            if (auto* port = board_.serial_port(uart.data_id))
                port->receive(data.uart_data);
        }

        void accelerometer_receive_callback(
            const libhcs::data::ImuAccelerometerDataView& data,
            const libhcs::time::SampleTime& time) override {
            [[maybe_unused]] const hcs_utility::RealtimeScope realtime_scope;
            if (auto* port = board_.imu_port())
                port->receive(
                    {.kind = device::ImuSample::Kind::kAccelerometer,
                     .x = data.x,
                     .y = data.y,
                     .z = data.z,
                     .timestamp_quarter_us = data.timestamp_quarter_us,
                     .time = time});
        }

        void gyroscope_receive_callback(
            const libhcs::data::ImuGyroscopeDataView& data,
            const libhcs::time::SampleTime& time) override {
            [[maybe_unused]] const hcs_utility::RealtimeScope realtime_scope;
            if (auto* port = board_.imu_port())
                port->receive(
                    {.kind = device::ImuSample::Kind::kGyroscope,
                     .x = data.x,
                     .y = data.y,
                     .z = data.z,
                     .timestamp_quarter_us = data.timestamp_quarter_us,
                     .time = time});
        }

    private:
        Board& board_;
    };

    /// 一个串口字段的字节数。libhcs 一个字段最多装约 1 KB，取小一些让它和 CAN 帧挤得进同一个包。
    static constexpr std::size_t kSerialChunkBytes = 256;

    Callback callback_{*this};
    std::unique_ptr<Sdk> sdk_; ///< 最后声明：先于 callback_ 析构
};

/// 导出为插件的板组件：在设备（派生类成员）析构之前先 shutdown()。
/// 接线结构体只有经它才是完整的类——直接导出 Gimbal 是编译错误，不是退出时的偶发崩溃。
///
///   struct Gimbal : Board<libhcs::board::Hpm5321> { ... };
///   using GimbalBoard = BoardComponent<Gimbal>;
///   PLUGINLIB_EXPORT_CLASS(hcs_core::hardware::GimbalBoard, hcs_executor::Component)
template <class Wiring>
class BoardComponent final : public Wiring {
public:
    ~BoardComponent() override { this->shutdown(); }

private:
    void sealed_by_board_component() const noexcept override {}
};

using libhcs::board::hcs::kClassic1M;

} // namespace hcs_core::hardware::board
