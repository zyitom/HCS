#include "hardware/board/board.hpp"

#include <algorithm>
#include <chrono>
#include <format>
#include <stdexcept>
#include <utility>

#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>

namespace hcs_core::hardware::board {

namespace {

/// 串口按丝印叫：uart0、uart7、dbus。
std::string uart_name(libhcs::data::DataId data_id) {
    using libhcs::data::DataId;
    switch (data_id) {
    case DataId::kUartDbus: return "dbus";
    case DataId::kUart0: return "uart0";
    case DataId::kUart1: return "uart1";
    case DataId::kUart2: return "uart2";
    case DataId::kUart3: return "uart3";
    case DataId::kUart7: return "uart7";
    case DataId::kUart10: return "uart10";
    default: return std::format("uart#{}", std::to_underlying(data_id));
    }
}

/// 设备的串口要求 → EP0 下发的设置。帧格式每一项都写明：留 0 就是"沿用固件当前"，
/// 而固件当前可能已被 CDC 改掉。
libhcs::board::hcs::UartSetting to_uart_setting(const device::SerialLine& line) {
    namespace vc = libhcs::board::hcs::vc;
    using Parity = device::SerialLine::Parity;
    libhcs::board::hcs::UartSetting setting{
        .baudrate = line.baudrate,
        .word_length = vc::kUartWordLength8,
        .parity = line.parity == Parity::kEven  ? vc::kUartParityEven
                : line.parity == Parity::kOdd   ? vc::kUartParityOdd
                                                : vc::kUartParityNone,
        .stop_bits = line.stop_bits == 2 ? vc::kUartStopBits2 : vc::kUartStopBits1,
    };
    if (line.rx_inverted)
        setting.rx_polarity =
            *line.rx_inverted ? vc::kUartRxPolarityInverted : vc::kUartRxPolarityNormal;
    return setting;
}

} // namespace

// ── CanBus ───────────────────────────────────────────────────────────────

CanBus::CanBus(
    BoardCore& board, std::size_t index, libhcs::board::hcs::CanPort port,
    const libhcs::board::hcs::CanSetting& setting)
    : board_(board)
    , index_(index)
    , port_(port)
    , setting_(setting) {
    board.add(*this);
}

std::string CanBus::label() const {
    return std::format("{}/can{}", board_.get_component_name(), std::to_underlying(port_));
}

void CanBus::attach(CanDevice& device) {
    const auto fail = [&](const CanDevice& other, std::string_view what) {
        throw std::invalid_argument(
            std::format("{}: {} and {} {}", label(), other.name(), device.name(), what));
    };

    if (device.fd() != setting_.fd || device.bitrate() != setting_.arbitration_baudrate)
        throw std::invalid_argument(std::format(
            "{}: {} needs a {} bus at {} bit/s, the bus is declared {} at {} bit/s", label(),
            device.name(), device.fd() ? "CAN FD" : "classic CAN", device.bitrate(),
            setting_.fd ? "CAN FD" : "classic CAN", setting_.arbitration_baudrate));

    const auto feedback = device.feedback_id();
    const auto command = device.command_id();
    const auto slot = device.command_slot();

    if (feedback >= table_.size() || command >= table_.size())
        throw std::invalid_argument(std::format(
            "{}: {} uses id 0x{:x}/0x{:x}, beyond the 11-bit standard id range", label(),
            device.name(), feedback, command));

    for (const auto* other : devices_) {
        // 两台的反馈落在同一个 id 上：只会有一台被解码，另一台永远"离线"，没有任何报错。
        if (other->feedback_id() == feedback)
            fail(*other, std::format("both report on feedback id 0x{:x}", feedback));
        // 一台的指令 id 是另一台的反馈 id：另一台的反馈帧会被这一台当成指令收下。
        if (other->command_id() == feedback || other->feedback_id() == command)
            fail(*other, std::format(
                             "mix a command id with a feedback id (0x{:x}/0x{:x} vs 0x{:x}/0x{:x})",
                             other->feedback_id(), other->command_id(), feedback, command));
        // 指令 id 相同：只允许几台合一帧（DJI）且各写各的槽位。
        if (other->command_id() == command) {
            const auto other_slot = other->command_slot();
            if (!slot || !other_slot)
                fail(*other, std::format("both command on id 0x{:x}", command));
            if (*slot == *other_slot)
                fail(*other, std::format(
                                 "write the same slot {} of the shared frame 0x{:x}", *slot,
                                 command));
        }
    }

    devices_.push_back(&device);
    table_[feedback] = &device;
}

// ── SerialPort ───────────────────────────────────────────────────────────

SerialPort::SerialPort(BoardCore& board, std::size_t index, libhcs::data::DataId data_id)
    : board_(board)
    , index_(index)
    , data_id_(data_id) {
    board.add(*this);
}

std::string SerialPort::label() const {
    return std::format("{}/{}", board_.get_component_name(), uart_name(data_id_));
}

std::optional<libhcs::board::hcs::UartSetting> SerialPort::setting() const {
    if (!device_)
        return std::nullopt;
    return to_uart_setting(device_->serial_line());
}

void SerialPort::attach(SerialDevice& device) {
    if (device_)
        throw std::invalid_argument(std::format(
            "{}: {} and {} on one serial port; a byte stream has no id to tell them apart",
            label(), device_->name(), device.name()));
    device_ = &device;
}

// ── ImuPort ──────────────────────────────────────────────────────────────

ImuPort::ImuPort(BoardCore& board)
    : board_(board) {
    board.add(*this);
}

std::string ImuPort::label() const { return std::format("{}/imu", board_.get_component_name()); }

void ImuPort::attach(ImuDevice& device) {
    if (device_)
        throw std::invalid_argument(std::format(
            "{}: {} and {} on one onboard imu; there is only one sensor to read", label(),
            device_->name(), device.name()));
    device_ = &device;
}

// ── BoardCore ────────────────────────────────────────────────────────────

/// 指令侧伙伴：设备的 /control_* 输入都注册在它身上，它排在所有控制器之后。
class BoardCore::Command : public hcs_executor::Component {
public:
    explicit Command(BoardCore& owner)
        : owner_(owner) {}

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        owner_.command_update(tick);
    }

    /// 安全量（/hardware/safe）：接了就按它发失能帧；没接（参数为空）就从不锁存。
    InputInterface<bool> safe;

private:
    BoardCore& owner_;
};

BoardCore::BoardCore()
    : Node{
          get_component_name(),
          rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
    , command_(create_partner_component<Command>(get_component_name() + "_command", *this)) {
    register_input(param<std::string>("transmitter", "/hardware/transmitter"), transmitter_);
    if (const auto safety = param<std::string>("safety", "/hardware/safe"); !safety.empty())
        command_->register_input(safety, command_->safe);
}

// 这里不能调 close_sdk()（纯虚，派生部分已析构）：板卡由 BoardComponent<> 先关。
BoardCore::~BoardCore() { leave_transmitter(); }

hcs_executor::Component& BoardCore::command() noexcept { return *command_; }

std::vector<Device*> BoardCore::devices() const {
    std::vector<Device*> devices;
    for (const auto* bus : can_buses_)
        devices.insert(devices.end(), bus->devices().begin(), bus->devices().end());
    for (const auto* port : serial_ports_)
        if (port->device())
            devices.push_back(port->device());
    if (imu_port_ && imu_port_->device())
        devices.push_back(imu_port_->device());
    return devices;
}

// 端口是否属于这个板型由描述符的类型在编译期保证，这里只防同一个口声明两次。
void BoardCore::add(CanBus& bus) {
    auto& slot = can_by_index_.at(bus.index());
    if (slot)
        throw std::invalid_argument(std::format("{} declared twice", bus.label()));
    slot = &bus;
    can_buses_.push_back(&bus);
}

void BoardCore::add(SerialPort& serial) {
    auto& slot = serial_by_index_.at(serial.index());
    if (slot)
        throw std::invalid_argument(std::format("{} declared twice", serial.label()));
    slot = &serial;
    serial_ports_.push_back(&serial);
}

void BoardCore::add(ImuPort& imu) {
    if (imu_port_)
        throw std::invalid_argument(std::format("{} declared twice", imu.label()));
    imu_port_ = &imu;
}

libhcs::board::hcs::Configuration BoardCore::configuration() const {
    // 声明了的 CAN 口按声明的帧型与速率下发：板端重初始化控制器（经典即关掉 FD，2.0 总线上
    // 任何 FD 位都会让总线崩溃）并自己回读确认，不符即构造失败；重连自动重放。
    // 挂了设备的串口按设备的要求下发。
    libhcs::board::hcs::Configuration config;
    for (const auto* bus : can_buses_)
        config.can[bus->index()] = bus->setting();
    for (const auto* serial : serial_ports_)
        config.uart[serial->index()] = serial->setting();
    return config;
}

void BoardCore::before_updating() {
    devices_ = devices();
    for (const auto* bus : can_buses_)
        for (const auto* device : bus->devices())
            logger().info("{} {}: {}", bus->label(), device->name(), device->describe());
    for (const auto* serial : serial_ports_)
        if (const auto* device = serial->device())
            logger().info("{} {}: {}", serial->label(), device->name(), device->describe());
    if (imu_port_)
        if (const auto* device = imu_port_->device())
            logger().info("{} {}: {}", imu_port_->label(), device->name(), device->describe());

    try {
        open();
    } catch (...) {
        shutdown();
        throw;
    }
}

void BoardCore::open() {
    if (!param<bool>("enabled", false)) {
        logger().warn("[{}] disabled by parameter; its devices stay offline", get_component_name());
        return;
    }

    auto options = libhcs::board::AdvancedOptions{};
    options.dangerously_skip_version_checks = param<bool>("dangerously_skip_version_checks", false);
    const auto io_cpu = param<std::int64_t>("io_thread_cpu", -1);
    const auto io_priority = param<std::int64_t>("io_thread_rt_priority", 0);
    if (io_cpu >= 0)
        options.set_io_thread_affinity(static_cast<int>(io_cpu), static_cast<int>(io_priority));

    // 构造期 Configuration：before-session 同步 apply 并由板端回读，被拒即在构造里抛；
    // 重连自动重放。板卡构造期就可能回调：所有设备此时都已登记。
    const auto serial_filter = param<std::string>("serial_filter", "");
    open_sdk(serial_filter, options, configuration());

    if (link_faulted())
        throw std::runtime_error(
            std::format("{}: board faulted at construction", get_component_name()));

    // 声明的 CAN 路数板上真有（单 CAN 与双 CAN 的 5321 是同一个固件）。
    std::size_t need = 0;
    for (const auto* bus : can_buses_)
        need = std::max(need, bus->index() + 1);
    if (const auto have = can_count(); have < need)
        throw std::runtime_error(std::format(
            "{}: the board carries {} CAN buses but the wiring declares {}", get_component_name(),
            have, need));

    logger().info(
        "[{}] opened: serial_filter='{}' io_cpu={} io_prio={}", get_component_name(),
        serial_filter, static_cast<long long>(io_cpu), static_cast<long long>(io_priority));

    util::TransmitBatch safe_batch;
    pack_frames(safe_batch, true);
    transmitter_ref_ = *transmitter_;
    lane_ = &transmitter_ref_->attach(*this, safe_batch);
    report_bus_load();

    report_timer_ = create_wall_timer(std::chrono::seconds{1}, [this] { report(); });
}

void BoardCore::shutdown() noexcept {
    leave_transmitter(); // 先离开发送线程：它会调本板的 send()
    close_sdk();         // 再停 IO 线程：它会回调进设备
}

void BoardCore::leave_transmitter() noexcept {
    if (report_timer_) {
        report_timer_->cancel();
        report_timer_.reset();
    }
    if (lane_) {
        transmitter_ref_->detach(*lane_);
        lane_ = nullptr;
    }
}

// ── 周期域 ───────────────────────────────────────────────────────────────

void BoardCore::update(const hcs_sync::Tick& tick) HCS_NONBLOCKING {
    // 回调线程只存原始帧，这里解成物理量写进输出
    for (auto* device : devices_)
        device->update_status(tick);
}

// 锁存期间整拍改发全失能帧，与构造期安全批次同一套 pack_frames(safe = true)。
// 仍每拍发：发送线程只在 fresh 时发，停发就回到"不发任何指令"，而 DM 失能帧
// 本身也是它回反馈的唯一理由——停发之后连复位要看的 online 都拿不到了。
void BoardCore::command_update(const hcs_sync::Tick& tick) HCS_NONBLOCKING {
    if (!lane_)
        return;
    const bool safe = command_->safe.has_provider() && *command_->safe;
    util::TransmitBatch batch;
    batch.sequence = ++batch_sequence_;
    pack_frames(batch, safe);
    lane_->publish(batch, tick.scheduled);
}

// 怎么编码、失能帧长什么样、要不要几台合一帧，都是各驱动 append_command() 自己的事；
// 这里只负责"哪台设备在哪路总线上"。
void BoardCore::pack_frames(util::TransmitBatch& batch, bool safe) HCS_NONBLOCKING {
    util::CommandFrames frames{batch};
    for (const auto* bus : can_buses_) {
        auto frames_on_bus = frames.on(bus->port());
        for (auto* device : bus->devices())
            device->append_command(frames_on_bus, safe);
    }
    frames.flush();
}

// ── 尽力域 ───────────────────────────────────────────────────────────────

// 每路总线负载 = 帧数/拍 × 拍率 × 130 µs（经典 CAN 1 Mbit/s 8 字节帧上界），>70% 告警
void BoardCore::report_bus_load() {
    util::TransmitBatch batch;
    pack_frames(batch, true);
    const double update_rate = param<double>("expected_update_rate", 1000.0);
    std::string summary;
    for (const auto* bus : can_buses_) {
        unsigned frames = 0;
        for (const auto& frame : std::span{batch.frames}.first(batch.frame_count))
            frames += frame.port == std::to_underlying(bus->port());
        const double load = frames * update_rate * 130e-6 * 100.0;
        summary += std::format(" {}={:.0f}%", bus->label(), load);
        if (load > 70.0)
            logger().warn("bus {} at {:.0f}% estimated load (>70%)", bus->label(), load);
    }
    logger().info("bus load estimate at {:.0f} Hz:{}", update_rate, summary);
}

void BoardCore::report() {
    for (auto* device : devices_)
        if (const auto problem = device->take_problem())
            logger().error("{}: {}", device->name(), *problem);

    if (link_faulted()) {
        if (param<bool>("exit_on_board_fault", true)) {
            logger().fatal("[{}] board faulted (link lost); shutting down", get_component_name());
            rclcpp::shutdown();
            return;
        }
        if (link_fault_report_.ready())
            logger().error(
                "[{}] board faulted (link lost); its devices are offline", get_component_name());
        return;
    }
    report_can_health();
}

void BoardCore::log_can_status(
    libhcs::board::hcs::CanPort port, const libhcs::board::hcs::vc::CanStatusPayload& status) {
    namespace vc = libhcs::board::hcs::vc;
    const unsigned flags = status.flags;
    const bool flagged = (flags & (vc::kCanErrorPassive | vc::kCanWarning | vc::kCanBusOff)) != 0;
    if (!flagged && status.tec == 0 && status.rec == 0 && status.rx_fifo_level == 0)
        return;
    logger().warn(
        "can{}: tec={} rec={} last={} flags=0x{:02x} rx_backlog={}", static_cast<unsigned>(port),
        static_cast<unsigned>(status.tec), static_cast<unsigned>(status.rec),
        libhcs::board::hcs::last_error_name(status.last_error), static_cast<unsigned>(flags),
        static_cast<unsigned>(status.rx_fifo_level));
}

void BoardCore::log_can_status_failure(
    libhcs::board::hcs::CanPort port, const std::exception& error) {
    logger().warn("can{} status read failed: {}", static_cast<unsigned>(port), error.what());
}

} // namespace hcs_core::hardware::board
