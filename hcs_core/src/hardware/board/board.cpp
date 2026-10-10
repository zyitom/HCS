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

/// 设备的串口要求 → EP0 下发的设置。声明必须说全（libhcs 拒绝缺项的声明）：速率、帧格式、
/// 接收极性一项不落。
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
        .rx_polarity = line.rx_inverted.value_or(false) ? vc::kUartRxPolarityInverted
                                                       : vc::kUartRxPolarityNormal,
    };
    return setting;
}

} // namespace

// ── CanBus ───────────────────────────────────────────────────────────────

CanBus::CanBus(
    BoardCore& board, libhcs::data::DataId data_id,
    const libhcs::board::hcs::CanSetting& setting)
    : board_(board)
    , data_id_(data_id)
    , setting_(setting) {
    board.add(*this);
}

std::string CanBus::label() const {
    return std::format(
        "{}/{}", board_.get_component_name(),
        libhcs::spec::channel_name(data_id_));
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

SerialPort::SerialPort(BoardCore& board, libhcs::data::DataId data_id)
    : board_(board)
    , data_id_(data_id) {
    board.add(*this);
}

std::string SerialPort::label() const {
    return std::format("{}/{}", board_.get_component_name(), libhcs::spec::channel_name(data_id_));
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
    transmits_ = device.transmits();
}

// ── ImuPort ──────────────────────────────────────────────────────────────

ImuPort::ImuPort(BoardCore& board)
    : board_(board) {
    board.add(*this);
}

std::string ImuPort::label() const { return std::format("{}/imu", board_.get_component_name()); }

std::optional<libhcs::board::hcs::ImuSetting> ImuPort::setting() const {
    if (!device_)
        return std::nullopt;
    const device::ImuSetup setup = device_->imu_setup();
    return libhcs::board::hcs::ImuSetting{
        .accelerometer_range_g = setup.accelerometer_range_g,
        .accelerometer_rate_hz = setup.accelerometer_rate_hz,
        .gyroscope_range_dps = setup.gyroscope_range_dps,
        .gyroscope_rate_hz = setup.gyroscope_rate_hz,
        .gyroscope_bandwidth_hz = setup.gyroscope_bandwidth_hz,
    };
}

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

// 到这里板卡早已由 BoardComponent<> 经 shutdown() 关掉；构造中途抛出时则根本没打开过。
// 留着这一句只为了那条路径上定时器和 Lane 也确实是空的。
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
    auto& slot = can_by_id_.at(std::to_underlying(bus.data_id()));
    if (slot)
        throw std::invalid_argument(std::format("{} declared twice", bus.label()));
    slot = &bus;
    can_buses_.push_back(&bus);
}

void BoardCore::add(SerialPort& serial) {
    auto& slot = serial_by_id_.at(std::to_underlying(serial.data_id()));
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
    // 挂了设备的串口、板载 IMU 按设备的要求下发。键是口的身份（DataId = 丝印号）。
    libhcs::board::hcs::Configuration config;
    for (const auto* bus : can_buses_)
        config.declare_can(bus->data_id(), bus->setting());
    for (const auto* serial : serial_ports_)
        if (const auto setting = serial->setting())
            config.declare_uart(serial->data_id(), *setting);
    if (imu_port_)
        if (const auto setting = imu_port_->setting())
            config.declare_imu(*setting);
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
    // 共享时间基准：写进声明（清单头的一位），板子接受了才开。开了之后 libhcs 每个保活周期带
    // 一次对时，并去读这块板所在 USB 主机控制器的微帧计数器（读不了就退回按 USB 往返估计，
    // 日志里有一行）。板子报告没有时间基准时构造即抛。
    const bool time_sync = param<bool>("time_sync", false);
    const auto io_cpu = param<std::int64_t>("io_thread_cpu", -1);
    const auto io_priority = param<std::int64_t>("io_thread_rt_priority", 0);
    if (io_cpu >= 0)
        options.set_io_thread_affinity(static_cast<int>(io_cpu), static_cast<int>(io_priority));

    // 构造期 Configuration：before-session 同步 apply 并由板端回读，被拒即在构造里抛；
    // 重连自动重放。板卡构造期就可能回调：所有设备此时都已登记。
    const auto serial_filter = param<std::string>("serial_filter", "");
    auto declaration = configuration();
    declaration.enable_time_sync(time_sync);
    open_sdk(serial_filter, options, declaration);

    if (link_faulted())
        throw std::runtime_error(
            std::format("{}: board faulted at construction", get_component_name()));

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
    // 回调线程只存原始帧和 SDK 算好的样本时刻，这里解成物理量写进输出
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
    // 串口的字节不进批次：各设备写进自己的发件箱，发送线程发这一拍的批次时一并取走。
    for (const auto* serial : serial_ports_)
        if (auto* device = serial->device())
            device->append_command(safe);
    lane_->publish(batch, tick.scheduled);
}

// 怎么编码、失能帧长什么样、要不要几台合一帧，都是各驱动 append_command() 自己的事；
// 这里只负责"哪台设备在哪路总线上"。只装 CAN 帧：它也用来拼预存的失能批次、估总线负载，
// 那两处不许真的往串口发件箱里写东西。
void BoardCore::pack_frames(util::TransmitBatch& batch, bool safe) HCS_NONBLOCKING {
    util::CommandFrames frames{batch};
    for (const auto* bus : can_buses_) {
        auto frames_on_bus = frames.on(bus->data_id());
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
    for (const auto* bus : can_buses_) {
        unsigned frames = 0;
        for (const auto& frame : std::span{batch.frames}.first(batch.frame_count))
            frames += frame.port == bus->data_id();
        const double load = frames * update_rate * 130e-6 * 100.0;
        logger().info(
            "bus {} estimated load at {:.0f} Hz: {:.0f}%", bus->label(), update_rate, load);
        if (load > 70.0)
            logger().warn("bus {} at {:.0f}% estimated load (>70%)", bus->label(), load);
    }
}

namespace {

// 板子的计数从上电起自由累加、线上按 2^16 回绕：相邻两次之差取模 2^16。板子每轮计数一动
// 就报，一轮 250 ms 里动不了 65536 次，差不会有歧义；板子重启必然换 SDK 对象（链路先报
// kFaulted），同一个基准不会跨过重启。
std::uint32_t since(std::uint16_t now, std::uint16_t before) noexcept {
    return static_cast<std::uint16_t>(now - before);
}

} // namespace

// 端口健康检查（1 Hz）：只看接线表声明过的口——没声明的口板子不报，这层也不碰。读的是
// 板子随 keepalive 推送的快照（不发 USB 请求，不会失败）；与上一秒比，有变化才打一行。
// 总线状态的跳变（error-passive / bus-off）libhcs 收到时已经打过，这里报这一秒的增量。
void BoardCore::report_port_health() {
    if (!opened())
        return;
    // 链路本身：板子没能交付的下行记录（主机或固件的帧定界 bug，或往没有的口发了数据）。
    // 逐条的 error 日志 libhcs 已经打过，这里报这一秒的增量。
    {
        const auto now = status<libhcs::data::LinkStatusView>(libhcs::data::DataId::kSession);
        const auto before = std::exchange(link_status_seen_, now);
        if (const auto errors = before ? since(now.downlink_errors, before->downlink_errors) : 0U)
            logger().warn(
                "{}: board failed to deliver {} downlink record(s) in the last second; last: {} "
                "({})",
                get_component_name(), errors, libhcs::spec::channel_name(now.last_field),
                libhcs::data::downlink_error_name(now.last_reason));
    }
    for (auto* bus : can_buses_) {
        const auto now = status<libhcs::data::CanStatusView>(bus->data_id());
        const auto before = bus->exchange_status(now);
        if (!before || now == *before)
            continue;

        // 单发：输了仲裁或出错的帧就此放弃，这种丢失不动 TEC/REC（三块板都计）。
        if (const auto abandoned = since(now.tx_cancelled, before->tx_cancelled))
            logger().warn(
                "{}: {} frame(s) abandoned in the last second (single-shot: lost arbitration "
                "or bus error)",
                bus->label(), abandoned);
        // 板子自己丢的：下行撞上发送队列满，上行撞上上行缓冲满。
        const auto tx_dropped = since(now.tx_dropped, before->tx_dropped);
        const auto rx_dropped = since(now.rx_dropped, before->rx_dropped);
        if (tx_dropped != 0 || rx_dropped != 0)
            logger().warn(
                "{}: board dropped {} downlink / {} uplink frame(s) in the last second (buffer "
                "full)",
                bus->label(), tx_dropped, rx_dropped);
        // 控制器的接收 FIFO 溢出：帧在控制器里就丢了，一次至少一帧。
        if (const auto lost = since(now.rx_lost, before->rx_lost))
            logger().warn(
                "{}: receive FIFO overflowed {} time(s) in the last second (frames lost in the "
                "controller)",
                bus->label(), lost);
        if (now.tec != before->tec || now.rec != before->rec || now.flags != before->flags
            || now.last_error != before->last_error
            || now.data_last_error != before->data_last_error)
            logger().warn(
                "{}: tec={} rec={} last={} data_last={} flags=0x{:02x}", bus->label(),
                static_cast<unsigned>(now.tec), static_cast<unsigned>(now.rec),
                libhcs::data::last_error_name(now.last_error),
                libhcs::data::last_error_name(now.data_last_error),
                static_cast<unsigned>(now.flags));
    }
    for (auto* port : serial_ports_) {
        const auto now = status<libhcs::data::UartStatusView>(port->data_id());
        const auto before = port->exchange_status(now);
        if (!before || now == *before)
            continue;

        const auto overrun = since(now.overrun, before->overrun);
        const auto parity = since(now.parity, before->parity);
        const auto framing = since(now.framing, before->framing);
        const auto noise = since(now.noise, before->noise);
        const auto unattributed = since(now.unattributed, before->unattributed);
        if (overrun != 0 || parity != 0 || framing != 0 || noise != 0 || unattributed != 0)
            logger().warn(
                "{}: receive errors in the last second: {} overrun, {} parity, {} framing, {} "
                "noise, {} of unknown kind",
                port->label(), overrun, parity, framing, noise, unattributed);
        const auto tx_dropped = since(now.tx_dropped, before->tx_dropped);
        const auto rx_dropped = since(now.rx_dropped, before->rx_dropped);
        if (tx_dropped != 0 || rx_dropped != 0)
            logger().warn(
                "{}: board dropped {} downlink / {} uplink chunk(s) in the last second (buffer "
                "full)",
                port->label(), tx_dropped, rx_dropped);
    }
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
    report_port_health();
}

} // namespace hcs_core::hardware::board
