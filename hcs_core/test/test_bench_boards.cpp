// 上板测试：真板子、真 USB、真总线。只有硬件能回答的问题放在这里——
// "接线表声明了的口通不通"、"没声明的口是不是真的不工作"。
//
// "没声明的口"有两道关，各测各的：
//   主机 SDK   板卡类的发送入口直接抛
//   libhcs::board::hcs::UndeclaredChannel（调用点就知道写错了）板子
//   不经板卡类的主机（RawHost）照样把数据发下去，板子收到就丢（最后一道）
//
// 台架有三种形态，插着哪些板子就跑哪一种（CAN
// 是总线，三块板同时挂在上面时"没声明的口没人
// 应答"这类检查不成立，所以各形态互斥，按插着的板子数区分）：
//
//   Bench           两块双 CAN 的 HPM5321，没有 mc02。CAN1 对 CAN1、CAN2 对 CAN2、UART0 交叉对接
//                   （A 的 TX 接 B 的 RX，反之亦然）。
//   Mc02Bench       一块双 CAN 的 HPM5321 加一块 mc02。CAN1 对 CAN1、CAN2 对 CAN2，5321 的 UART0
//                   与 mc02 的 UART1（USART1）交叉对接。
//   CBoardBench     一块双 CAN 的 HPM5321、一块 mc02、一块 c_board。三块板的 CAN1 挂在同一条
//                   总线上（c_board 只有经典 CAN）；c_board 的丝印 UART1 与 mc02 的一个 TTL 串口
//                   交叉对接，是哪一个开测时探出来。插着 c_board 时其余三种形态都跳过。
//   ThreeBoardBench 两块双 CAN 的 HPM5321 加一块 mc02（实车的组合）。三块板的 CAN1 挂在同一条
//                   总线上（总线两端各一个 120 欧，断电量 CANH-CANL 约 60 欧），三块板插同一个
//                   USB 主机控制器。只测时间戳：同一帧被两块板同时收到，两个时间戳之差就是
//                   跨板误差的直接读数。时间基准是声明的一部分（Configuration::time_sync）。
//
// 板子的固件必须是从当前 libhcs
// 编出来的那一版：版本不符时握手就会失败，测试报的就是那句 "Reflash the board"。
//
// 板子凑不成某种形态时那一组全部跳过（结果里是 SKIPPED，不是失败），所以平时的
// colcon test 不受影响；插着板子时它就真的跑，跑不过就是真的有问题。
//
//   HCS_BENCH=0                    强制跳过（板子插着但线没接成台架形态时用）
//   HCS_BENCH_BOARD_A / _B         Bench：指定两块板的 USB
//   序列号子串；不设则取找到的前两块
//                                  （按序列号排序）
//
// 每个用例自己打开板子、用完关掉：libhcs
// 里"配置即声明"，一次握手声明一组口，换一组声明
// 就是换一次握手，这本身就是被测的东西。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include <libhcs/board/c_board.hpp>
#include <libhcs/board/hpm5321.hpp>
#include <libhcs/board/mc02.hpp>
#include <libhcs/protocol/usb_identity.hpp>
#include <libhcs/time/sample_time.hpp>
#include <libhcs/time/sof_stamp.hpp>

namespace {

using namespace std::chrono_literals;
using libhcs::board::CBoard;
using libhcs::board::Hpm5321;
using libhcs::board::Mc02;
using libhcs::board::hcs::CanSetting;
using libhcs::board::hcs::Configuration;
using libhcs::board::hcs::ImuSetting;
using libhcs::board::hcs::kClassic1M;
using libhcs::board::hcs::kFd1M5M;
using libhcs::board::hcs::uart_8n1;
using libhcs::board::hcs::UartSetting;
using libhcs::board::hcs::UndeclaredChannel;
using libhcs::data::DataId;
namespace vc = libhcs::board::hcs::vc;

/// 帧到了就算通：给足余量，台架上实际是一两毫秒的事。
constexpr auto kArrival = 500ms;
/// 确认"没有东西来"要等多久。
constexpr auto kSilence = 300ms;
/// 握手之后控制器上总线、串口武装接收所需的时间。
constexpr auto kSettle = 50ms;
/// 板间串口对接用的速率：5321 的 80 MHz 时钟下能精确分出来。
constexpr std::uint32_t kUartBaudrate = 1'000'000;
/// mc02 的丝印 UART1（EP0 按 DataId 寻址）。
constexpr DataId kMc02Uart1 = DataId::kUart1;

// ── 找板子 ───────────────────────────────────────────────────────────────

std::string read_line(const std::filesystem::path& path) {
    std::ifstream file{path};
    std::string line;
    std::getline(file, line);
    return line;
}

/// 这台机器上 USB 身份为 vendor:product 的设备的序列号，按字典序。
std::vector<std::string> attached(std::uint16_t vendor_id, std::uint16_t product_id) {
    char vendor[8], product[8];
    std::snprintf(vendor, sizeof(vendor), "%04x", vendor_id);
    std::snprintf(product, sizeof(product), "%04x", product_id);

    std::vector<std::string> serials;
    std::error_code error;
    for (const auto& entry : std::filesystem::directory_iterator{"/sys/bus/usb/devices", error}) {
        if (read_line(entry.path() / "idVendor") != vendor
            || read_line(entry.path() / "idProduct") != product)
            continue;
        if (auto serial = read_line(entry.path() / "serial"); !serial.empty())
            serials.push_back(std::move(serial));
    }
    std::ranges::sort(serials);
    return serials;
}

/// 正在跑应用固件的双 CAN HPM5321。
std::vector<std::string> attached_boards() {
    namespace identity = libhcs::core::protocol::usb_identity;
    return attached(identity::kDmtoolVendorId, identity::kHpm5321DualCanProductId);
}

/// mc02 的 USB 身份（libhcs::board::Mc02 里写死的那一对）。
constexpr std::uint16_t kMc02VendorId = 0xA511;
constexpr std::uint16_t kMc02ProductId = 0x0723;

/// mc02。它的 bootloader 用的是同一个 USB 身份：停在 DFU 里的板也会被数进来，
/// 那时握手失败，测试报错而不是跳过——板子插着却不能用，本来就该让人看见。
std::vector<std::string> attached_mc02() { return attached(kMc02VendorId, kMc02ProductId); }

/// c_board（libhcs::board::CBoard 里写死的那一对）。bootloader 的 PID 相同，同 mc02。
constexpr std::uint16_t kCBoardVendorId = 0xA511;
constexpr std::uint16_t kCBoardProductId = 0xF407;
std::vector<std::string> attached_cboard() { return attached(kCBoardVendorId, kCBoardProductId); }

bool bench_disabled() {
    const char* enabled = std::getenv("HCS_BENCH");
    return enabled != nullptr && *enabled == '0';
}

/// Bench 形态的两块板；凑不齐（或插着 mc02，那是另一种形态）就是空。
std::optional<std::array<std::string, 2>> bench() {
    if (bench_disabled() || !attached_mc02().empty() || !attached_cboard().empty())
        return std::nullopt;

    auto boards = attached_boards();
    const auto pick = [&](const char* variable) -> std::optional<std::string> {
        const char* wanted = std::getenv(variable);
        if (wanted == nullptr || *wanted == '\0')
            return std::nullopt;
        const auto found = std::ranges::find_if(
            boards, [&](const std::string& serial) { return serial.contains(wanted); });
        if (found == boards.end())
            return std::string{}; // 点名的那块不在：当成台架不齐
        std::string serial = *found;
        boards.erase(found);
        return serial;
    };
    const auto a = pick("HCS_BENCH_BOARD_A");
    const auto b = pick("HCS_BENCH_BOARD_B");
    if ((a && a->empty()) || (b && b->empty()))
        return std::nullopt;

    std::array<std::string, 2> chosen;
    for (auto* slot : {&a, &b}) {
        auto& target = chosen[slot == &a ? 0 : 1];
        if (*slot) {
            target = **slot;
        } else if (!boards.empty()) {
            target = boards.front();
            boards.erase(boards.begin());
        } else {
            return std::nullopt;
        }
    }
    return chosen;
}

/// ThreeBoardBench 形态的三块板 {5321 A, 5321 B, mc02}：5321 恰好两块、mc02 恰好一块。
std::optional<std::array<std::string, 3>> three_board_bench() {
    if (bench_disabled() || !attached_cboard().empty())
        return std::nullopt;
    auto boards = attached_boards();
    auto mc02 = attached_mc02();
    if (boards.size() != 2 || mc02.size() != 1)
        return std::nullopt;
    return std::array{std::move(boards[0]), std::move(boards[1]), std::move(mc02.front())};
}

/// CBoardBench 形态的三块板 {5321, mc02, c_board}：各恰好一块，否则是空。
std::optional<std::array<std::string, 3>> cboard_bench() {
    if (bench_disabled())
        return std::nullopt;
    auto boards = attached_boards();
    auto mc02 = attached_mc02();
    auto cboard = attached_cboard();
    if (boards.size() != 1 || mc02.size() != 1 || cboard.size() != 1)
        return std::nullopt;
    return std::array{std::move(boards.front()), std::move(mc02.front()), std::move(cboard.front())};
}

/// Mc02Bench 形态的两块板 {5321, mc02}：各恰好一块，否则是空。
std::optional<std::array<std::string, 2>> mc02_bench() {
    if (bench_disabled() || !attached_cboard().empty())
        return std::nullopt;
    auto boards = attached_boards();
    auto mc02 = attached_mc02();
    if (boards.size() != 1 || mc02.size() != 1)
        return std::nullopt;
    return std::array{std::move(boards.front()), std::move(mc02.front())};
}

// ── 一块打开的板 ─────────────────────────────────────────────────────────

struct Frame {
    DataId port;
    std::uint32_t can_id;
    std::vector<std::byte> data;
};

/// libhcs 的 IO 线程把收到的东西交到这里，测试线程来问。
class Inbox {
public:
    void push_frame(DataId port, const libhcs::data::CanDataView& data) {
        const std::scoped_lock lock{mutex_};
        frames_.push_back({
            port, data.can_id, {data.can_data.begin(), data.can_data.end()}
        });
        changed_.notify_all();
    }

    void push_bytes(const libhcs::data::UartDataView& data) {
        const std::scoped_lock lock{mutex_};
        bytes_.insert(bytes_.end(), data.uart_data.begin(), data.uart_data.end());
        changed_.notify_all();
    }

    /// 同上，另按口记一份（一块板开着几个串口时分得清是哪个口收到的）。
    void push_bytes(DataId port, const libhcs::data::UartDataView& data) {
        const std::scoped_lock lock{mutex_};
        bytes_.insert(bytes_.end(), data.uart_data.begin(), data.uart_data.end());
        auto& on_port = bytes_by_port_[port];
        on_port.insert(on_port.end(), data.uart_data.begin(), data.uart_data.end());
        changed_.notify_all();
    }

    /// 至今在 port 上收到的字节。
    std::vector<std::byte> bytes_on(DataId port) {
        const std::scoped_lock lock{mutex_};
        const auto found = bytes_by_port_.find(port);
        return found == bytes_by_port_.end() ? std::vector<std::byte>{} : found->second;
    }

    /// 等 port 上出现 id 为 can_id 的帧。
    std::optional<Frame>
        wait_frame(DataId port, std::uint32_t can_id, std::chrono::milliseconds timeout) {
        std::unique_lock lock{mutex_};
        const auto match = [&](const Frame& frame) {
            return frame.port == port && frame.can_id == can_id;
        };
        if (!changed_.wait_for(lock, timeout, [&] { return std::ranges::any_of(frames_, match); }))
            return std::nullopt;
        return *std::ranges::find_if(frames_, match);
    }

    /// 至今在 port 上收到的帧数。
    std::size_t frames_on(DataId port) {
        const std::scoped_lock lock{mutex_};
        return static_cast<std::size_t>(
            std::ranges::count_if(frames_, [&](const Frame& frame) { return frame.port == port; }));
    }

    /// 等串口上累计收到 size 个字节，返回至今收到的全部。
    std::vector<std::byte> wait_bytes(std::size_t size, std::chrono::milliseconds timeout) {
        std::unique_lock lock{mutex_};
        changed_.wait_for(lock, timeout, [&] { return bytes_.size() >= size; });
        return bytes_;
    }

    /// 丢掉至今收到的串口字节：换了速率或帧格式之后从头数。
    void clear_bytes() {
        const std::scoped_lock lock{mutex_};
        bytes_.clear();
        bytes_by_port_.clear();
    }

private:
    std::mutex mutex_;
    std::condition_variable changed_;
    std::vector<Frame> frames_;
    std::vector<std::byte> bytes_;
    std::map<DataId, std::vector<std::byte>> bytes_by_port_;
};

struct Listener final
    : Hpm5321::Callback
    , Inbox {
    void can_receive_callback(
        libhcs::spec::hpm5321::Spec::Can port, const libhcs::data::CanDataView& data,
        const libhcs::time::SampleTime&) override {
        push_frame(port.data_id, data);
    }
    void uart_receive_callback(
        libhcs::spec::hpm5321::Spec::Uart, const libhcs::data::UartDataView& data,
        const libhcs::time::SampleTime&) override {
        push_bytes(data);
    }
};

struct Mc02Listener final
    : Mc02::Callback
    , Inbox {
    void can_receive_callback(
        libhcs::spec::mc02::Spec::Can port, const libhcs::data::CanDataView& data,
        const libhcs::time::SampleTime&) override {
        push_frame(port.data_id, data);
    }
    void uart_receive_callback(
        libhcs::spec::mc02::Spec::Uart port, const libhcs::data::UartDataView& data,
        const libhcs::time::SampleTime&) override {
        push_bytes(port.data_id, data);
    }

    // 板载 IMU 的样本：只数个数，另留下加速度的平方和（板子静止时它就是重力）。
    void accelerometer_receive_callback(
        const libhcs::data::ImuAccelerometerDataView& data, const libhcs::time::SampleTime&)
        override {
        const double x = data.x, y = data.y, z = data.z;
        const std::scoped_lock lock{imu_mutex};
        ++accelerometer_samples;
        accelerometer_square_sum += x * x + y * y + z * z;
    }
    void gyroscope_receive_callback(const libhcs::data::ImuGyroscopeDataView&,
        const libhcs::time::SampleTime&) override {
        const std::scoped_lock lock{imu_mutex};
        ++gyroscope_samples;
    }

    /// 加速度原始值的均方根模长。
    [[nodiscard]] double accelerometer_rms() {
        const std::scoped_lock lock{imu_mutex};
        return accelerometer_samples == 0
                 ? 0.0
                 : std::sqrt(accelerometer_square_sum / static_cast<double>(accelerometer_samples));
    }
    [[nodiscard]] std::array<std::size_t, 2> imu_samples() {
        const std::scoped_lock lock{imu_mutex};
        return {accelerometer_samples, gyroscope_samples};
    }

    std::mutex imu_mutex;
    std::size_t accelerometer_samples = 0, gyroscope_samples = 0;
    double accelerometer_square_sum = 0.0;

    // GPIO 输入线的样本, 按到达顺序。
    struct GpioSample {
        std::uint8_t line;
        bool high;
        std::optional<std::uint32_t> timestamp;
    };
    void gpio_digital_read_result_callback(
        libhcs::spec::mc02::Spec::GpioLine line, const libhcs::data::GpioDigitalDataView& data,
        const libhcs::time::SampleTime&) override {
        const std::scoped_lock lock{gpio_mutex};
        gpio_samples.push_back({line.line, data.high, data.timestamp_quarter_us});
    }
    [[nodiscard]] std::vector<GpioSample> gpio() {
        const std::scoped_lock lock{gpio_mutex};
        return gpio_samples;
    }

    std::mutex gpio_mutex;
    std::vector<GpioSample> gpio_samples;
};

/// 接线表声明的那些口。uart 是对接串口的 DataId（5321 的 UART0 / mc02 的 UART1）。
Configuration declare(
    std::initializer_list<DataId> buses, std::optional<DataId> uart = std::nullopt,
    const CanSetting& bus_setting = kClassic1M) {
    Configuration configuration;
    for (const DataId port : buses)
        configuration.declare_can(port, bus_setting);
    if (uart)
        configuration.declare_uart(*uart, uart_8n1(kUartBaudrate));
    return configuration;
}

/// 同一份声明，另要共享时间基准。
Configuration with_time_sync(Configuration configuration) {
    configuration.enable_time_sync();
    return configuration;
}

/// 一块按给定声明打开的板。构造返回时握手与会话都已建立；失败则抛出。
struct Node {
    Node(const std::string& serial, const Configuration& configuration)
        : board(
              listener, serial,
              // 与部署配置一致：产品串里的版本号在开发机上对不上，真正的门是 EP0 的线格式指纹。
              libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true),
              configuration) {}

    void send(DataId port, std::uint32_t can_id, std::span<const std::byte> data) {
        board.start_transmit().can_transmit(
            libhcs::spec::hpm5321::Spec::Can{port}, {.can_id = can_id, .can_data = data});
    }

    void send_uart(std::span<const std::byte> data) {
        board.start_transmit().uart_transmit(
            libhcs::spec::hpm5321::Spec::Uarts::kUart0, {.uart_data = data});
    }

    Listener listener; // 先于 board 构造、后于它析构：IO 线程回调进来时它必须在
    Hpm5321 board;
};

/// 同上，mc02。对接的串口是丝印 UART1。
struct Mc02Node {
    Mc02Node(const std::string& serial, const Configuration& configuration)
        : board(
              listener, serial,
              libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true),
              configuration) {}

    void send(DataId port, std::uint32_t can_id, std::span<const std::byte> data) {
        board.start_transmit().can_transmit(
            libhcs::spec::mc02::Spec::Can{port}, {.can_id = can_id, .can_data = data});
    }

    void send_uart(std::span<const std::byte> data) {
        board.start_transmit().uart_transmit(
            libhcs::spec::mc02::Spec::Uarts::kUart1, {.uart_data = data});
    }

    void send_uart(DataId port, std::span<const std::byte> data) {
        board.start_transmit().uart_transmit(
            libhcs::spec::mc02::Spec::Uart{port}, {.uart_data = data});
    }

    Mc02Listener listener;
    Mc02 board;
};

/// c_board：两路经典 CAN、三个串口（DBUS / 丝印 UART1 / UART2）、板载 BMI088。
struct CBoardListener final
    : CBoard::Callback
    , Inbox {
    void can_receive_callback(
        libhcs::spec::c_board::Spec::Can port, const libhcs::data::CanDataView& data,
        const libhcs::time::SampleTime&) override {
        push_frame(port.data_id, data);
    }
    void uart_receive_callback(
        libhcs::spec::c_board::Spec::Uart port, const libhcs::data::UartDataView& data,
        const libhcs::time::SampleTime&) override {
        push_bytes(port.data_id, data);
    }
    void accelerometer_receive_callback(
        const libhcs::data::ImuAccelerometerDataView& data, const libhcs::time::SampleTime&)
        override {
        const double x = data.x, y = data.y, z = data.z;
        const std::scoped_lock lock{imu_mutex};
        ++accelerometer_samples;
        accelerometer_square_sum += x * x + y * y + z * z;
    }
    void gyroscope_receive_callback(const libhcs::data::ImuGyroscopeDataView&,
        const libhcs::time::SampleTime&) override {
        const std::scoped_lock lock{imu_mutex};
        ++gyroscope_samples;
    }
    [[nodiscard]] std::array<std::size_t, 2> imu_samples() {
        const std::scoped_lock lock{imu_mutex};
        return {accelerometer_samples, gyroscope_samples};
    }
    [[nodiscard]] double accelerometer_rms() {
        const std::scoped_lock lock{imu_mutex};
        return accelerometer_samples == 0
                 ? 0.0
                 : std::sqrt(accelerometer_square_sum / static_cast<double>(accelerometer_samples));
    }

    std::mutex imu_mutex;
    std::size_t accelerometer_samples = 0, gyroscope_samples = 0;
    double accelerometer_square_sum = 0.0;
};

struct CBoardNode {
    CBoardNode(const std::string& serial, const Configuration& configuration)
        : board(
              listener, serial,
              libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true),
              configuration) {}

    void send(DataId port, std::uint32_t can_id, std::span<const std::byte> data) {
        board.start_transmit().can_transmit(
            libhcs::spec::c_board::Spec::Can{port}, {.can_id = can_id, .can_data = data});
    }

    void send_uart(std::span<const std::byte> data) {
        board.start_transmit().uart_transmit(
            libhcs::spec::c_board::Spec::Uarts::kUart1, {.uart_data = data});
    }

    CBoardListener listener;
    CBoard board;
};

/// 不经板卡类的主机：握手照做、声明照发，但发送直接走协议层，绕过 SDK 对"没声明的口"的
/// 检查——等于一个写错了的、或者别人写的主机程序。用来验证板子自己的那一道：收到就丢。
class RawHost {
public:
    RawHost(
        std::uint16_t vendor_id, std::uint16_t product_id, const std::string& serial,
        const Configuration& configuration)
        : handler_(
              vendor_id, product_id, serial,
              libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true), ignore_,
              [configuration](libhcs::host::protocol::Handler& handler) {
                  (void)libhcs::board::hcs::apply(handler, configuration);
              }) {}

    /// 不做 EP0 握手的变体: 会话照开(kStart 照发), 只是清单位置空着。用来验证
    /// 板子的握手门 -- 门没开时 kStart 必须被静默拒绝。
    RawHost(std::uint16_t vendor_id, std::uint16_t product_id, const std::string& serial)
        : handler_(
              vendor_id, product_id, serial,
              libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true), ignore_,
              [](libhcs::host::protocol::Handler&) {}) {}

    /// @return 协议层收下了这一帧（与板子转不转发无关）
    bool send_can(DataId bus, std::uint32_t can_id, std::span<const std::byte> data) {
        return handler_.start_transmit().write_can(bus, {.can_id = can_id, .can_data = data});
    }
    bool send_uart(DataId port, std::span<const std::byte> data) {
        return handler_.start_transmit().write_uart(port, {.uart_data = data});
    }

    /// 一批发两条记录（同一次 USB 传输）：第一条发往板子必拒的口，第二条正常。
    /// 验证板子对拒收的记录只跳过那一条，后面的照常。
    void send_refused_then_can(
        DataId refused_bus, std::uint32_t refused_id, DataId bus, std::uint32_t can_id,
        std::span<const std::byte> data) {
        auto builder = handler_.start_transmit();
        (void)builder.write_can(refused_bus, {.can_id = refused_id, .can_data = data});
        (void)builder.write_can(bus, {.can_id = can_id, .can_data = data});
    }

    /// 板子随 keepalive 推送的链路状态（下行流错误，DataId::kSession）；还没收到 kStart 后的
    /// 基线时为空。
    [[nodiscard]] std::optional<libhcs::data::LinkStatusView> link_status() const {
        const auto any = handler_.port_status(DataId::kSession);
        if (const auto* link = std::get_if<libhcs::data::LinkStatusView>(&any))
            return *link;
        return std::nullopt;
    }

    /// @param bus 总线的 DataId（丝印号）
    [[nodiscard]] libhcs::data::CanStatusView can_status(DataId bus) const {
        return handler_.status<libhcs::data::CanStatusView>(bus);
    }
    [[nodiscard]] bool link_up() const noexcept {
        return handler_.link_state() == libhcs::host::protocol::Handler::LinkState::kUp;
    }

private:
    /// 上行一律收下不看：回 false 会被当成协议错误拆掉会话。
    struct Ignore final : libhcs::data::DataCallback {
        bool can_receive_callback(
            DataId, const libhcs::data::CanDataView&, const libhcs::time::SampleTime&) override {
            return true;
        }
        bool uart_receive_callback(
            DataId, const libhcs::data::UartDataView&, const libhcs::time::SampleTime&) override {
            return true;
        }
        bool gpio_digital_read_result_callback(std::uint8_t,
            const libhcs::data::GpioDigitalDataView&, const libhcs::time::SampleTime&) override {
            return true;
        }
        bool gpio_analog_read_result_callback(std::uint8_t,
            const libhcs::data::GpioAnalogDataView&, const libhcs::time::SampleTime&) override {
            return true;
        }
        void accelerometer_receive_callback(
            const libhcs::data::ImuAccelerometerDataView&, const libhcs::time::SampleTime&)
            override {}
        void gyroscope_receive_callback(
            const libhcs::data::ImuGyroscopeDataView&, const libhcs::time::SampleTime&) override {}
        void temperature_receive_callback(
            const libhcs::data::ImuTemperatureDataView&, const libhcs::time::SampleTime&)
            override {}
    };

    Ignore ignore_; // 先于 handler_ 构造、后于它析构
    libhcs::host::protocol::Handler handler_;
};

RawHost raw_hpm(const std::string& serial, const Configuration& configuration) {
    namespace identity = libhcs::core::protocol::usb_identity;
    return RawHost{
        identity::kDmtoolVendorId, identity::kHpm5321DualCanProductId, serial, configuration};
}

RawHost raw_mc02(const std::string& serial, const Configuration& configuration) {
    return RawHost{kMc02VendorId, kMc02ProductId, serial, configuration};
}

RawHost raw_cboard(const std::string& serial, const Configuration& configuration) {
    return RawHost{kCBoardVendorId, kCBoardProductId, serial, configuration};
}

RawHost raw_hpm_without_handshake(const std::string& serial) {
    namespace identity = libhcs::core::protocol::usb_identity;
    return RawHost{identity::kDmtoolVendorId, identity::kHpm5321DualCanProductId, serial};
}

RawHost raw_mc02_without_handshake(const std::string& serial) {
    return RawHost{kMc02VendorId, kMc02ProductId, serial};
}

/// 帧没到时附在失败信息后面：两端控制器的运行时状态（板子随 keepalive 推送），一眼分清是
/// 没人应答、线上出错还是板子自己丢了（读法见 libhcs/data/datas.hpp 的 CanStatusView）。
std::string bus_report(Node& hpm, Mc02Node& mc02, DataId port) {
    const auto line = [](const char* who, const libhcs::data::CanStatusView& status) {
        return std::format(
            "\n  {}: tec={} rec={} last_error={} data_last_error={} flags={:#04x} "
            "tx_cancelled={} tx_dropped={} rx_dropped={}",
            who, status.tec, status.rec, libhcs::data::last_error_name(status.last_error),
            libhcs::data::last_error_name(status.data_last_error), status.flags,
            status.tx_cancelled, status.tx_dropped, status.rx_dropped);
    };
    return line("5321", hpm.board.handle(libhcs::spec::hpm5321::Spec::Can{port}).status())
         + line("mc02", mc02.board.handle(libhcs::spec::mc02::Spec::Can{port}).status());
}

/// 端口状态随 keepalive 推送（250 ms 一轮）：等到 pred 成立，最多 1 s。返回最后一次读到的。
template <class Read, class Pred>
auto wait_status(Read read, Pred pred) {
    auto status = read();
    for (int i = 0; i < 100 && !pred(status); ++i) {
        std::this_thread::sleep_for(10ms);
        status = read();
    }
    return status;
}

std::vector<std::byte> pattern(std::size_t size, std::uint8_t seed) {
    std::vector<std::byte> bytes(size);
    for (std::size_t i = 0; i < size; ++i)
        bytes[i] = static_cast<std::byte>(seed + i * 13);
    return bytes;
}

class Bench : public testing::Test {
protected:
    void SetUp() override {
        const auto boards = bench();
        if (!boards)
            GTEST_SKIP() << "needs two dual-CAN HPM5321 boards (and no mc02) wired CAN1-CAN1, "
                            "CAN2-CAN2, UART0-UART0; found "
                         << attached_boards().size() << " HPM5321, " << attached_mc02().size()
                         << " mc02";
        serial_a_ = (*boards)[0];
        serial_b_ = (*boards)[1];
    }

    [[nodiscard]] std::unique_ptr<Node> open_a(const Configuration& configuration) const {
        return std::make_unique<Node>(serial_a_, configuration);
    }
    [[nodiscard]] std::unique_ptr<Node> open_b(const Configuration& configuration) const {
        return std::make_unique<Node>(serial_b_, configuration);
    }

    std::string serial_a_, serial_b_;
};

// ── 用例 ─────────────────────────────────────────────────────────────────

// 握手：板子报的版本与这份 SDK 一致（不一致时构造就抛了），通道数对得上，
// 声明成经典 CAN 的总线跑的就是经典 CAN。
TEST_F(Bench, HandshakeAgreesOnTheInterface) {
    const auto a = open_a(declare({DataId::kCan1, DataId::kCan2}, DataId::kUart0));
    // 声明过的总线跑的就是声明的帧型; 主机自己的记录里一个口一位。
    const auto interface = a->board.interface();
    EXPECT_TRUE(interface.declared(DataId::kCan1));
    EXPECT_TRUE(interface.declared(DataId::kCan2));
    EXPECT_TRUE(interface.declared(DataId::kUart0));
    EXPECT_FALSE(interface.can_fd(DataId::kCan1));
}

// 两边都声明了的总线：帧两个方向都到，内容不变。
TEST_F(Bench, ADeclaredBusCarriesFramesBothWays) {
    const auto a = open_a(declare({DataId::kCan1}));
    const auto b = open_b(declare({DataId::kCan1}));
    std::this_thread::sleep_for(kSettle);

    const auto payload = pattern(8, 0x11);
    a->send(DataId::kCan1, 0x321, payload);
    const auto at_b = b->listener.wait_frame(DataId::kCan1, 0x321, kArrival);
    ASSERT_TRUE(at_b.has_value()) << "A -> B on CAN1 never arrived";
    EXPECT_EQ(at_b->data, payload);

    const auto reply = pattern(8, 0x77);
    b->send(DataId::kCan1, 0x123, reply);
    const auto at_a = a->listener.wait_frame(DataId::kCan1, 0x123, kArrival);
    ASSERT_TRUE(at_a.has_value()) << "B -> A on CAN1 never arrived";
    EXPECT_EQ(at_a->data, reply);
}

// 这次固件改动的核心：B 没声明 CAN2，它的 CAN2 控制器就不在总线上——
// A 往 CAN2 发的帧 B 收不到，而且根本没人应答（A 的控制器记下 ACK 错误）。
// 同一时刻声明了的 CAN1 照常工作。
TEST_F(Bench, AnUndeclaredBusIsOffTheWire) {
    const auto a = open_a(declare({DataId::kCan1, DataId::kCan2}));
    const auto b = open_b(declare({DataId::kCan1}));
    std::this_thread::sleep_for(kSettle);

    const auto payload = pattern(8, 0x22);
    for (std::uint32_t i = 0; i < 4; ++i) {
        a->send(DataId::kCan2, 0x200 + i, payload);
        std::this_thread::sleep_for(5ms);
    }
    std::this_thread::sleep_for(kSilence);
    EXPECT_EQ(b->listener.frames_on(DataId::kCan2), 0U) << "frames reached an undeclared bus";

    // 总线上只有 A 一个节点在：帧发出去没有 ACK。发送错误计数涨了、最近的错误是 ACK，
    // 说明对面的控制器确实没有参与总线，而不只是主机把帧丢了。状态是 A 随 keepalive
    // 推上来的，没有 EP0 请求。
    const auto can2 = a->board.handle(libhcs::spec::hpm5321::Spec::Can{DataId::kCan2});
    const auto status = wait_status([&] { return can2.status(); }, [](const auto& s) {
        return s.tec > 0;
    });
    EXPECT_GT(status.tec, 0) << "somebody acknowledged frames on a bus B did not declare";
    EXPECT_EQ(status.last_error, libhcs::data::CanLastError::kAck);

    a->send(DataId::kCan1, 0x321, payload);
    EXPECT_TRUE(b->listener.wait_frame(DataId::kCan1, 0x321, kArrival).has_value())
        << "the declared bus stopped working";
}

// 往没声明的口发，第一道：板卡类当场抛
// UndeclaredChannel，这一帧根本没进发送缓冲。
// 抛完之后会话照旧，声明了的口照常工作。
TEST_F(Bench, TransmittingOnAnUndeclaredChannelThrows) {
    const auto a = open_a(declare({DataId::kCan1, DataId::kCan2}, DataId::kUart0));
    const auto b = open_b(declare({DataId::kCan1}));
    std::this_thread::sleep_for(kSettle);

    const auto payload = pattern(8, 0x33);
    EXPECT_THROW(b->send(DataId::kCan2, 0x2AA, payload), UndeclaredChannel);
    EXPECT_THROW(b->send_uart(pattern(32, 0x44)), UndeclaredChannel);
    std::this_thread::sleep_for(kSilence);
    EXPECT_EQ(a->listener.frames_on(DataId::kCan2), 0U);
    EXPECT_TRUE(a->listener.wait_bytes(1, 0ms).empty());

    EXPECT_EQ(b->board.link_state(), libhcs::host::protocol::Handler::LinkState::kUp);
    b->send(DataId::kCan1, 0x123, payload);
    EXPECT_TRUE(a->listener.wait_frame(DataId::kCan1, 0x123, kArrival).has_value())
        << "the declared bus stopped working";
}

// 第二道：主机不做检查、照样把帧发下去，板子收到就丢，线上什么都没有。
TEST_F(Bench, TheBoardDropsWhatAnUncheckedHostSendsToAnUndeclaredBus) {
    const auto a = open_a(declare({DataId::kCan1, DataId::kCan2}));
    auto b = raw_hpm(serial_b_, declare({DataId::kCan1}));
    std::this_thread::sleep_for(kSettle);

    const auto payload = pattern(8, 0x33);
    ASSERT_TRUE(b.send_can(DataId::kCan2, 0x2AA, payload));
    std::this_thread::sleep_for(kSilence);
    EXPECT_EQ(a->listener.frames_on(DataId::kCan2), 0U);

    EXPECT_TRUE(b.link_up());
    ASSERT_TRUE(b.send_can(DataId::kCan1, 0x123, payload));
    EXPECT_TRUE(a->listener.wait_frame(DataId::kCan1, 0x123, kArrival).has_value())
        << "the declared bus stopped working";
}

// 换一次握手就是换一组声明：上一次声明过的口不会留到这一次。
TEST_F(Bench, ANewHandshakeReplacesTheOldDeclaration) {
    const auto a = open_a(declare({DataId::kCan1, DataId::kCan2}));
    const auto payload = pattern(8, 0x44);
    {
        const auto b = open_b(declare({DataId::kCan1, DataId::kCan2}));
        std::this_thread::sleep_for(kSettle);
        a->send(DataId::kCan2, 0x250, payload);
        ASSERT_TRUE(b->listener.wait_frame(DataId::kCan2, 0x250, kArrival).has_value())
            << "CAN2 does not work even when both ends declare it: check the bench "
               "wiring";
    }

    const auto b = open_b(declare({DataId::kCan1}));
    std::this_thread::sleep_for(kSettle);
    a->send(DataId::kCan2, 0x251, payload);
    std::this_thread::sleep_for(kSilence);
    EXPECT_EQ(b->listener.frames_on(DataId::kCan2), 0U)
        << "CAN2 stayed up from the previous handshake";
}

// 串口同理：没声明就不转发；声明了，字节按序、一个不少地到。
TEST_F(Bench, AnUndeclaredUartForwardsNothingAndADeclaredOneForwardsEverything) {
    const auto a = open_a(declare({}, DataId::kUart0));
    const auto bytes = pattern(96, 0x55);
    {
        const auto b = open_b(declare({DataId::kCan1}));
        std::this_thread::sleep_for(kSettle);
        a->send_uart(bytes);
        EXPECT_TRUE(b->listener.wait_bytes(1, kSilence).empty())
            << "bytes were forwarded from a UART the host did not declare";
    }

    const auto b = open_b(declare({}, DataId::kUart0));
    std::this_thread::sleep_for(kSettle);
    a->send_uart(bytes);
    EXPECT_EQ(b->listener.wait_bytes(bytes.size(), kArrival), bytes);
}

// 运行期改任意一个口，主机都会把整份声明重放一遍。设置没变的串口不能因此被重写：
// 重写分频器要先中止发送 DMA，正在线上的字节就丢了。两边各在串口满载时重放几次，
// 字节一个不少、按序到。
TEST_F(Bench, ReplayingTheDeclarationDoesNotInterruptAStreamingUart) {
    const auto a = open_a(declare({DataId::kCan1}, DataId::kUart0));
    const auto b = open_b(declare({DataId::kCan1}, DataId::kUart0));
    std::this_thread::sleep_for(kSettle);

    // 每段 256 字节在 1 Mbit/s 下约 2.6 ms：重放到达时发送 DMA 一定还在跑。
    constexpr std::size_t kChunk = 256;
    constexpr std::size_t kChunks = 8;
    std::vector<std::byte> sent;
    for (std::size_t i = 0; i < kChunks; ++i) {
        const auto chunk = pattern(kChunk, static_cast<std::uint8_t>(i));
        a->send_uart(chunk);
        sent.insert(sent.end(), chunk.begin(), chunk.end());
        auto& replaying = (i % 2 == 0) ? *a : *b; // 发送端与接收端轮流重放
        replaying.board.handle(libhcs::spec::hpm5321::Spec::Cans::kCan1).configure(kClassic1M);
    }
    EXPECT_EQ(b->listener.wait_bytes(sent.size(), kArrival), sent)
        << "a replayed declaration rewrote the streaming UART";
}

// ── 一块 5321 对一块 mc02 ────────────────────────────────────────────────
//
// mc02 上"没声明"比 5321 更彻底：通道从上电起就是停的（FDCAN 留在 INIT，串口不武装接收），
// 所以这里问的是同一组问题，答案由另一份固件给出。总线按实车的帧型跑：CAN-FD 1M/5M。

class Mc02Bench : public testing::Test {
protected:
    void SetUp() override {
        const auto boards = mc02_bench();
        if (!boards)
            GTEST_SKIP() << "needs exactly one dual-CAN HPM5321 and one mc02 wired CAN1-CAN1, "
                            "CAN2-CAN2, UART0-UART1; found "
                         << attached_boards().size() << " HPM5321, " << attached_mc02().size()
                         << " mc02";
        serial_hpm_ = (*boards)[0];
        serial_mc02_ = (*boards)[1];
    }

    [[nodiscard]] std::unique_ptr<Node> open_hpm(const Configuration& configuration) const {
        return std::make_unique<Node>(serial_hpm_, configuration);
    }
    [[nodiscard]] std::unique_ptr<Mc02Node> open_mc02(const Configuration& configuration) const {
        return std::make_unique<Mc02Node>(serial_mc02_, configuration);
    }

    /// 两条总线都声明、都跑 FD 的 5321：对面的 mc02 声明什么，由各用例自己定。
    [[nodiscard]] std::unique_ptr<Node> open_hpm_both() const {
        return open_hpm(declare({DataId::kCan1, DataId::kCan2}, std::nullopt, kFd1M5M));
    }

    std::string serial_hpm_, serial_mc02_;
};

// 握手：版本一致，通道数是这块板的三路 CAN、六个串口；声明成 FD 的总线报的就是 FD。
TEST_F(Mc02Bench, HandshakeAgreesOnTheInterface) {
    const auto mc02 = open_mc02(declare({DataId::kCan1}, kMc02Uart1, kFd1M5M));
    const auto interface = mc02->board.interface();
    EXPECT_TRUE(interface.declared(DataId::kCan1));
    EXPECT_FALSE(interface.declared(DataId::kCan2)) << "the wiring did not declare it";
    EXPECT_TRUE(interface.can_fd(DataId::kCan1));
    EXPECT_EQ(mc02->board.link_state(), libhcs::host::protocol::Handler::LinkState::kUp);
}

// 两边都声明了的两条总线：帧两个方向都到，内容不变。
TEST_F(Mc02Bench, DeclaredBusesCarryFramesBothWays) {
    const auto hpm = open_hpm_both();
    const auto mc02 = open_mc02(declare({DataId::kCan1, DataId::kCan2}, std::nullopt, kFd1M5M));
    std::this_thread::sleep_for(kSettle);

    for (const DataId port : {DataId::kCan1, DataId::kCan2}) {
        const auto bus = static_cast<unsigned>(port);
        const auto payload = pattern(8, static_cast<std::uint8_t>(0x11 * bus));
        hpm->send(port, 0x320 + bus, payload);
        const auto at_mc02 = mc02->listener.wait_frame(port, 0x320 + bus, kArrival);
        ASSERT_TRUE(at_mc02.has_value())
            << "5321 -> mc02 on CAN" << bus << " never arrived" << bus_report(*hpm, *mc02, port);
        EXPECT_EQ(at_mc02->data, payload);

        const auto reply = pattern(8, static_cast<std::uint8_t>(0x70 + bus));
        mc02->send(port, 0x120 + bus, reply);
        const auto at_hpm = hpm->listener.wait_frame(port, 0x120 + bus, kArrival);
        ASSERT_TRUE(at_hpm.has_value())
            << "mc02 -> 5321 on CAN" << bus << " never arrived" << bus_report(*hpm, *mc02, port);
        EXPECT_EQ(at_hpm->data, reply);
    }
}

// mc02 没声明 CAN2：它的 FDCAN2 不在总线上——5321 往 CAN2
// 发的帧它收不到，也没人应答。同一时刻声明了的 CAN1 照常工作。
TEST_F(Mc02Bench, AnUndeclaredBusIsOffTheWire) {
    const auto hpm = open_hpm_both();
    const auto mc02 = open_mc02(declare({DataId::kCan1}, std::nullopt, kFd1M5M));
    std::this_thread::sleep_for(kSettle);

    const auto payload = pattern(8, 0x22);
    for (std::uint32_t i = 0; i < 4; ++i) {
        hpm->send(DataId::kCan2, 0x200 + i, payload);
        std::this_thread::sleep_for(5ms);
    }
    std::this_thread::sleep_for(kSilence);
    EXPECT_EQ(mc02->listener.frames_on(DataId::kCan2), 0U) << "frames reached an undeclared bus";

    const auto can2 = hpm->board.handle(libhcs::spec::hpm5321::Spec::Can{DataId::kCan2});
    const auto status = wait_status([&] { return can2.status(); }, [](const auto& s) {
        return s.tec > 0;
    });
    EXPECT_GT(status.tec, 0) << "somebody acknowledged frames on a bus the mc02 did not declare";
    // FD 且 BRS 的帧, M_CAN 把 ACK 错误记在 DLEC(data_last_error), 不在 LEC:
    // hpm 与 mc02 实测都是这样(2026-10-06)。
    EXPECT_EQ(status.data_last_error, libhcs::data::CanLastError::kAck);

    hpm->send(DataId::kCan1, 0x321, payload);
    EXPECT_TRUE(mc02->listener.wait_frame(DataId::kCan1, 0x321, kArrival).has_value())
        << "the declared bus stopped working";
}

// 往没声明的口发，第一道：板卡类当场抛
// UndeclaredChannel——CAN、串口、按描述符发都一样，
// 两种板型都一样。这一帧没进发送缓冲，线上什么都没有；会话照旧，声明了的口照常工作。
TEST_F(Mc02Bench, TransmittingOnAnUndeclaredChannelThrows) {
    const auto hpm = open_hpm(declare({DataId::kCan1}, std::nullopt, kFd1M5M));
    const auto mc02 = open_mc02(declare({DataId::kCan1}, std::nullopt, kFd1M5M));
    std::this_thread::sleep_for(kSettle);

    const auto payload = pattern(8, 0x33);
    const auto bytes = pattern(32, 0x44);
    EXPECT_THROW(mc02->send(DataId::kCan2, 0x2AA, payload), UndeclaredChannel);
    EXPECT_THROW(mc02->send(DataId::kCan3, 0x2AB, payload), UndeclaredChannel);
    EXPECT_THROW(mc02->send_uart(bytes), UndeclaredChannel);
    EXPECT_THROW(
        mc02->board.start_transmit().uart_transmit(
            libhcs::spec::mc02::Spec::Uarts::kUart7, {.uart_data = bytes}),
        UndeclaredChannel);
    EXPECT_THROW(hpm->send(DataId::kCan2, 0x2AC, payload), UndeclaredChannel);
    EXPECT_THROW(hpm->send_uart(bytes), UndeclaredChannel);

    // 一个包里先写了声明过的口、再碰到没声明的：抛出之前写进去的那一帧照常发出。
    EXPECT_THROW(
        mc02->board.start_transmit()
            .can_transmit(
                libhcs::spec::mc02::Spec::Cans::kCan1, {.can_id = 0x124, .can_data = payload})
            .can_transmit(
                libhcs::spec::mc02::Spec::Cans::kCan2, {.can_id = 0x2AD, .can_data = payload}),
        UndeclaredChannel);
    EXPECT_TRUE(hpm->listener.wait_frame(DataId::kCan1, 0x124, kArrival).has_value())
        << "the frame written before the refused one was lost";

    EXPECT_EQ(mc02->board.link_state(), libhcs::host::protocol::Handler::LinkState::kUp);
    EXPECT_EQ(hpm->board.link_state(), libhcs::host::protocol::Handler::LinkState::kUp);
    mc02->send(DataId::kCan1, 0x123, payload);
    EXPECT_TRUE(hpm->listener.wait_frame(DataId::kCan1, 0x123, kArrival).has_value())
        << "the declared bus stopped working" << bus_report(*hpm, *mc02, DataId::kCan1);
}

// 第二道，mc02 的固件：主机不做检查、照样把数据发下去，板子收到就丢——没有把帧交给控制器，
// 线上什么都没有。没声明的口不在总线上，板子也不推它的状态。
TEST_F(Mc02Bench, TheMc02DropsWhatAnUncheckedHostSendsToUndeclaredChannels) {
    const auto hpm = open_hpm(declare({DataId::kCan1, DataId::kCan2}, DataId::kUart0, kFd1M5M));
    auto mc02 = raw_mc02(serial_mc02_, declare({DataId::kCan1}, std::nullopt, kFd1M5M));
    std::this_thread::sleep_for(kSettle);

    const auto payload = pattern(8, 0x33);
    ASSERT_TRUE(mc02.send_can(DataId::kCan2, 0x2AA, payload));      // 没声明的 CAN
    ASSERT_TRUE(mc02.send_can(DataId::kCan3, 0x2AB, payload));      // 没声明、也没接线的 CAN
    ASSERT_TRUE(mc02.send_uart(DataId::kUart1, pattern(32, 0x44))); // 没声明的串口
    std::this_thread::sleep_for(kSilence);

    EXPECT_EQ(hpm->listener.frames_on(DataId::kCan2), 0U);
    EXPECT_TRUE(hpm->listener.wait_bytes(1, 0ms).empty());
    EXPECT_EQ(mc02.can_status(DataId::kCan2), libhcs::data::CanStatusView{})
        << "the board reported status for a port it was not given";

    EXPECT_TRUE(mc02.link_up());
    ASSERT_TRUE(mc02.send_can(DataId::kCan1, 0x123, payload));
    EXPECT_TRUE(hpm->listener.wait_frame(DataId::kCan1, 0x123, kArrival).has_value())
        << "the declared bus stopped working";
}

// 第二道，5321 的固件：同一件事，方向反过来。
TEST_F(Mc02Bench, The5321DropsWhatAnUncheckedHostSendsToUndeclaredChannels) {
    auto hpm = raw_hpm(serial_hpm_, declare({DataId::kCan1}, std::nullopt, kFd1M5M));
    const auto mc02 = open_mc02(declare({DataId::kCan1, DataId::kCan2}, kMc02Uart1, kFd1M5M));
    std::this_thread::sleep_for(kSettle);

    const auto payload = pattern(8, 0x33);
    ASSERT_TRUE(hpm.send_can(DataId::kCan2, 0x2AA, payload));      // 没声明的 CAN
    ASSERT_TRUE(hpm.send_uart(DataId::kUart0, pattern(32, 0x44))); // 没声明的串口
    std::this_thread::sleep_for(kSilence);
    EXPECT_EQ(mc02->listener.frames_on(DataId::kCan2), 0U);
    EXPECT_TRUE(mc02->listener.wait_bytes(1, 0ms).empty());

    EXPECT_TRUE(hpm.link_up());
    ASSERT_TRUE(hpm.send_can(DataId::kCan1, 0x123, payload));
    EXPECT_TRUE(mc02->listener.wait_frame(DataId::kCan1, 0x123, kArrival).has_value())
        << "the declared bus stopped working";
}

// 声明一个板子没有的口：握手时就抛，说清是哪一路。
// GPIO 口是一路口, 线逐根声明。台架上 PWM 排针空着: 带上拉的输入线读出来就是高电平 --
// 下行的"读一次"(kGpioOther)、上行的电平(kGpio)与带时间戳的读数(kGpioOther)都在真固件
// 上走一遍。只声明输入: 不往空着的排针上驱动任何东西。没声明的线读回 Off, 板上没有的线
// 在构造时就被拒。
TEST_F(Mc02Bench, AGpioLineRunsAsDeclaredAndAnswersItsReads) {
    using Pwm = libhcs::spec::mc02::Spec::Gpios;
    const libhcs::board::hcs::GpioSetting pulled_up{
        .mode = vc::kGpioModeInput, .pull = vc::kGpioPullUp};
    const libhcs::board::hcs::GpioSetting pulled_up_stamped{
        .mode = vc::kGpioModeInput,
        .pull = vc::kGpioPullUp,
        .input_flags = vc::kGpioInputTimestamp};
    Configuration configuration = declare({DataId::kCan1}, std::nullopt, kFd1M5M);
    configuration.declare_gpio(Pwm::kPwm2, pulled_up);
    configuration.declare_gpio(Pwm::kPwm3, pulled_up_stamped);
    const auto mc02 = open_mc02(configuration);

    EXPECT_EQ(mc02->board.handle(Pwm::kPwm2).setting(), pulled_up);
    EXPECT_EQ(mc02->board.handle(Pwm::kPwm3).setting(), pulled_up_stamped);
    EXPECT_EQ(mc02->board.handle(Pwm::kPwm1).setting().mode, vc::kGpioModeOff);
    EXPECT_TRUE(mc02->board.interface().line_declared(Pwm::kPwm2.line));
    EXPECT_FALSE(mc02->board.interface().output(Pwm::kPwm2.line));

    mc02->board.start_transmit().gpio_read(Pwm::kPwm2).gpio_read(Pwm::kPwm3);
    for (int i = 0; i < 200 && mc02->listener.gpio().size() < 2; ++i)
        std::this_thread::sleep_for(1ms);
    const auto samples = mc02->listener.gpio();
    ASSERT_EQ(samples.size(), 2U) << "each read-once request gets exactly one sample";
    EXPECT_EQ(samples[0].line, Pwm::kPwm2.line);
    EXPECT_TRUE(samples[0].high) << "a pulled-up input reads low: is something attached to PWM2?";
    EXPECT_FALSE(samples[0].timestamp.has_value());
    EXPECT_EQ(samples[1].line, Pwm::kPwm3.line);
    EXPECT_TRUE(samples[1].high) << "a pulled-up input reads low: is something attached to PWM3?";
    EXPECT_TRUE(samples[1].timestamp.has_value()) << "the line was declared with timestamps";

    Configuration fifth_line = declare({DataId::kCan1}, std::nullopt, kFd1M5M);
    fifth_line.declare_gpio(4, pulled_up);
    EXPECT_THROW((void)open_mc02(fifth_line), std::runtime_error) << "mc02 has four lines";
}

// 蜂鸣器是一路口: 不声明时通道不开、读回静音、往它发音即抛; 声明后从静音开始, 按记录放音,
// 读回的是定时器寄存器里的音; 静音记录让它停。用 20 kHz: 无源蜂鸣器在这里几乎不出声,
// 测试不该在台架上响。
TEST_F(Mc02Bench, TheBuzzerPlaysOnlyWhenDeclared) {
    const auto wait_for = [](auto&& condition) {
        for (int i = 0; i < 100 && !condition(); ++i)
            std::this_thread::sleep_for(1ms);
        return condition();
    };
    {
        const auto mc02 = open_mc02(declare({DataId::kCan1}, std::nullopt, kFd1M5M));
        EXPECT_EQ(mc02->board.buzzer_state().frequency_hz, 0U);
        EXPECT_THROW(
            mc02->board.start_transmit().buzzer_tone({.frequency_hz = 20'000, .loudness = 255}),
            UndeclaredChannel);
    }

    Configuration configuration = declare({DataId::kCan1}, std::nullopt, kFd1M5M);
    configuration.declare_buzzer();
    const auto mc02 = open_mc02(configuration);
    EXPECT_EQ(mc02->board.buzzer_state().frequency_hz, 0U) << "a declared buzzer starts silent";

    mc02->board.start_transmit().buzzer_tone({.frequency_hz = 20'000, .loudness = 255});
    EXPECT_TRUE(wait_for([&] { return mc02->board.buzzer_state().frequency_hz == 20'000U; }));
    EXPECT_EQ(mc02->board.buzzer_state().loudness, 255U);

    mc02->board.start_transmit().buzzer_tone({.frequency_hz = 0, .loudness = 0});
    EXPECT_TRUE(wait_for([&] { return mc02->board.buzzer_state().frequency_hz == 0U; }));
}

TEST_F(Mc02Bench, DeclaringAChannelTheBoardLacksThrowsAtConstruction) {
    Configuration no_such_bus;
    no_such_bus.declare_can(DataId::kCan0, kFd1M5M); // mc02 只有 CAN1..CAN3
    EXPECT_THROW((void)open_mc02(no_such_bus), std::runtime_error);

    Configuration no_such_port;
    no_such_port.declare_uart(DataId::kUart0, uart_8n1(115'200)); // 不在 mc02 的清单里
    EXPECT_THROW((void)open_mc02(no_such_port), std::runtime_error);
}

// 运行时补声明：configure_canN() 对一条构造时没声明的总线，就是声明并启动它——
// 板子把它拉上总线，主机这边发送入口也随之放行。
TEST_F(Mc02Bench, ConfiguringABusAtRunTimeDeclaresIt) {
    const auto hpm = open_hpm_both();
    const auto mc02 = open_mc02(declare({DataId::kCan1}, std::nullopt, kFd1M5M));
    std::this_thread::sleep_for(kSettle);

    const auto payload = pattern(8, 0x66);
    hpm->send(DataId::kCan2, 0x260, payload);
    std::this_thread::sleep_for(kSilence);
    ASSERT_EQ(mc02->listener.frames_on(DataId::kCan2), 0U);
    ASSERT_THROW(mc02->send(DataId::kCan2, 0x160, payload), UndeclaredChannel);

    mc02->board.handle(libhcs::spec::mc02::Spec::Cans::kCan2).configure(kFd1M5M);
    std::this_thread::sleep_for(kSettle);
    hpm->send(DataId::kCan2, 0x261, payload);
    EXPECT_TRUE(mc02->listener.wait_frame(DataId::kCan2, 0x261, kArrival).has_value())
        << "CAN2 did not start after configure_can2()" << bus_report(*hpm, *mc02, DataId::kCan2);

    EXPECT_NO_THROW(mc02->send(DataId::kCan2, 0x161, payload));
    EXPECT_TRUE(hpm->listener.wait_frame(DataId::kCan2, 0x161, kArrival).has_value())
        << "mc02 -> 5321 on the bus declared at run time" << bus_report(*hpm, *mc02, DataId::kCan2);
}

// 换一次握手就是换一组声明：上一次声明过的口不会留到这一次。
TEST_F(Mc02Bench, ANewHandshakeReplacesTheOldDeclaration) {
    const auto hpm = open_hpm_both();
    const auto payload = pattern(8, 0x44);
    {
        const auto mc02 = open_mc02(declare({DataId::kCan1, DataId::kCan2}, std::nullopt, kFd1M5M));
        std::this_thread::sleep_for(kSettle);
        hpm->send(DataId::kCan2, 0x250, payload);
        ASSERT_TRUE(mc02->listener.wait_frame(DataId::kCan2, 0x250, kArrival).has_value())
            << "CAN2 does not work even when both ends declare it: check the bench "
               "wiring"
            << bus_report(*hpm, *mc02, DataId::kCan2);
    }

    const auto mc02 = open_mc02(declare({DataId::kCan1}, std::nullopt, kFd1M5M));
    std::this_thread::sleep_for(kSettle);
    hpm->send(DataId::kCan2, 0x251, payload);
    std::this_thread::sleep_for(kSilence);
    EXPECT_EQ(mc02->listener.frames_on(DataId::kCan2), 0U)
        << "CAN2 stayed up from the previous handshake";
}

// 板载 IMU 也是一路要声明的通道：没声明，mc02 根本不初始化
// BMI088，一个样本都没有；
// 声明了，样本按声明的速率来、按声明的量程来（板子静止时加速度的模长就是 1
// g，量程翻倍原始值减半）；下一次握手不声明，它又停了。
TEST_F(Mc02Bench, TheOnboardImuStreamsOnlyWhenDeclaredAndAsDeclared) {
    constexpr auto kWindow = 500ms;
    const auto observe = [&](const Configuration& configuration) {
        const auto mc02 = open_mc02(configuration);
        std::this_thread::sleep_for(kWindow);
        const auto samples = mc02->listener.imu_samples();
        return std::tuple{samples[0], samples[1], mc02->listener.accelerometer_rms()};
    };

    Configuration without = declare({DataId::kCan1}, std::nullopt, kFd1M5M);
    {
        const auto [accelerometer, gyroscope, rms] = observe(without);
        EXPECT_EQ(accelerometer, 0U) << "accelerometer samples from an IMU nobody declared";
        EXPECT_EQ(gyroscope, 0U) << "gyroscope samples from an IMU nobody declared";
    }

    Configuration six_g = without;
    six_g.declare_imu(
        ImuSetting{
            .accelerometer_range_g = 6,
            .accelerometer_rate_hz = 1600,
            .gyroscope_range_dps = 2000,
            .gyroscope_rate_hz = 2000,
            .gyroscope_bandwidth_hz = 230});
    const auto [accelerometer_6g, gyroscope_6g, rms_6g] = observe(six_g);
    // 握手里初始化传感器要几十毫秒，窗口里实际采样的时间略短于 500 ms：给两成余量。
    EXPECT_NEAR(static_cast<double>(accelerometer_6g), 1600 * 0.5, 1600 * 0.5 * 0.2);
    EXPECT_NEAR(static_cast<double>(gyroscope_6g), 2000 * 0.5, 2000 * 0.5 * 0.2);
    // 静止的板子：1 g 在 ±6 g 量程下是 32767 / 6 ≈ 5461。
    EXPECT_NEAR(rms_6g, 32767.0 / 6.0, 32767.0 / 6.0 * 0.15) << "is the board lying still?";

    Configuration twelve_g_slow = without;
    twelve_g_slow.declare_imu(
        ImuSetting{
            .accelerometer_range_g = 12,
            .accelerometer_rate_hz = 400,
            .gyroscope_range_dps = 2000,
            .gyroscope_rate_hz = 1000,
            .gyroscope_bandwidth_hz = 116});
    const auto [accelerometer_12g, gyroscope_12g, rms_12g] = observe(twelve_g_slow);
    EXPECT_NEAR(static_cast<double>(accelerometer_12g), 400 * 0.5, 400 * 0.5 * 0.2);
    EXPECT_NEAR(static_cast<double>(gyroscope_12g), 1000 * 0.5, 1000 * 0.5 * 0.2);
    EXPECT_NEAR(rms_12g, 32767.0 / 12.0, 32767.0 / 12.0 * 0.15);

    {
        const auto [accelerometer, gyroscope, rms] = observe(without);
        EXPECT_EQ(accelerometer, 0U) << "the IMU stayed up from the previous handshake";
        EXPECT_EQ(gyroscope, 0U) << "the IMU stayed up from the previous handshake";
    }
}

// 运行期改任意一个口，主机都会把整份声明重放一遍。设置没变的东西不能因此被重来：串口不
// 重写分频器（会停口），IMU 不重新初始化（在 EP0 处理器里阻塞约 40 ms，这期间 1 Mbit/s
// 的串口要进约 4 KB，接收环只有 2 KB）。5321 一边持续往 mc02 的 UART1 发，mc02 一边
// 重放几次声明；字节一个不少、按序到。
TEST_F(Mc02Bench, ReplayingTheDeclarationDoesNotInterruptAStreamingUart) {
    const auto hpm = open_hpm(declare({}, DataId::kUart0));
    Configuration with_imu = declare({}, kMc02Uart1);
    with_imu.declare_imu(
        ImuSetting{
            .accelerometer_range_g = 6,
            .accelerometer_rate_hz = 1600,
            .gyroscope_range_dps = 2000,
            .gyroscope_rate_hz = 2000,
            .gyroscope_bandwidth_hz = 230});
    const auto mc02 = open_mc02(with_imu);
    std::this_thread::sleep_for(kSettle);

    // 32 段各 256 字节，每段在 1 Mbit/s 下约 2.6 ms：整个流约 85 ms，线上始终有字节。
    constexpr std::size_t kChunk = 256;
    constexpr std::size_t kChunks = 32;
    std::vector<std::byte> sent;
    for (std::size_t i = 0; i < kChunks; ++i) {
        const auto chunk = pattern(kChunk, static_cast<std::uint8_t>(i));
        sent.insert(sent.end(), chunk.begin(), chunk.end());
    }
    std::thread stream([&] {
        for (std::size_t i = 0; i < kChunks; ++i) {
            hpm->send_uart(std::span{sent}.subspan(i * kChunk, kChunk));
            std::this_thread::sleep_for(2500us);
        }
    });
    for (int i = 0; i < 4; ++i) {
        std::this_thread::sleep_for(15ms);
        mc02->board.handle(libhcs::spec::mc02::Spec::Uarts::kUart1).configure(uart_8n1(kUartBaudrate));
    }
    stream.join();
    EXPECT_EQ(mc02->listener.wait_bytes(sent.size(), kArrival), sent)
        << "a replayed declaration interrupted the mc02's UART1";
}

// 主机断开超过会话租约(4 s)后板子会停掉全部口。再来的主机声明同样的 IMU 设置, IMU 必须
// 重新启动: "设置没变就不重启"只对正在跑的 IMU 成立(这条曾经不成立 -- 会话结束停了采样
// 却没忘掉"在跑", 重启 HCS 之后 IMU 一个样本都没有)。
TEST_F(Mc02Bench, TheImuComesBackForAHostThatReconnectsAfterTheLease) {
    Configuration with_imu = declare({DataId::kCan1}, std::nullopt, kFd1M5M);
    with_imu.declare_imu(
        ImuSetting{
            .accelerometer_range_g = 6,
            .accelerometer_rate_hz = 1600,
            .gyroscope_range_dps = 2000,
            .gyroscope_rate_hz = 2000,
            .gyroscope_bandwidth_hz = 230});
    const auto samples = [&] {
        const auto mc02 = open_mc02(with_imu);
        std::this_thread::sleep_for(300ms);
        return mc02->listener.imu_samples()[0];
    };
    ASSERT_GT(samples(), 0U);
    std::this_thread::sleep_for(5s); // 租约到期, 板子停掉全部口
    EXPECT_GT(samples(), 0U) << "the IMU stayed stopped for a host that declared it again";
}

// 租约到期后, 一个不做 EP0 握手的主机发 kStart 必须被静默拒绝。门随会话一起忘掉:
// 不拔线换主机程序不会重新枚举(tud_mount_cb 不触发), 门若不被清, 从不握手的新主机
// 就继承了上一台的通行权 [实测于 hpm_board]。mc02 与 hpm 的会话结束路径一直清门;
// 这条用例把"租约到期也算会话结束"钉住 -- 三块板共用 core 的会话状态机, 到期走
// 同一条 deactivate 路径。
TEST_F(Mc02Bench, AHostWithoutTheEp0HandshakeCannotOpenASessionAfterTheLease) {
    auto mc02 = open_mc02(declare({DataId::kCan1}, std::nullopt, kFd1M5M));
    auto hpm = open_hpm(declare({DataId::kCan1}, std::nullopt, kFd1M5M));
    ASSERT_TRUE(mc02->board.link_state() == libhcs::host::protocol::Handler::LinkState::kUp);
    ASSERT_TRUE(hpm->board.link_state() == libhcs::host::protocol::Handler::LinkState::kUp);
    mc02.reset();
    hpm.reset();
    std::this_thread::sleep_for(5s); // 租约到期, 板子停口并忘掉握手

    // 开不了会话的主机不会把链路带到 kUp: 板子静默拒绝, 主机侧在 SESSION_ACK
    // 超时处抛出(固件注释里的"约一个轮次后自行触发 ack 超时"就是这一步)。
    EXPECT_THROW((void)raw_mc02_without_handshake(serial_mc02_), std::runtime_error);
    EXPECT_THROW((void)raw_hpm_without_handshake(serial_hpm_), std::runtime_error);
}

// 声明一个板子给不了的 IMU：5321 没有 IMU，mc02 的 BMI088 没有 5 g
// 这一档——握手时就抛，板子随后照常可用。
TEST_F(Mc02Bench, DeclaringAnImuTheBoardCannotRunThrowsAtConstruction) {
    Configuration no_imu_here = declare({DataId::kCan1}, std::nullopt, kFd1M5M);
    no_imu_here.declare_imu(
        ImuSetting{
            .accelerometer_range_g = 6,
            .accelerometer_rate_hz = 1600,
            .gyroscope_range_dps = 2000,
            .gyroscope_rate_hz = 2000,
            .gyroscope_bandwidth_hz = 230});
    EXPECT_THROW((void)open_hpm(no_imu_here), std::runtime_error);

    Configuration no_such_range = declare({DataId::kCan1}, std::nullopt, kFd1M5M);
    no_such_range.declare_imu(
        ImuSetting{
            .accelerometer_range_g = 5,
            .accelerometer_rate_hz = 1600,
            .gyroscope_range_dps = 2000,
            .gyroscope_rate_hz = 2000,
            .gyroscope_bandwidth_hz = 230});
    EXPECT_THROW((void)open_mc02(no_such_range), std::runtime_error);

    const auto hpm = open_hpm(declare({DataId::kCan1}, std::nullopt, kFd1M5M));
    const auto mc02 = open_mc02(declare({DataId::kCan1}, std::nullopt, kFd1M5M));
    std::this_thread::sleep_for(kSettle);
    const auto payload = pattern(8, 0x77);
    hpm->send(DataId::kCan1, 0x321, payload);
    EXPECT_TRUE(mc02->listener.wait_frame(DataId::kCan1, 0x321, kArrival).has_value());
    EXPECT_EQ(mc02->listener.imu_samples()[1], 0U);
}

// 串口：mc02 没声明 UART1
// 时什么都不转发；声明了，两个方向的字节都按序、一个不少地到。
TEST_F(Mc02Bench, AnUndeclaredUartForwardsNothingAndADeclaredOneForwardsBothWays) {
    const auto hpm = open_hpm(declare({}, DataId::kUart0));
    const auto bytes = pattern(96, 0x55);
    {
        const auto mc02 = open_mc02(declare({DataId::kCan1}, std::nullopt, kFd1M5M));
        std::this_thread::sleep_for(kSettle);
        hpm->send_uart(bytes);
        EXPECT_TRUE(mc02->listener.wait_bytes(1, kSilence).empty())
            << "bytes were forwarded from a UART the host did not declare";
    }

    const auto mc02 = open_mc02(declare({}, kMc02Uart1));
    std::this_thread::sleep_for(kSettle);
    hpm->send_uart(bytes);
    EXPECT_EQ(mc02->listener.wait_bytes(bytes.size(), kArrival), bytes) << "5321 -> mc02";

    const auto reply = pattern(96, 0xA5);
    mc02->send_uart(reply);
    EXPECT_EQ(hpm->listener.wait_bytes(reply.size(), kArrival), reply) << "mc02 -> 5321";
}

// 串口的速率与帧格式也是声明的内容，运行时还能改。两边改成同一组，字节照样双向到齐；
// 只改 mc02 一边，5321
// 发过去的字节就对不上。后一半证明设置真的落到了线上：两块板要是
// 一致地没理会请求，前一半照样会过（这块板的 BRR
// 曾经一次都没写过，自环测试全绿）。
TEST_F(Mc02Bench, TheUartRunsAtTheRateAndFramingItWasGiven) {
    const auto hpm = open_hpm(declare({}, DataId::kUart0));
    const auto mc02 = open_mc02(declare({}, kMc02Uart1));
    const auto bytes = pattern(96, 0x3C);
    const auto reply = pattern(96, 0xC3);

    // 换设置时线上要是静的；换完等端口重新稳定，再把换的过程中收到的东西丢掉。
    const auto settle = [&] {
        std::this_thread::sleep_for(kSettle);
        hpm->listener.clear_bytes();
        mc02->listener.clear_bytes();
    };

    struct Case {
        const char* name;
        UartSetting setting;
    };
    const auto framed = [](std::uint32_t baudrate, vc::UartParity parity, vc::UartStopBits stop) {
        return UartSetting{
            .baudrate = baudrate,
            .word_length = vc::kUartWordLength8,
            .parity = parity,
            .stop_bits = stop,
            .rx_polarity = vc::kUartRxPolarityNormal};
    };
    const Case cases[] = {
        { "115200 8N1",   framed(115'200, vc::kUartParityNone, vc::kUartStopBits1)},
        {"2000000 8N1", framed(2'000'000, vc::kUartParityNone, vc::kUartStopBits1)},
        { "921600 8E1",   framed(921'600, vc::kUartParityEven, vc::kUartStopBits1)},
        { "100000 8E2",   framed(100'000, vc::kUartParityEven, vc::kUartStopBits2)},
        {"1000000 8O1", framed(1'000'000,  vc::kUartParityOdd, vc::kUartStopBits1)},
    };
    for (const Case& test : cases) {
        hpm->board.handle(libhcs::spec::hpm5321::Spec::Uarts::kUart0).configure(test.setting);
        mc02->board.handle(libhcs::spec::mc02::Spec::Uarts::kUart1).configure(test.setting);
        settle();
        hpm->send_uart(bytes);
        EXPECT_EQ(mc02->listener.wait_bytes(bytes.size(), kArrival), bytes)
            << test.name << ", 5321 -> mc02";
        mc02->send_uart(reply);
        EXPECT_EQ(hpm->listener.wait_bytes(reply.size(), kArrival), reply)
            << test.name << ", mc02 -> 5321";
    }

    // 两边回到同一组，然后只动 mc02：先是速率，再是帧格式。
    const auto common = framed(kUartBaudrate, vc::kUartParityNone, vc::kUartStopBits1);
    const Case mismatches[] = {
        {   "rate",       framed(115'200, vc::kUartParityNone, vc::kUartStopBits1)},
        {"framing", framed(kUartBaudrate, vc::kUartParityEven, vc::kUartStopBits1)},
    };
    for (const Case& test : mismatches) {
        hpm->board.handle(libhcs::spec::hpm5321::Spec::Uarts::kUart0).configure(common);
        mc02->board.handle(libhcs::spec::mc02::Spec::Uarts::kUart1).configure(test.setting);
        settle();
        hpm->send_uart(bytes);
        EXPECT_NE(mc02->listener.wait_bytes(bytes.size(), kSilence), bytes)
            << "the mc02 still received the bytes intact after only its " << test.name
            << " was changed";
    }

    // 对不上的那几轮在 mc02 的接收机里留下过帧错误：改回来之后端口照常工作。
    mc02->board.handle(libhcs::spec::mc02::Spec::Uarts::kUart1).configure(common);
    settle();
    hpm->send_uart(bytes);
    EXPECT_EQ(mc02->listener.wait_bytes(bytes.size(), kArrival), bytes)
        << "the port did not recover from the mismatched rounds";
}

// 串口的接收错误随 keepalive 推给主机（v16，不走 EP0）。两个方向各验一次，两种计法：
// mc02 每次 HAL 错误回调记一次，5321 每轮读一次 LSR、见到错误位记一次。速率对上之后
// 计数不再涨。
TEST_F(Mc02Bench, UartReceiveErrorsArePushedToTheHost) {
    const auto hpm = open_hpm(declare({}, DataId::kUart0));
    const auto mc02 = open_mc02(declare({}, kMc02Uart1));
    std::this_thread::sleep_for(kSettle);
    auto hpm_uart = hpm->board.handle(libhcs::spec::hpm5321::Spec::Uarts::kUart0);
    auto mc02_uart = mc02->board.handle(libhcs::spec::mc02::Spec::Uarts::kUart1);
    const auto errors = [](const libhcs::data::UartStatusView& s) {
        return s.overrun + s.parity + s.framing + s.noise + s.unattributed;
    };
    const auto common = uart_8n1(kUartBaudrate);
    const auto slow = uart_8n1(115'200);

    // 5321 发 1 M，mc02 按 115200 收：mc02 的接收机满是帧错误。
    mc02_uart.configure(slow);
    std::this_thread::sleep_for(kSettle);
    const auto mc02_before = errors(mc02_uart.status());
    hpm->send_uart(pattern(256, 0x5A));
    const auto mc02_after = errors(wait_status([&] { return mc02_uart.status(); }, [&](const auto& s) {
        return errors(s) > mc02_before;
    }));
    EXPECT_GT(mc02_after, mc02_before) << "the mc02 saw garbage but reported no receive error";

    // 反过来：mc02 按 115200 发，5321 按 1 M 收。
    const auto hpm_before = errors(hpm_uart.status());
    mc02->send_uart(pattern(256, 0xA5));
    const auto hpm_after = errors(wait_status([&] { return hpm_uart.status(); }, [&](const auto& s) {
        return errors(s) > hpm_before;
    }));
    EXPECT_GT(hpm_after, hpm_before) << "the 5321 saw garbage but reported no receive error";

    // 对上之后干净的字节不再添错误。
    mc02_uart.configure(common);
    std::this_thread::sleep_for(kSilence); // 换设置那一下可能留一个错误, 等它报完
    const auto settled_mc02 = errors(mc02_uart.status());
    const auto settled_hpm = errors(hpm_uart.status());
    mc02->listener.clear_bytes();
    const auto bytes = pattern(96, 0x3C);
    hpm->send_uart(bytes);
    EXPECT_EQ(mc02->listener.wait_bytes(bytes.size(), kArrival), bytes);
    mc02->send_uart(bytes);
    std::this_thread::sleep_for(kSilence);
    EXPECT_EQ(errors(mc02_uart.status()), settled_mc02);
    EXPECT_EQ(errors(hpm_uart.status()), settled_hpm);
}

// 5321
// 一侧的串口：没声明时接收是整个停着的（不只是不转发），对面发什么都没有；重新握手
// 声明之后从干净的状态起来，停着期间线上那些字节一个都不带进来。反复几轮、每轮长度不同，
// 让接收环每次停在不一样的位置上再重起。
TEST_F(Mc02Bench, The5321UartIsOffUntilDeclaredAndStartsClean) {
    const auto mc02 = open_mc02(declare({}, kMc02Uart1));
    const auto send_in_chunks = [&](const std::vector<std::byte>& bytes) {
        for (std::size_t offset = 0; offset < bytes.size(); offset += 64)
            mc02->send_uart(
                std::span{bytes}.subspan(offset, std::min<std::size_t>(64, bytes.size() - offset)));
    };

    std::uint8_t round = 0;
    for (const std::size_t size : {96U, 333U, 1500U, 40U}) {
        {
            const auto hpm = open_hpm(declare({DataId::kCan1}, std::nullopt, kFd1M5M));
            std::this_thread::sleep_for(kSettle);
            send_in_chunks(pattern(size, static_cast<std::uint8_t>(0x11 + round)));
            EXPECT_TRUE(hpm->listener.wait_bytes(1, kSilence).empty())
                << "round " << int{round} << ": bytes were forwarded from an undeclared UART";
        }

        const auto hpm = open_hpm(declare({}, DataId::kUart0));
        std::this_thread::sleep_for(kSettle);
        const auto bytes = pattern(size, static_cast<std::uint8_t>(0x80 + round));
        send_in_chunks(bytes);
        EXPECT_EQ(hpm->listener.wait_bytes(bytes.size(), kArrival), bytes)
            << "round " << int{round} << ", mc02 -> 5321";

        const auto reply = pattern(96, static_cast<std::uint8_t>(0xC0 + round));
        hpm->send_uart(reply);
        EXPECT_EQ(mc02->listener.wait_bytes(reply.size(), kArrival), reply)
            << "round " << int{round} << ", 5321 -> mc02";
        mc02->listener.clear_bytes();
        ++round;
    }
}

// 5321
// 的串口被挂起时，挂起之前主机已经交给它的字节照常发完：新一轮握手不把半截数据
// 截在板子里，也不留到下次声明时才冒出来。用慢速率让这批字节在线上要走 170 ms，握手落在发送中途。
TEST_F(Mc02Bench, BytesThe5321AlreadyAcceptedStillGoOutAfterItsUartIsSuspended) {
    constexpr std::uint32_t slow = 115'200;
    Configuration mc02_uart, hpm_uart;
    mc02_uart.declare_uart(kMc02Uart1, uart_8n1(slow));
    hpm_uart.declare_uart(DataId::kUart0, uart_8n1(slow));
    const auto mc02 = open_mc02(mc02_uart);

    const auto bytes = pattern(2000, 0x5A);
    {
        const auto hpm = open_hpm(hpm_uart);
        std::this_thread::sleep_for(kSettle);
        for (std::size_t offset = 0; offset < bytes.size(); offset += 100)
            hpm->send_uart(std::span{bytes}.subspan(offset, 100));
        // 等这批字节过了 USB 进到板子的发送环里（线上此时才走了不到十分之一）。
        std::this_thread::sleep_for(15ms);
    }
    // 新的握手：串口没声明，于是被挂起。
    const auto hpm = open_hpm(declare({DataId::kCan1}, std::nullopt, kFd1M5M));
    EXPECT_LT(mc02->listener.wait_bytes(bytes.size(), 0ms).size(), bytes.size())
        << "the transmission had already finished before the suspend: this run proves nothing";
    EXPECT_EQ(mc02->listener.wait_bytes(bytes.size(), kArrival), bytes);

    // 排空之后这一口不再出声。
    mc02->listener.clear_bytes();
    EXPECT_TRUE(mc02->listener.wait_bytes(1, kSilence).empty());
}

// ── 时间戳 ───────────────────────────────────────────────────────────────

/// 开了时间基准的板：记下每一帧带的时间戳和它到主机的时刻，以及板子每轮对时的状态
/// 上报。5321 与 mc02 的回调签名不同，各有一个薄的派生类。
struct TimedInbox {
    struct Arrival {
        std::uint32_t can_id;
        std::optional<libhcs::time::SofStamp> stamp; ///< 原始 24 位戳（跨板互减用）
        libhcs::time::SampleTime time;               ///< SDK 算好的样本时刻
        std::chrono::steady_clock::time_point at;    ///< 到达时刻
    };

    void push_arrival(
        const libhcs::data::CanDataView& data, const libhcs::time::SampleTime& time) {
        const auto now = std::chrono::steady_clock::now();
        const std::scoped_lock lock{mutex};
        arrivals.push_back({data.can_id, data.sof_stamp, time, now});
        changed.notify_all();
    }
    void push_status(const libhcs::data::TimeStatusView& data) {
        const std::scoped_lock lock{mutex};
        if (data.state == libhcs::data::TimeState::kValid)
            ++valid_reports;
        capture_fresh += data.capture_fresh_count;
        capture_stale += data.capture_stale_count;
        ++reports;
        changed.notify_all();
    }

    /// 等板子报出 count 次 valid 的对时状态；返回实际等到的次数。
    std::size_t wait_valid(std::size_t count, std::chrono::milliseconds timeout) {
        std::unique_lock lock{mutex};
        changed.wait_for(lock, timeout, [&] { return valid_reports >= count; });
        return valid_reports;
    }

    std::optional<Arrival> wait(std::uint32_t can_id, std::chrono::milliseconds timeout) {
        std::unique_lock lock{mutex};
        const auto match = [&](const Arrival& arrival) { return arrival.can_id == can_id; };
        if (!changed.wait_for(lock, timeout, [&] { return std::ranges::any_of(arrivals, match); }))
            return std::nullopt;
        return *std::ranges::find_if(arrivals, match);
    }

    std::mutex mutex;
    std::condition_variable changed;
    std::vector<Arrival> arrivals;
    std::size_t reports = 0, valid_reports = 0;
    std::uint64_t capture_fresh = 0, capture_stale = 0;
};

struct TimedListener final
    : Hpm5321::Callback
    , TimedInbox {
    void can_receive_callback(libhcs::spec::hpm5321::Spec::Can, const libhcs::data::CanDataView& d,
        const libhcs::time::SampleTime& time) override {
        push_arrival(d, time);
    }
    void time_status_callback(const libhcs::data::TimeStatusView& data) override {
        push_status(data);
    }
};

struct TimedMc02Listener final
    : Mc02::Callback
    , TimedInbox {
    void can_receive_callback(libhcs::spec::mc02::Spec::Can, const libhcs::data::CanDataView& d,
        const libhcs::time::SampleTime& time) override {
        push_arrival(d, time);
    }
    void time_status_callback(const libhcs::data::TimeStatusView& data) override {
        push_status(data);
    }
};

// 5321 给收到的每一帧打的硬件时间戳，SDK 换到本机 steady_clock（CLOCK_MONOTONIC）之后
// （回调里的 SampleTime），必须落在 "主机把这一帧交给 mc02 去发"与"主机从 5321 收到这一帧"
// 之间——帧是在这两个时刻之间上的总线。这是整条链路的验收：板上两个硬件锁存值相减、24 位
// 时间戳的还原、微帧轴到时钟的映射，任何一环错一个微帧（125 µs），就会有帧落到括区外面。
//
// 5321 的时间基准由声明打开（with_time_sync）。
TEST_F(Mc02Bench, The5321PlacesReceivedFramesOnTheHostClock) {
    TimedListener listener;
    Hpm5321 hpm{
        listener, serial_hpm_,
        libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true),
        with_time_sync(declare({DataId::kCan1}, std::nullopt, kFd1M5M))};
    const auto mc02 = open_mc02(declare({DataId::kCan1}, std::nullopt, kFd1M5M));

    // 板子的时间线：对时每 250 ms 一轮，拟合窗口 1 s，所以两三秒内应当报 valid。
    {
        std::unique_lock lock{listener.mutex};
        listener.changed.wait_for(lock, 6s, [&] { return listener.valid_reports >= 4; });
        ASSERT_GT(listener.valid_reports, 0U)
            << "the 5321 accepted the time base but never reported a valid time line ("
            << listener.reports << " reports)";
    }

    // 主机一侧：SampleTime 的来源 == kBoardStamp 就是"戳已经换到本机时钟"。读得了主机
    // 控制器的计数器时亚微秒，读不了按 USB 往返拟合时几十微秒——括区检查对两种都成立，
    // 实际精度看下面打印的分布。
    const auto tolerance = 300us;

    constexpr int frames = 200;
    std::vector<double> after_send_us, before_arrival_us;
    for (int i = 0; i < frames; ++i) {
        const auto can_id = static_cast<std::uint32_t>(0x400 + i);
        const auto payload = pattern(8, static_cast<std::uint8_t>(i));
        const auto sent = std::chrono::steady_clock::now();
        mc02->send(DataId::kCan1, can_id, payload);
        const auto arrival = listener.wait(can_id, kArrival);
        ASSERT_TRUE(arrival.has_value()) << "frame " << i << " never arrived";
        ASSERT_TRUE(arrival->stamp.has_value())
            << "frame " << i << " carries no timestamp: the capture ring could not place it";
        ASSERT_EQ(arrival->time.source, libhcs::time::SampleTime::Source::kBoardStamp)
            << "frame " << i << " arrived with a stamp the SDK could not place on the host clock";

        const auto& started = arrival->time.host;
        EXPECT_GE(started, sent - tolerance)
            << "frame " << i << " is stamped "
            << std::chrono::duration<double, std::micro>(sent - started).count()
            << " us BEFORE the host asked for it to be sent";
        EXPECT_LE(started, arrival->at + tolerance)
            << "frame " << i << " is stamped "
            << std::chrono::duration<double, std::micro>(started - arrival->at).count()
            << " us AFTER the host received it";
        after_send_us.push_back(std::chrono::duration<double, std::micro>(started - sent).count());
        before_arrival_us.push_back(
            std::chrono::duration<double, std::micro>(arrival->at - started).count());
        std::this_thread::sleep_for(2ms);
    }

    // 从帧起始到主机收到，至少是这一帧在线上的时间（FD 1M/5M 8 字节约 50 µs）加
    // USB 上行。比这还短，说明时间戳偏晚了。
    std::ranges::sort(after_send_us);
    std::ranges::sort(before_arrival_us);
    EXPECT_GT(before_arrival_us.front(), 40.0);

    const std::scoped_lock lock{listener.mutex};
    const auto captured = listener.capture_fresh + listener.capture_stale;
    ASSERT_GT(listener.capture_fresh, 0U) << "no Start-of-Frame was ever latched by hardware";
    // 双沿捕获（2026-10-05 起）：每个 SOF 都锁到。只取上升沿的旧镜像恰是 50%。
    EXPECT_GT(
        static_cast<double>(listener.capture_fresh), 0.99 * static_cast<double>(captured))
        << "the 5321 latches only part of the SOFs: an image without the double-edge capture?";
    std::printf(
        "    sample time: converted from the board stamp (SampleTime::kBoardStamp)\n"
        "    frame start after the host's send call: min %.1f / median %.1f / max %.1f us\n"
        "    host receive after the frame start:     min %.1f / median %.1f / max %.1f us\n"
        "    hardware SOF captures: %llu fresh, %llu stale (%.1f%% fresh)\n",
        after_send_us.front(), after_send_us[after_send_us.size() / 2], after_send_us.back(),
        before_arrival_us.front(), before_arrival_us[before_arrival_us.size() / 2],
        before_arrival_us.back(), static_cast<unsigned long long>(listener.capture_fresh),
        static_cast<unsigned long long>(listener.capture_stale),
        100.0 * static_cast<double>(listener.capture_fresh) / static_cast<double>(captured));
}

// 时间基准是声明的一部分：清单要了才开，没要就关——连上一个会话留着开的也关。一块 5321 与
// 一块 mc02（CAN1 在同一条总线上）按"要、不要、再要"连开三轮，每轮互发 50 帧，看收到的帧
// 带不带时间戳。中间那轮紧跟在时间基准开着的会话之后，验的正是"新清单没要它，板子就把它
// 关掉"；第三轮验关了还能再开。两板台架与三板台架各跑一遍。
void expect_time_base_only_while_declared(const std::string& serial_hpm, const std::string& serial_mc02) {
    struct Round {
        std::size_t hpm_frames = 0, hpm_stamped = 0, hpm_reports = 0;
        std::size_t mc02_frames = 0, mc02_stamped = 0, mc02_reports = 0;
    };
    const auto run = [&](bool time_sync) {
        Configuration declaration = declare({DataId::kCan1}, std::nullopt, kFd1M5M);
        declaration.enable_time_sync(time_sync);
        TimedListener hpm_listener;
        TimedMc02Listener mc02_listener;
        Hpm5321 hpm{
            hpm_listener, serial_hpm,
            libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true), declaration};
        Mc02 mc02{
            mc02_listener, serial_mc02,
            libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true), declaration};
        if (time_sync) {
            (void)hpm_listener.wait_valid(2, 6s);
            (void)mc02_listener.wait_valid(2, 6s);
        } else {
            std::this_thread::sleep_for(1500ms); // 同样几轮保活：板子要是还在报时间线，就该报了
        }

        // 两边错开发：CAN 单发不重传，同一时刻起发的两帧输掉仲裁的那帧就丢了。
        const std::array<std::byte, 8> payload{};
        for (std::uint32_t i = 0; i < 50; ++i) {
            mc02.start_transmit().can_transmit(
                libhcs::spec::mc02::Spec::Cans::kCan1, {.can_id = 0x600 + i, .can_data = payload});
            std::this_thread::sleep_for(1ms);
            hpm.start_transmit().can_transmit(
                libhcs::spec::hpm5321::Spec::Cans::kCan1,
                {.can_id = 0x680 + i, .can_data = payload});
            std::this_thread::sleep_for(1ms);
        }
        std::this_thread::sleep_for(50ms);

        Round round;
        const auto stamped = [](const TimedInbox::Arrival& arrival) {
            return arrival.stamp.has_value();
        };
        const auto mine = [](std::uint32_t low) {
            return [low](const TimedInbox::Arrival& arrival) {
                return arrival.can_id >= low && arrival.can_id < low + 50;
            };
        };
        {
            const std::scoped_lock lock{hpm_listener.mutex};
            auto from_mc02 = hpm_listener.arrivals | std::views::filter(mine(0x600));
            round.hpm_frames = std::ranges::distance(from_mc02);
            round.hpm_stamped = std::ranges::count_if(from_mc02, stamped);
            round.hpm_reports = hpm_listener.reports;
        }
        {
            const std::scoped_lock lock{mc02_listener.mutex};
            auto from_hpm = mc02_listener.arrivals | std::views::filter(mine(0x680));
            round.mc02_frames = std::ranges::distance(from_hpm);
            round.mc02_stamped = std::ranges::count_if(from_hpm, stamped);
            round.mc02_reports = mc02_listener.reports;
        }
        std::printf(
            "    time_sync %-3s: 5321 %zu/%zu frames stamped, %zu reports; "
            "mc02 %zu/%zu stamped, %zu reports\n",
            time_sync ? "on" : "off", round.hpm_stamped, round.hpm_frames, round.hpm_reports,
            round.mc02_stamped, round.mc02_frames, round.mc02_reports);
        return round;
    };

    for (const bool time_sync : {true, false, true}) {
        const Round round = run(time_sync);
        ASSERT_GT(round.hpm_frames, 40U) << "the 5321 did not receive the mc02's frames";
        ASSERT_GT(round.mc02_frames, 40U) << "the mc02 did not receive the 5321's frames";
        if (time_sync) {
            EXPECT_EQ(round.hpm_stamped, round.hpm_frames) << "declared, yet the 5321 missed stamps";
            EXPECT_EQ(round.mc02_stamped, round.mc02_frames) << "declared, yet the mc02 missed stamps";
            EXPECT_GT(round.hpm_reports, 0U);
            EXPECT_GT(round.mc02_reports, 0U);
        } else {
            EXPECT_EQ(round.hpm_stamped, 0U) << "the 5321 still runs a time base nobody declared";
            EXPECT_EQ(round.mc02_stamped, 0U) << "the mc02 still runs a time base nobody declared";
            EXPECT_EQ(round.hpm_reports, 0U);
            EXPECT_EQ(round.mc02_reports, 0U);
        }
    }
}

TEST_F(Mc02Bench, TheTimeBaseRunsOnlyWhileDeclared) {
    expect_time_base_only_while_declared(serial_hpm_, serial_mc02_);
}

// ── 三板台架：跨板时间戳 ─────────────────────────────────────────────────

/// 开了时间基准的一块 5321 / mc02，CAN1 声明为 FD 1M/5M。
struct TimedHpm {
    explicit TimedHpm(const std::string& serial)
        : board(
              listener, serial,
              libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true),
              with_time_sync(declare({DataId::kCan1}, std::nullopt, kFd1M5M))) {}

    void send(std::uint32_t can_id, std::span<const std::byte> data) {
        board.start_transmit().can_transmit(
            libhcs::spec::hpm5321::Spec::Cans::kCan1, {.can_id = can_id, .can_data = data});
    }

    TimedListener listener;
    Hpm5321 board;
};

struct TimedMc02 {
    explicit TimedMc02(const std::string& serial)
        : board(
              listener, serial,
              libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true),
              with_time_sync(declare({DataId::kCan1}, std::nullopt, kFd1M5M))) {}

    void send(std::uint32_t can_id, std::span<const std::byte> data) {
        board.start_transmit().can_transmit(
            libhcs::spec::mc02::Spec::Cans::kCan1, {.can_id = can_id, .can_data = data});
    }

    TimedMc02Listener listener;
    Mc02 board;
};

/// b - a, 两个时间戳之差（ns）：24 位回绕上取最近的那个方向。
double stamp_difference_ns(libhcs::time::SofStamp a, libhcs::time::SofStamp b) {
    constexpr auto modulus = static_cast<std::int64_t>(libhcs::time::SofStamp::kModulus);
    std::int64_t ticks = (static_cast<std::int64_t>(b.ticks) - a.ticks) % modulus;
    if (ticks >= modulus / 2)
        ticks -= modulus;
    if (ticks < -modulus / 2)
        ticks += modulus;
    return static_cast<double>(ticks) * 125'000.0 / libhcs::time::SofStamp::kTicksPerMicroframe;
}

/// HCS_BENCH_DUMP=<文件> 时，把每一帧的原始时间戳追加写进去（诊断用，默认不写）。
/// 每行：用例, 帧号, 板1 的 ticks, 板2 的 ticks, 板1 收到的 steady ns, 板2 收到的 steady ns。
void dump_pair(
    const char* what, int frame, libhcs::time::SofStamp first, libhcs::time::SofStamp second,
    std::chrono::steady_clock::time_point first_at,
    std::chrono::steady_clock::time_point second_at) {
    static std::FILE* const file = [] {
        const char* path = std::getenv("HCS_BENCH_DUMP");
        return path != nullptr && *path != '\0' ? std::fopen(path, "a") : nullptr;
    }();
    if (file == nullptr)
        return;
    const auto ns = [](std::chrono::steady_clock::time_point at) {
        return static_cast<long long>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(at.time_since_epoch()).count());
    };
    std::fprintf(
        file, "%s,%d,%u,%u,%lld,%lld\n", what, frame, first.ticks, second.ticks, ns(first_at),
        ns(second_at));
    std::fflush(file);
}

/// 一组跨板差值的统计。中位数与 MAD 换算的 sigma 不受个别离群点左右；离群点另数。
struct Spread {
    double median = 0.0, mean = 0.0, sigma = 0.0, robust_sigma = 0.0, worst_from_median = 0.0;
    std::size_t outliers = 0; // 离中位数超过 1 us
};

Spread spread_of(std::vector<double> values) {
    Spread result;
    if (values.empty())
        return result;
    std::ranges::sort(values);
    result.median = values[values.size() / 2];
    double sum = 0.0, square_sum = 0.0;
    std::vector<double> deviations;
    for (const double value : values) {
        sum += value;
        square_sum += value * value;
        const double deviation = std::fabs(value - result.median);
        deviations.push_back(deviation);
        result.worst_from_median = std::max(result.worst_from_median, deviation);
        if (deviation > 1000.0)
            ++result.outliers;
    }
    const auto n = static_cast<double>(values.size());
    result.mean = sum / n;
    result.sigma = std::sqrt(std::max(0.0, (square_sum / n) - (result.mean * result.mean)));
    std::ranges::sort(deviations);
    result.robust_sigma = 1.4826 * deviations[deviations.size() / 2];
    return result;
}

void print_spread(const char* what, const Spread& spread, std::size_t count) {
    std::printf(
        "    %s, %zu frames: median %+.1f ns, mean %+.1f ns, sigma %.1f ns (robust %.1f), "
        "worst %.1f ns from the median, %zu beyond 1 us\n",
        what, count, spread.median, spread.mean, spread.sigma, spread.robust_sigma,
        spread.worst_from_median, spread.outliers);
}

class ThreeBoardBench : public testing::Test {
protected:
    void SetUp() override {
        const auto boards = three_board_bench();
        if (!boards)
            GTEST_SKIP() << "needs two dual-CAN HPM5321 and one mc02 with their CAN1 on one bus; "
                            "found "
                         << attached_boards().size() << " HPM5321, " << attached_mc02().size()
                         << " mc02";
        serial_a_ = (*boards)[0];
        serial_b_ = (*boards)[1];
        serial_mc02_ = (*boards)[2];
    }

    /// 等每块板的时间线报 valid。没开时间基准的固件从不报 valid：跳过而不是失败
    /// （返回 false）。主机一侧的映射锁定与否，看帧自带的 SampleTime 来源。
    [[nodiscard]] static bool wait_for_time(std::initializer_list<TimedInbox*> boards) {
        for (TimedInbox* board : boards) {
            if (board->wait_valid(4, 6s) == 0)
                return false;
        }
        return true;
    }

    std::string serial_a_, serial_b_, serial_mc02_;
};

TEST_F(ThreeBoardBench, TheTimeBaseRunsOnlyWhileDeclared) {
    expect_time_base_only_while_declared(serial_a_, serial_mc02_);
}

// 两块 5321 收到同一帧（同一条总线，相距几十厘米，传播差 ns 级），两个时间戳之差就是
// 跨板误差：两块板各自的硬件锁存、各自的 SOF 环拟合、线格式的量化，全在这一个数里。
// 均值是两块板收发器延迟之类的常数差，sigma 才是随机误差。
//
// 预算 [推断, 未上板]：每块板约 5 ns（CAN 时钟 12.5 ns 的帧起始沿同步 3.6、PTPC 6.25 ns 1.8、
// 31 个 SOF 锁存值的直线约 2、7.6 ns 的时间戳量化 2.2），两块合成约 7 ns。判据按 25 ns 给，
// 第一次上板后按实测收紧。
TEST_F(ThreeBoardBench, TwoHpmBoardsStampTheSameFrameAlike) {
    TimedHpm a{serial_a_};
    TimedHpm b{serial_b_};
    TimedMc02 mc02{serial_mc02_};
    if (!wait_for_time({&a.listener, &b.listener}))
        FAIL() << "a 5321 accepted the time base but never reported a valid time line";

    constexpr int frames = 500;
    std::vector<double> differences;
    for (int i = 0; i < frames; ++i) {
        const auto can_id = static_cast<std::uint32_t>(0x500 + (i % 0x100));
        const auto payload = pattern(8, static_cast<std::uint8_t>(i));
        mc02.send(can_id, payload);
        const auto at_a = a.listener.wait(can_id, kArrival);
        const auto at_b = b.listener.wait(can_id, kArrival);
        ASSERT_TRUE(at_a.has_value() && at_b.has_value()) << "frame " << i << " did not arrive";
        ASSERT_TRUE(at_a->stamp.has_value() && at_b->stamp.has_value())
            << "frame " << i << " carries no timestamp on board " << (at_a->stamp ? "B" : "A");
        differences.push_back(stamp_difference_ns(*at_a->stamp, *at_b->stamp));
        dump_pair("two_hpm", i, *at_a->stamp, *at_b->stamp, at_a->at, at_b->at);
        {
            const std::scoped_lock lock_a{a.listener.mutex};
            a.listener.arrivals.clear();
        }
        {
            const std::scoped_lock lock_b{b.listener.mutex};
            b.listener.arrivals.clear();
        }
        std::this_thread::sleep_for(2ms);
    }

    const Spread spread = spread_of(differences);
    print_spread("B - A", spread, differences.size());
    std::printf(
        "    hardware SOF captures: A %llu fresh / %llu stale, B %llu fresh / %llu stale\n",
        static_cast<unsigned long long>(a.listener.capture_fresh),
        static_cast<unsigned long long>(a.listener.capture_stale),
        static_cast<unsigned long long>(b.listener.capture_fresh),
        static_cast<unsigned long long>(b.listener.capture_stale));

    EXPECT_EQ(spread.outliers, 0U);
    EXPECT_LT(std::fabs(spread.median), 200.0) << "a constant offset between two identical boards";
    EXPECT_LT(spread.sigma, 25.0);
    EXPECT_LT(spread.worst_from_median, 100.0);
}

// mc02 与一块 5321 收到同一帧（5321 A 发）。mc02 的帧时间戳走 TIM3（FDCAN 外部时间戳）
// -> TIM5（整数关系拼接）-> SOF 环，USB 是全速；它对 5321 的中位数是两块板 SOF 锁存延迟之差
// 这样的常数，要标定进 mc02 固件；离群点是 FDCAN 采样 TIM3 时锁到坏值的直接证据，
// 一个都不许有。同时验 mc02 的时间戳换到主机时钟后落在"发出"与"收到"之间。
//
// mc02 的 .ioc 要 TIM5 Prescaler 0 / Period 4294967295、TIM3 Prescaler 1 / Period 65535
// （firmware/mc02/AGENTS.md「CAN 帧时间戳」），否则 mc02 不打戳，本用例跳过。
TEST_F(ThreeBoardBench, TheMc02StampsTheSameFrameAsA5321) {
    TimedHpm a{serial_a_};
    TimedHpm b{serial_b_};
    TimedMc02 mc02{serial_mc02_};
    if (!wait_for_time({&b.listener, &mc02.listener}))
        FAIL() << "a board accepted the time base but never reported a valid time line";
    const auto tolerance = 300us;

    constexpr int frames = 500;
    std::vector<double> differences;
    int unstamped = 0;
    for (int i = 0; i < frames; ++i) {
        const auto can_id = static_cast<std::uint32_t>(0x600 + (i % 0x100));
        const auto payload = pattern(8, static_cast<std::uint8_t>(i));
        const auto sent = std::chrono::steady_clock::now();
        a.send(can_id, payload);
        const auto at_b = b.listener.wait(can_id, kArrival);
        const auto at_mc02 = mc02.listener.wait(can_id, kArrival);
        ASSERT_TRUE(at_b.has_value() && at_mc02.has_value()) << "frame " << i << " did not arrive";
        ASSERT_TRUE(at_b->stamp.has_value()) << "frame " << i << " carries no timestamp on the 5321";
        if (!at_mc02->stamp) {
            ++unstamped;
        } else {
            differences.push_back(stamp_difference_ns(*at_b->stamp, *at_mc02->stamp));
            dump_pair("mc02_hpm", i, *at_b->stamp, *at_mc02->stamp, at_b->at, at_mc02->at);

            ASSERT_EQ(at_mc02->time.source, libhcs::time::SampleTime::Source::kBoardStamp);
            const auto& started = at_mc02->time.host;
            EXPECT_GE(started, sent - tolerance) << "frame " << i << " stamped before it was sent";
            EXPECT_LE(started, at_mc02->at + tolerance)
                << "frame " << i << " stamped after the host received it";
        }
        {
            const std::scoped_lock lock_b{b.listener.mutex};
            b.listener.arrivals.clear();
        }
        {
            const std::scoped_lock lock_mc02{mc02.listener.mutex};
            mc02.listener.arrivals.clear();
        }
        std::this_thread::sleep_for(2ms);
    }

    if (differences.empty())
        FAIL() << "the mc02 stamps no frame: its .ioc is not at TIM5 0/0xFFFFFFFF + TIM3 1/65535, "
                  "or TIM5 ITR7 carries no SOF";
    const Spread spread = spread_of(differences);
    print_spread("mc02 - 5321", spread, differences.size());
    std::printf(
        "    mc02 SOF captures %llu fresh / %llu stale; %d of %d frames unstamped\n",
        static_cast<unsigned long long>(mc02.listener.capture_fresh),
        static_cast<unsigned long long>(mc02.listener.capture_stale), unstamped, frames);

    EXPECT_EQ(unstamped, 0);
    EXPECT_EQ(spread.outliers, 0U) << "torn TIM3 samples (or a wrong pairing) on the mc02";
    EXPECT_LT(std::fabs(spread.median), 5000.0) << "the two axes are not the same axis";
    EXPECT_LT(spread.robust_sigma, 60.0);
}

// mc02 板载 IMU（BMI088）的样本带的是板上自己的 1/4 us 时间戳（数据就绪中断里读 1 MHz 的
// TIM23）。回调里的 SampleTime：board 是这块板 64 位计时器上的时刻（相邻样本相减就是积分要
// 的 dt，不经过主机），host 是换到本机 steady_clock 的同一刻，应当落在主机收到它之前、且不早
// 于几毫秒（全速 USB 加批量）。全速的 mc02 一个人锁不住主机计数器的整数偏移，所以同时开一块 5321。
TEST_F(ThreeBoardBench, TheMc02ImuSamplesLandOnTheHostClock) {
    struct Listener final : Mc02::Callback {
        void time_status_callback(const libhcs::data::TimeStatusView& data) override {
            const std::scoped_lock lock{mutex};
            if (data.state == libhcs::data::TimeState::kValid)
                ++valid_reports;
        }
        void gyroscope_receive_callback(
            const libhcs::data::ImuGyroscopeDataView& data,
            const libhcs::time::SampleTime& time) override {
            const auto now = std::chrono::steady_clock::now();
            const std::scoped_lock lock{mutex};
            if (recording && samples.size() < 4000)
                samples.push_back({time, now});
        }
        struct Sample {
            libhcs::time::SampleTime time;
            std::chrono::steady_clock::time_point at;
        };
        std::mutex mutex;
        std::size_t valid_reports = 0;
        bool recording = false;
        std::vector<Sample> samples;

        [[nodiscard]] bool saw_a_converted_sample() const {
            return std::ranges::any_of(samples, [](const Sample& sample) {
                return sample.time.source == libhcs::time::SampleTime::Source::kBoardStamp;
            });
        }
    };

    TimedHpm a{serial_a_};
    Listener listener;
    Configuration configuration = declare({DataId::kCan1}, std::nullopt, kFd1M5M);
    configuration.declare_imu(
        ImuSetting{
            .accelerometer_range_g = 6,
            .accelerometer_rate_hz = 1600,
            .gyroscope_range_dps = 2000,
            .gyroscope_rate_hz = 2000,
            .gyroscope_bandwidth_hz = 230});
    Mc02 mc02{
        listener, serial_mc02_,
        libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true),
        with_time_sync(configuration)};

    const auto deadline = std::chrono::steady_clock::now() + 20s;
    while (std::chrono::steady_clock::now() < deadline) {
        {
            const std::scoped_lock lock{listener.mutex};
            if (listener.valid_reports >= 4 && listener.saw_a_converted_sample())
                break;
        }
        std::this_thread::sleep_for(50ms);
    }
    {
        const std::scoped_lock lock{listener.mutex};
        ASSERT_GT(listener.valid_reports, 0U)
            << "the mc02 accepted the time base but never reported a valid time line";
        ASSERT_TRUE(listener.saw_a_converted_sample())
            << "the host never placed an IMU sample on its clock";
        listener.recording = true;
    }
    std::this_thread::sleep_for(1s);
    const std::scoped_lock lock{listener.mutex};
    listener.recording = false;
    ASSERT_GT(listener.samples.size(), 1000U) << "the IMU did not stream";

    const auto tolerance = 300us;
    std::vector<double> lead_us;      // host receive - sample time（已换算的样本）
    std::vector<double> interval_us;  // 板上时刻的间隔：dt 本来就该在这里算
    std::optional<libhcs::time::BoardClock::time_point> previous_board;
    for (const auto& sample : listener.samples) {
        ASSERT_TRUE(sample.time.board.has_value()) << "an IMU sample without its board time";
        if (previous_board)
            interval_us.push_back(
                std::chrono::duration<double, std::micro>(*sample.time.board - *previous_board)
                    .count());
        previous_board = sample.time.board;

        if (sample.time.source != libhcs::time::SampleTime::Source::kBoardStamp)
            continue; // 个别样本到达时映射还没锁定：host 是到达时刻，不进括区检查
        EXPECT_LE(sample.time.host, sample.at + tolerance)
            << "a sample stamped after the host received it";
        EXPECT_GE(sample.time.host, sample.at - 5ms)
            << "a sample stamped long before it arrived";
        lead_us.push_back(
            std::chrono::duration<double, std::micro>(sample.at - sample.time.host).count());
    }
    ASSERT_FALSE(lead_us.empty())
        << "every sample arrived while the mappings were unlocked: the axis did not hold";
    std::ranges::sort(lead_us);
    double interval_sum = 0;
    for (const double v : interval_us)
        interval_sum += v;
    const double interval_mean = interval_sum / static_cast<double>(interval_us.size());
    std::printf(
        "    %zu gyroscope samples, %zu of them converted to the host clock\n"
        "    host receive after the sample's data-ready: min %.1f / median %.1f / max %.1f us\n"
    "    sample interval on the board clock: mean %.2f us (2 kHz nominal: 500)\n",
        listener.samples.size(), lead_us.size(), lead_us.front(), lead_us[lead_us.size() / 2],
        lead_us.back(), interval_mean);
    // BMI088 的片内振荡器有百分之几的容差：只查量级对。
    EXPECT_NEAR(interval_mean, 500.0, 25.0);
}

// 板子对"字节完整但拒收"的下行记录只丢那一条：同一批里排在后面的正常帧照常转发，
// 且下一次 keepalive 应答在 kKeepaliveAck 之后追加链路状态（kPortStatus），原因 kRefused、
// 字段是被拒的口，计数只记这一条。发送侧直接用协议层 Handler（绕过板卡类的
// UndeclaredChannel 门），等于一个写错了的主机程序：5321 没有 CAN3。
TEST_F(ThreeBoardBench, ARefusedRecordIsSkippedAndReported) {
    auto a = raw_hpm(serial_a_, declare({DataId::kCan1}));
    const auto b = std::make_unique<Node>(serial_b_, declare({DataId::kCan1}));
    std::this_thread::sleep_for(kSettle);
    ASSERT_TRUE(a.link_up());

    // 基线：kStart 之后的第一轮 keepalive 应答带上链路状态（错误计数从上电起算，可能非零）。
    const auto has_status = [](const auto& status) { return status.has_value(); };
    const auto baseline = wait_status([&] { return a.link_status(); }, has_status);
    ASSERT_TRUE(baseline) << "no link status after the session started";

    const auto payload = pattern(8, 0x88);
    a.send_refused_then_can(DataId::kCan3, 0x3AA, DataId::kCan1, 0x3AB, payload);
    EXPECT_TRUE(b->listener.wait_frame(DataId::kCan1, 0x3AB, kArrival).has_value())
        << "the good record after the refused one was lost";

    // 链路状态搭下一轮 keepalive 应答（250 ms 一轮），1 s 内必有。
    const auto status = wait_status([&] { return a.link_status(); }, [&](const auto& now) {
        return now && now->downlink_errors != baseline->downlink_errors;
    });
    ASSERT_TRUE(status);
    EXPECT_EQ(static_cast<std::uint16_t>(status->downlink_errors - baseline->downlink_errors), 1U)
        << "the good record was counted as an error too";
    EXPECT_EQ(status->last_field, DataId::kCan3);
    EXPECT_EQ(status->last_reason, libhcs::data::DownlinkError::kRefused);

    // 之后不再有新错误：报一次就够。
    std::this_thread::sleep_for(kSilence);
    EXPECT_EQ(a.link_status()->downlink_errors, status->downlink_errors);
}

// ── mc02 的时间戳，两板台架就能验的部分 ───────────────────────────────────

// mc02 的 SOF 硬件捕获在 TIM5 的 ITR7 上（RM0468 Table 94 / 355）。2026-09-07 的一次扫描记录
// 说 TIM5 没有 SOF，与手册矛盾：这一条就是裁决。全速 USB 每毫秒一个 SOF，每一个都应当新鲜。
TEST_F(Mc02Bench, TheMc02LatchesEverySofInHardware) {
    TimedMc02 mc02{serial_mc02_};
    ASSERT_GT(mc02.listener.wait_valid(4, 6s), 0U)
        << "the mc02 accepted the time base but never reported a valid time line";
    std::this_thread::sleep_for(2s);

    const std::scoped_lock lock{mc02.listener.mutex};
    const auto fresh = mc02.listener.capture_fresh;
    const auto stale = mc02.listener.capture_stale;
    std::printf(
        "    mc02 hardware SOF captures: %llu fresh, %llu stale over %zu reports\n",
        static_cast<unsigned long long>(fresh), static_cast<unsigned long long>(stale),
        mc02.listener.reports);
    ASSERT_GT(fresh, 0U) << "TIM5 ITR7 latched no SOF (or the TIM5/TIM3 .ioc settings are off)";
    EXPECT_GT(static_cast<double>(fresh), 0.99 * static_cast<double>(fresh + stale));
}

// mc02 收到的帧（5321 发）换到主机时钟后，落在"主机交给 5321 发"与"主机从 mc02 收到"之间。
// 走的是 TIM3（FDCAN 外部时间戳）-> TIM5 -> SOF 环这一整条路。
TEST_F(Mc02Bench, TheMc02PlacesReceivedFramesOnTheHostClock) {
    TimedHpm hpm{serial_hpm_};
    TimedMc02 mc02{serial_mc02_};
    ASSERT_GT(mc02.listener.wait_valid(4, 6s), 0U) << "the mc02 never reported a valid time line";
    ASSERT_GT(hpm.listener.wait_valid(4, 6s), 0U) << "the 5321 never reported a valid time line";
    const auto tolerance = 300us;

    constexpr int frames = 200;
    int unstamped = 0;
    std::vector<double> after_send_us, before_arrival_us;
    for (int i = 0; i < frames; ++i) {
        const auto can_id = static_cast<std::uint32_t>(0x480 + (i % 0x40));
        const auto payload = pattern(8, static_cast<std::uint8_t>(i));
        const auto sent = std::chrono::steady_clock::now();
        hpm.send(can_id, payload);
        const auto arrival = mc02.listener.wait(can_id, kArrival);
        ASSERT_TRUE(arrival.has_value()) << "frame " << i << " never arrived";
        {
            const std::scoped_lock lock{mc02.listener.mutex};
            mc02.listener.arrivals.clear();
        }
        if (!arrival->stamp) {
            ++unstamped;
            continue;
        }
        ASSERT_EQ(arrival->time.source, libhcs::time::SampleTime::Source::kBoardStamp);
        const auto& started = arrival->time.host;
        EXPECT_GE(started, sent - tolerance) << "frame " << i << " stamped before it was sent";
        EXPECT_LE(started, arrival->at + tolerance)
            << "frame " << i << " stamped after the host received it";
        after_send_us.push_back(std::chrono::duration<double, std::micro>(started - sent).count());
        before_arrival_us.push_back(
            std::chrono::duration<double, std::micro>(arrival->at - started).count());
        std::this_thread::sleep_for(2ms);
    }
    ASSERT_FALSE(after_send_us.empty())
        << "the mc02 stamps no frame: its .ioc is not at TIM5 0/0xFFFFFFFF + TIM3 1/65535, or "
           "TIM5 ITR7 carries no SOF";
    EXPECT_EQ(unstamped, 0);

    std::ranges::sort(after_send_us);
    std::ranges::sort(before_arrival_us);
    // 帧在线上（FD 1M/5M 8 字节约 50 us）加 USB 全速上行：比这还短，说明时间戳偏晚。
    EXPECT_GT(before_arrival_us.front(), 40.0);
    std::printf(
        "    sample time: converted from the board stamp (SampleTime::kBoardStamp)\n"
        "    frame start after the host's send call: min %.1f / median %.1f / max %.1f us\n"
        "    host receive after the frame start:     min %.1f / median %.1f / max %.1f us\n"
        "    %d of %d frames unstamped\n",
        after_send_us.front(), after_send_us[after_send_us.size() / 2], after_send_us.back(),
        before_arrival_us.front(), before_arrival_us[before_arrival_us.size() / 2],
        before_arrival_us.back(), unstamped, frames);
}

// ── CBoardBench：5321 + mc02 + c_board，CAN1 三板共线，c_board UART1 对 mc02 的一个串口 ──

/// mc02 上可能接着 c_board UART1 的 TTL 串口（UART2 / UART3 是 RS-485 收发器，不算）。
constexpr std::array kMc02TtlUarts{DataId::kUart1, DataId::kUart7, DataId::kUart10};

class CBoardBench : public testing::Test {
protected:
    void SetUp() override {
        const auto boards = cboard_bench();
        if (!boards)
            GTEST_SKIP() << "needs exactly one dual-CAN HPM5321, one mc02 and one c_board with "
                            "CAN1 on one bus and the c_board's UART1 crossed to an mc02 UART; found "
                         << attached_boards().size() << " HPM5321, " << attached_mc02().size()
                         << " mc02, " << attached_cboard().size() << " c_board";
        serial_hpm_ = (*boards)[0];
        serial_mc02_ = (*boards)[1];
        serial_cboard_ = (*boards)[2];
    }

    [[nodiscard]] std::unique_ptr<Node> open_hpm(const Configuration& configuration) const {
        return std::make_unique<Node>(serial_hpm_, configuration);
    }
    [[nodiscard]] std::unique_ptr<Mc02Node> open_mc02(const Configuration& configuration) const {
        return std::make_unique<Mc02Node>(serial_mc02_, configuration);
    }
    [[nodiscard]] std::unique_ptr<CBoardNode> open_cboard(const Configuration& configuration) const {
        return std::make_unique<CBoardNode>(serial_cboard_, configuration);
    }

    /// 哪个 mc02 串口接着 c_board 的 UART1：c_board 发一段，mc02 的 TTL 串口全开着听。
    /// 探一次，整组用例共用。
    std::optional<DataId> mc02_uart() {
        static std::optional<std::optional<DataId>> found;
        if (found)
            return *found;
        Configuration all;
        for (const DataId port : kMc02TtlUarts)
            all.declare_uart(port, uart_8n1(kUartBaudrate));
        const auto mc02 = open_mc02(all);
        const auto cboard = open_cboard(declare({}, DataId::kUart1));
        std::this_thread::sleep_for(kSettle);
        const auto probe = pattern(64, 0x42);
        cboard->send_uart(probe);
        std::this_thread::sleep_for(kSilence);
        found = std::optional<DataId>{};
        for (const DataId port : kMc02TtlUarts) {
            if (mc02->listener.bytes_on(port) == probe) {
                *found = port;
                std::printf("    c_board UART1 is wired to the mc02's %s\n",
                            port == DataId::kUart1   ? "UART1"
                            : port == DataId::kUart7 ? "UART7"
                                                     : "UART10");
            }
        }
        return *found;
    }

    std::string serial_hpm_, serial_mc02_, serial_cboard_;
};

/// 三块板 CAN1 的推送状态，帧没到时附在失败信息后面。
std::string can1_report(Node* hpm, Mc02Node* mc02, CBoardNode* cboard) {
    const auto line = [](const char* who, const libhcs::data::CanStatusView& status) {
        return std::format(
            "\n  {}: tec={} rec={} last_error={} data_last_error={} flags={:#04x} "
            "tx_cancelled={} rx_lost={}",
            who, status.tec, status.rec, libhcs::data::last_error_name(status.last_error),
            libhcs::data::last_error_name(status.data_last_error), status.flags,
            status.tx_cancelled, status.rx_lost);
    };
    std::string report;
    if (hpm)
        report += line("5321", hpm->board.handle(libhcs::spec::hpm5321::Spec::Cans::kCan1).status());
    if (mc02)
        report += line("mc02", mc02->board.handle(libhcs::spec::mc02::Spec::Cans::kCan1).status());
    if (cboard)
        report += line(
            "c_board", cboard->board.handle(libhcs::spec::c_board::Spec::Cans::kCan1).status());
    return report;
}

// 握手：c_board 是两路 CAN、三个串口；声明了的口报已声明，经典 CAN 报的就是经典。
TEST_F(CBoardBench, HandshakeAgreesOnTheInterface) {
    const auto cboard = open_cboard(declare({DataId::kCan1}, DataId::kUart1));
    const auto interface = cboard->board.interface();
    EXPECT_TRUE(interface.declared(DataId::kCan1));
    EXPECT_FALSE(interface.declared(DataId::kCan2)) << "the wiring did not declare it";
    EXPECT_FALSE(interface.can_fd(DataId::kCan1)) << "bxCAN has no FD";
    EXPECT_TRUE(interface.declared(DataId::kUart1));
    EXPECT_EQ(cboard->board.link_state(), libhcs::host::protocol::Handler::LinkState::kUp);
}

// CAN1 上三块板都声明经典 1M：每块发的帧另外两块都收到，内容不变。
TEST_F(CBoardBench, Can1CarriesClassicFramesBetweenAllThree) {
    const auto hpm = open_hpm(declare({DataId::kCan1}));
    const auto mc02 = open_mc02(declare({DataId::kCan1}));
    const auto cboard = open_cboard(declare({DataId::kCan1}));
    std::this_thread::sleep_for(kSettle);

    const auto expect = [&](const char* from, Inbox& a, const char* to_a, Inbox& b,
                            const char* to_b, std::uint32_t can_id, const std::vector<std::byte>& data) {
        for (auto [inbox, to] : {std::pair{&a, to_a}, std::pair{&b, to_b}}) {
            const auto frame = inbox->wait_frame(DataId::kCan1, can_id, kArrival);
            ASSERT_TRUE(frame.has_value()) << from << " -> " << to << " never arrived"
                                           << can1_report(hpm.get(), mc02.get(), cboard.get());
            EXPECT_EQ(frame->data, data) << from << " -> " << to;
        }
    };
    const auto from_hpm = pattern(8, 0x10);
    hpm->send(DataId::kCan1, 0x301, from_hpm);
    expect("5321", mc02->listener, "mc02", cboard->listener, "c_board", 0x301, from_hpm);
    const auto from_mc02 = pattern(8, 0x20);
    mc02->send(DataId::kCan1, 0x302, from_mc02);
    expect("mc02", hpm->listener, "5321", cboard->listener, "c_board", 0x302, from_mc02);
    const auto from_cboard = pattern(8, 0x30);
    cboard->send(DataId::kCan1, 0x303, from_cboard);
    expect("c_board", hpm->listener, "5321", mc02->listener, "mc02", 0x303, from_cboard);
}

// c_board 没声明 CAN1，它的 bxCAN 就停在 INIT 模式、不在总线上：5321 一个人在 CAN1 上发经典帧
// （mc02 也没声明 CAN1），没人应答——5321 记下 ACK 错误，c_board 一帧也收不到。c_board 若还在
// 总线上，它会应答（也会把同一条线上的 CAN-FD 帧当格式错误打掉）。
//
// 不拿 5321↔mc02 的 CAN-FD 来测：三块板同挂这条线时，FD 帧即使 c_board 离线也过不去，错误全在
// 5 Mbit 的数据段（仲裁段干净），是线上的问题（三个终端并联、分支），不是固件。[实测 2026-10-07]
TEST_F(CBoardBench, AnUndeclaredCBoardStaysOffTheBus) {
    const auto hpm = open_hpm(declare({DataId::kCan1}));
    const auto mc02 = open_mc02(declare({DataId::kCan2}));
    const auto cboard = open_cboard(declare({DataId::kCan2}));
    std::this_thread::sleep_for(kSettle);

    const auto payload = pattern(8, 0x51);
    for (std::uint32_t i = 0; i < 4; ++i) {
        hpm->send(DataId::kCan1, 0x311 + i, payload);
        std::this_thread::sleep_for(5ms);
    }
    const auto can1 = hpm->board.handle(libhcs::spec::hpm5321::Spec::Cans::kCan1);
    const auto status = wait_status([&] { return can1.status(); }, [](const auto& s) {
        return s.tec > 0;
    });
    EXPECT_GT(status.tec, 0) << "somebody acknowledged frames on a bus nobody else declared"
                             << can1_report(hpm.get(), nullptr, nullptr);
    EXPECT_EQ(status.last_error, libhcs::data::CanLastError::kAck)
        << can1_report(hpm.get(), nullptr, nullptr);
    std::this_thread::sleep_for(kSilence);
    EXPECT_EQ(cboard->listener.frames_on(DataId::kCan1), 0U) << "an undeclared bus delivered";
}

// c_board 一个人在 CAN1 上（另两块板开着会话、但没声明 CAN1，不在总线上）：没人应答，
// 推送的状态里 TEC 涨、错误码是 ACK。经典 CAN 的 ACK 记在 last_error；这是 CAN_SCE 错误
// 中断锁存的那一位。
TEST_F(CBoardBench, TheCBoardPushesAckErrorsWhenAloneOnTheBus) {
    const auto hpm = open_hpm(declare({DataId::kCan2}));
    const auto mc02 = open_mc02(declare({DataId::kCan2}));
    const auto cboard = open_cboard(declare({DataId::kCan1}));
    std::this_thread::sleep_for(kSettle);

    const auto payload = pattern(8, 0x61);
    for (std::uint32_t i = 0; i < 4; ++i) {
        cboard->send(DataId::kCan1, 0x320 + i, payload);
        std::this_thread::sleep_for(5ms);
    }
    const auto can1 = cboard->board.handle(libhcs::spec::c_board::Spec::Cans::kCan1);
    const auto status = wait_status([&] { return can1.status(); }, [](const auto& s) {
        return s.tec > 0;
    });
    EXPECT_GT(status.tec, 0) << "somebody acknowledged" << can1_report(nullptr, nullptr, cboard.get());
    EXPECT_EQ(status.last_error, libhcs::data::CanLastError::kAck)
        << can1_report(nullptr, nullptr, cboard.get());
    std::this_thread::sleep_for(kSilence);
    EXPECT_EQ(hpm->listener.frames_on(DataId::kCan1), 0U);
    EXPECT_EQ(mc02->listener.frames_on(DataId::kCan1), 0U);
}

// 不经板卡类的主机往 c_board 没声明的 CAN1 发：板子收到就丢，线上什么都没有。
TEST_F(CBoardBench, TheCBoardDropsWhatAnUncheckedHostSendsToAnUndeclaredBus) {
    const auto hpm = open_hpm(declare({DataId::kCan1}));
    auto cboard = raw_cboard(serial_cboard_, declare({DataId::kCan2}));
    std::this_thread::sleep_for(kSettle);
    EXPECT_TRUE(cboard.send_can(DataId::kCan1, 0x331, pattern(8, 0x71)))
        << "the protocol layer takes it; the board is the one that must drop it";
    std::this_thread::sleep_for(kSilence);
    EXPECT_EQ(hpm->listener.frames_on(DataId::kCan1), 0U) << "an undeclared bus transmitted";
}

// c_board UART1 与 mc02 的串口：两边都声明了才通，两个方向字节都原样到。
TEST_F(CBoardBench, TheCBoardUartCarriesBytesBothWays) {
    const auto port = mc02_uart();
    ASSERT_TRUE(port.has_value()) << "none of the mc02's UART1/7/10 hears the c_board's UART1";
    const auto mc02 = open_mc02(declare({}, *port));
    const auto cboard = open_cboard(declare({}, DataId::kUart1));
    std::this_thread::sleep_for(kSettle);

    const auto there = pattern(200, 0x13);
    cboard->send_uart(there);
    std::this_thread::sleep_for(kSilence);
    EXPECT_EQ(mc02->listener.bytes_on(*port), there) << "c_board -> mc02";

    const auto back = pattern(200, 0x37);
    mc02->send_uart(*port, back);
    EXPECT_EQ(cboard->listener.wait_bytes(back.size(), kArrival), back) << "mc02 -> c_board";
}

// c_board 没声明 UART1：对面发来的字节不转发，不经板卡类的主机写给它的也不发出去。
TEST_F(CBoardBench, AnUndeclaredCBoardUartForwardsNothing) {
    const auto port = mc02_uart();
    ASSERT_TRUE(port.has_value()) << "none of the mc02's UART1/7/10 hears the c_board's UART1";
    const auto mc02 = open_mc02(declare({}, *port));
    {
        const auto cboard = open_cboard(declare({DataId::kCan1}));
        std::this_thread::sleep_for(kSettle);
        mc02->send_uart(*port, pattern(64, 0x21));
        std::this_thread::sleep_for(kSilence);
        EXPECT_TRUE(cboard->listener.bytes_on(DataId::kUart1).empty())
            << "an undeclared UART forwarded bytes";
    }
    {
        auto cboard = raw_cboard(serial_cboard_, declare({DataId::kCan1}));
        std::this_thread::sleep_for(kSettle);
        EXPECT_TRUE(cboard.send_uart(DataId::kUart1, pattern(64, 0x22)));
        std::this_thread::sleep_for(kSilence);
        EXPECT_TRUE(mc02->listener.bytes_on(*port).empty()) << "an undeclared UART transmitted";
    }
}

// 速率不符时 c_board 的接收机满是错误，推送的串口状态里有；对上之后干净的字节不再添错误。
TEST_F(CBoardBench, TheCBoardPushesUartReceiveErrors) {
    const auto port = mc02_uart();
    ASSERT_TRUE(port.has_value()) << "none of the mc02's UART1/7/10 hears the c_board's UART1";
    Configuration slow;
    slow.declare_uart(*port, uart_8n1(115'200));
    const auto mc02 = open_mc02(slow);
    const auto cboard = open_cboard(declare({}, DataId::kUart1));
    std::this_thread::sleep_for(kSettle);
    auto uart = cboard->board.handle(libhcs::spec::c_board::Spec::Uarts::kUart1);
    const auto errors = [](const libhcs::data::UartStatusView& s) {
        return s.overrun + s.parity + s.framing + s.noise + s.unattributed;
    };

    const auto before = errors(uart.status());
    mc02->send_uart(*port, pattern(256, 0xA5));
    const auto after = errors(wait_status([&] { return uart.status(); }, [&](const auto& s) {
        return errors(s) > before;
    }));
    EXPECT_GT(after, before) << "the c_board saw garbage but reported no receive error";

    mc02->board.handle(libhcs::spec::mc02::Spec::Uart{*port}).configure(uart_8n1(kUartBaudrate));
    std::this_thread::sleep_for(kSilence);
    const auto settled = errors(uart.status());
    cboard->listener.clear_bytes();
    const auto clean = pattern(96, 0x3C);
    mc02->send_uart(*port, clean);
    (void)cboard->listener.wait_bytes(clean.size(), kArrival);
    std::this_thread::sleep_for(kSilence);
    const auto all = cboard->listener.bytes_on(DataId::kUart1);
    const auto mismatch = std::ranges::mismatch(all, clean);
    EXPECT_EQ(all, clean) << "received " << all.size() << " bytes, first difference at "
                          << (mismatch.in1 - all.begin());
    EXPECT_EQ(errors(uart.status()), settled) << "clean bytes added receive errors";
}

// c_board 的板载 IMU：没声明一个样本都没有；声明了按声明来，静止时加速度模长约 1 g。
TEST_F(CBoardBench, TheCBoardImuStreamsOnlyWhenDeclared) {
    constexpr auto kWindow = 500ms;
    {
        const auto cboard = open_cboard(declare({DataId::kCan1}));
        std::this_thread::sleep_for(kWindow);
        const auto samples = cboard->listener.imu_samples();
        EXPECT_EQ(samples[0], 0U) << "accelerometer samples from an IMU nobody declared";
        EXPECT_EQ(samples[1], 0U) << "gyroscope samples from an IMU nobody declared";
    }
    Configuration with_imu = declare({DataId::kCan1});
    with_imu.declare_imu(
        ImuSetting{
            .accelerometer_range_g = 6,
            .accelerometer_rate_hz = 1600,
            .gyroscope_range_dps = 2000,
            .gyroscope_rate_hz = 2000,
            .gyroscope_bandwidth_hz = 230});
    const auto cboard = open_cboard(with_imu);
    std::this_thread::sleep_for(kWindow);
    const auto samples = cboard->listener.imu_samples();
    // 0.5 s 窗口：标称 800 / 1000 个，留出握手与首个样本的余量。
    EXPECT_GT(samples[0], 600U) << "accelerometer";
    EXPECT_GT(samples[1], 750U) << "gyroscope";
    // 6 g 量程下 1 g = 32768 / 6 ≈ 5461 LSB。
    const double rms = cboard->listener.accelerometer_rms();
    EXPECT_NEAR(rms, 32768.0 / 6.0, 32768.0 / 6.0 * 0.15) << "the board is not at rest, or the range is wrong";
}

// c_board 的共享时间基准（10-06 自 mc02 移植，此前没上过板）：只在声明时运行；声明了几轮保活
// 之内进入 valid，拟合出的每微帧 tick 数就是标称 500 quarter-us（168 MHz 周期计数器换算，
// 晶振偏差几十 ppm），每轮的样本外预测误差在微秒以内。本板没有 SOF 硬件捕获、CAN 帧不带
// 时间戳，跨板的帧时间戳比较只能在 5321 / mc02 之间做（Mc02Bench / ThreeBoardBench）。
TEST_F(CBoardBench, TheCBoardTimeBaseRunsOnlyWhileDeclaredAndFits) {
    struct Listener final : CBoard::Callback {
        void time_status_callback(const libhcs::data::TimeStatusView& data) override {
            const std::scoped_lock lock{mutex};
            ++reports;
            if (data.state == libhcs::data::TimeState::kValid) {
                ++valid;
                last_valid = data;
            }
        }
        std::mutex mutex;
        std::size_t reports = 0, valid = 0;
        std::optional<libhcs::data::TimeStatusView> last_valid;
    };
    const auto open = [&](Listener& listener, bool time_sync) {
        Configuration declaration = declare({DataId::kCan1});
        declaration.enable_time_sync(time_sync);
        return std::make_unique<CBoard>(
            listener, serial_cboard_,
            libhcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true), declaration);
    };
    {
        Listener listener;
        const auto cboard = open(listener, false);
        std::this_thread::sleep_for(1500ms);
        const std::scoped_lock lock{listener.mutex};
        EXPECT_EQ(listener.reports, 0U) << "time status from a board that was not asked for it";
    }
    Listener listener;
    const auto cboard = open(listener, true);
    for (int i = 0; i < 80; ++i) {
        {
            const std::scoped_lock lock{listener.mutex};
            if (listener.valid >= 4)
                break;
        }
        std::this_thread::sleep_for(100ms);
    }
    const std::scoped_lock lock{listener.mutex};
    ASSERT_GE(listener.valid, 4U) << listener.reports << " reports, none valid enough";
    const auto& status = *listener.last_valid;
    constexpr double kNominal = 500.0 * 65536.0;
    const double ppm = (status.ticks_per_microframe_q16 - kNominal) / kNominal * 1e6;
    const double residual_mean_ns = status.residual_mean_q16 / 65536.0 * 250.0;
    const double residual_max_ns = status.residual_abs_max_q16 / 65536.0 * 250.0;
    std::printf(
        "    c_board time base: rate %+.1f ppm, residual mean %.1f ns / max %.1f ns over %u "
        "samples, %u anomalies\n",
        ppm, residual_mean_ns, residual_max_ns, status.residual_count, status.anomaly_count);
    EXPECT_LT(std::abs(ppm), 500.0) << "not a crystal";
    EXPECT_LT(std::abs(residual_mean_ns), 1000.0);
    EXPECT_LT(residual_max_ns, 10'000.0);
}

} // namespace
