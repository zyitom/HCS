// 板组件（hardware/board）：设备登记与分发、撞号与总线速率检查、EP0 配置、共用发送线程
// 按板的陈旧判断、安全锁存组件；端口包装层（跨线程交接、掉线计数、被拒帧计数）；
// 以及真实的平衡步兵三块板全禁用接进组件图空跑。
// 假的 libhcs 板卡类（FakeSdk，接口与 libhcs::board::Hpm5321 相同）套进真正的 Board<Sdk>：
// 测试经 libhcs 的通用回调入口注入帧，发送线程发出的帧交回测试。

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <libhcs/board/hpm5321.hpp>
#include <libhcs/board/mc02.hpp>
#include <rclcpp/rclcpp.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_executor/wiring.hpp>
#include <hcs_msgs/switch.hpp>

#include "hardware/board/board.hpp"
#include "hardware/board/devices.hpp"

#include "hipnuc_test_frames.hpp"

// 组件本体：真实接线表（匿名命名空间里的板在本 TU 可见）、发送线程、安全锁存。
#include "hardware/balance_infantry.cpp"
#include "hardware/safety_latch.cpp"
#include "hardware/transmit_thread.cpp"

namespace {

using namespace hcs_core::hardware::board; // NOLINT(google-build-using-namespace)
using hcs_core::hardware::util::BoardFrame;
using hcs_executor::Component;
using hcs_executor::Linker;
using libhcs::data::DataId;

using DmMode = hcs_core::hardware::device::DmMotor::ControlMode;
using LkMode = hcs_core::hardware::device::LkMotor::ControlMode;
using LkType = hcs_core::hardware::device::LkMotor::Type;
using DjiType = hcs_core::hardware::device::DjiMotor::Type;
constexpr auto kJ4310 = hcs_core::hardware::device::DmMotor::kJ4310Factory;

// 必填字段没有默认值：漏写不编译（这里用"不可默认构造"间接断言）。
static_assert(!std::is_default_constructible_v<hcs_core::hardware::device::DmMotor::Config>);
static_assert(!std::is_default_constructible_v<hcs_core::hardware::device::LkMotor::Config>);
static_assert(!std::is_default_constructible_v<hcs_core::hardware::device::DjiMotor::Config>);
static_assert(!std::is_default_constructible_v<hcs_core::hardware::device::Hipnuc::Config>);

// 端口属于哪个板型由描述符的类型决定：把 mc02 的 CAN 描述符交给 5321 的板，编译不过。
static_assert(std::is_constructible_v<
              CanBus, Board<libhcs::board::Hpm5321>&,
              const libhcs::board::Hpm5321::Callback::Spec::Can&,
              const libhcs::board::hcs::CanSetting&>);
static_assert(!std::is_constructible_v<
              CanBus, Board<libhcs::board::Hpm5321>&,
              const libhcs::board::Mc02::Callback::Spec::Can&,
              const libhcs::board::hcs::CanSetting&>);

// ── 假 SDK ───────────────────────────────────────────────────────────────

/// 与 libhcs::board::Hpm5321 同形的假板卡：回调、描述符都用 Hpm5321 的，不碰 USB。
class FakeSdk {
public:
    using Callback = libhcs::board::Hpm5321::Callback;

    FakeSdk(
        Callback& callback, std::string_view, const libhcs::board::AdvancedOptions&,
        const libhcs::board::hcs::Configuration& configuration)
        : callback_(callback)
        , configuration_(configuration) {}

    [[nodiscard]] libhcs::host::protocol::Handler::LinkState link_state() const noexcept {
        return libhcs::host::protocol::Handler::LinkState::kUp;
    }
    [[nodiscard]] libhcs::board::hcs::Interface interface() const { return {.can_count = 2}; }
    [[nodiscard]] libhcs::board::hcs::vc::CanStatusPayload can_status(libhcs::board::hcs::CanPort) {
        return {};
    }

    /// 一次 start_transmit() = 一个发送缓冲：析构时交给测试。
    class PacketBuilder {
    public:
        explicit PacketBuilder(FakeSdk& sdk)
            : sdk_(sdk) {}
        PacketBuilder(const PacketBuilder&) = delete;
        PacketBuilder& operator=(const PacketBuilder&) = delete;
        ~PacketBuilder() { sdk_.commit(std::move(frames_)); }

        PacketBuilder& can_transmit(
            libhcs::board::hcs::CanPort port, const libhcs::data::CanDataView& data) {
            BoardFrame frame{.port = std::to_underlying(port), .can_id = data.can_id};
            std::ranges::copy(data.can_data, frame.data.begin());
            frames_.push_back(frame);
            return *this;
        }

    private:
        FakeSdk& sdk_;
        std::vector<BoardFrame> frames_;
    };
    PacketBuilder start_transmit() { return PacketBuilder{*this}; }

    /// 走 libhcs 的通用入口（DataId），与真实 IO 线程同一条路。
    void inject_can(DataId id, std::uint32_t can_id, std::array<std::uint8_t, 8> data) {
        callback_.can_receive_callback(
            id, libhcs::data::CanDataView{
                    .can_id = can_id, .can_data = std::as_bytes(std::span{data})});
    }

    /// 板载 IMU 的样本，同样走 libhcs 的通用回调接口。
    void inject_accelerometer(std::int16_t x, std::int16_t y, std::int16_t z, std::uint32_t time) {
        static_cast<libhcs::data::DataCallback&>(callback_).accelerometer_receive_callback(
            {.x = x, .y = y, .z = z, .timestamp_quarter_us = time});
    }
    void inject_gyroscope(std::int16_t x, std::int16_t y, std::int16_t z, std::uint32_t time) {
        static_cast<libhcs::data::DataCallback&>(callback_).gyroscope_receive_callback(
            {.x = x, .y = y, .z = z, .timestamp_quarter_us = time});
    }

    /// 等发送线程发完第 n 个缓冲，返回它。
    std::vector<BoardFrame> wait_packet(std::uint64_t n) {
        std::unique_lock lock{mutex_};
        EXPECT_TRUE(sent_.wait_for(lock, std::chrono::seconds{2}, [&] { return packets_ >= n; }))
            << "packet " << n << " never sent";
        return last_;
    }

    [[nodiscard]] std::uint64_t packets() {
        const std::scoped_lock lock{mutex_};
        return packets_;
    }

    [[nodiscard]] const libhcs::board::hcs::Configuration& configuration() const {
        return configuration_;
    }

private:
    void commit(std::vector<BoardFrame> frames) {
        const std::scoped_lock lock{mutex_};
        last_ = std::move(frames);
        ++packets_;
        sent_.notify_all();
    }

    Callback& callback_;
    libhcs::board::hcs::Configuration configuration_;
    std::mutex mutex_;
    std::condition_variable sent_;
    std::uint64_t packets_ = 0;
    std::vector<BoardFrame> last_;
};

// ── 测试用的车：两块板 ─────────────────────────────────────────────────

struct LeftWiring : Board<FakeSdk> {
    CanBus can1{*this, Spec::kCans.kCan1, kClassic1M};
    DmMotor joint{can1, "/left/joint", {
        .control_mode = DmMode::kTorque, .esc_id = 0x02, .master_id = 0x06, .mit_range = kJ4310,
    }};
    DjiMotor wheel{can1, "/left/wheel", {.motor_type = DjiType::kM3508, .id = 1}};
};
using Left = BoardComponent<LeftWiring>;

struct RightWiring : Board<FakeSdk> {
    CanBus can2{*this, Spec::kCans.kCan2, kClassic1M};
    SerialPort uart0{*this, Spec::kUarts.kUart0};
    ImuPort onboard_imu{*this};
    LkMotor yaw{can2, "/right/yaw", {
        .motor_type = LkType::kMG5010Ei10, .control_mode = LkMode::kTorque, .can_id = 0x145,
    }};
    Hipnuc imu{uart0, "/right/imu", {.baudrate = 921600}};
    Bmi088Ekf onboard{onboard_imu, "/right/onboard", {}};
};
using Right = BoardComponent<RightWiring>;

/// 测试替身：遥控拨杆、给轮子的恒定力矩（锁存与否在 DJI 帧上看得出来：锁存 = 0 电流）。
class Stand : public Component {
public:
    Stand() {
        register_output("/test/switch", rearm_switch, hcs_msgs::Switch::MIDDLE);
        register_output("/left/wheel/control_torque", torque_, 1.0);
    }
    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {}

    OutputInterface<hcs_msgs::Switch> rearm_switch;

private:
    OutputInterface<double> torque_;
};

/// 读 /hardware/safe 的探针。
class Probe : public Component {
public:
    Probe() { register_input("/hardware/safe", safe); }
    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {}

    InputInterface<bool> safe;
};

template <class T>
std::shared_ptr<T> make_component(const std::string& name) {
    const Component::NameScope scope{name};
    return std::make_shared<T>();
}

/// DM 反馈帧：D[0] 低 4 位是 ESC_ID，高 4 位是状态（1 = 使能）。
std::array<std::uint8_t, 8> dm_feedback(std::uint8_t esc_id) {
    return {static_cast<std::uint8_t>(0x10 | (esc_id & 0x0F)), 0x80, 0x00, 0x80, 0x08, 0x00, 30, 30};
}

/// 一张完整的图：发送线程 + 两块板 + 锁存 + 替身；像 executor 一样逐拍跑。
struct Rig {
    Rig() {
        tx = make_component<hcs_core::hardware::TransmitThread>("tx");
        left = make_component<Left>("left_board");
        right = make_component<Right>("right_board");
        latch = make_component<hcs_core::hardware::SafetyLatch>("test_latch");
        stand = make_component<Stand>("stand");
        probe = make_component<Probe>("probe");
        components = {tx,    left,  left->partner_components().at(0),
                      right, right->partner_components().at(0),
                      latch, stand, probe};
        auto linked = Linker::link(components);
        EXPECT_TRUE(linked.has_value()) << linked.error().message;
        wiring = std::move(*linked);
        for (const auto& component : components)
            component->before_updating();
        newly_failed.reserve(components.size());
    }

    /// 一拍：按依赖顺序 update（跳过已隔离的）→ 拍尾门铃。
    void tick() {
        for (const auto& entry : wiring.latches)
            entry.latch(entry.interface);
        const hcs_sync::Tick tick{
            .scheduled = hcs_sync::Clock::now(), .dt = std::chrono::milliseconds{1},
            .sequence = sequence++};
        for (auto* component : wiring.updating_order)
            if (!component->failed())
                component->update(tick);
        for (auto* doorbell : wiring.tick_end_doorbells)
            doorbell->ring();
    }

    /// tick() 并等两块板都发完这一拍。
    void tick_and_wait() {
        tick();
        left_frames = left->sdk()->wait_packet(sequence);
        right_frames = right->sdk()->wait_packet(sequence);
    }

    /// DJI 0x200 帧里 id 1 的槽位（大端 int16）。
    [[nodiscard]] int wheel_current() const {
        for (const auto& frame : left_frames)
            if (frame.can_id == 0x200)
                return static_cast<std::int16_t>(
                    (std::to_integer<int>(frame.data[0]) << 8) | std::to_integer<int>(frame.data[1]));
        ADD_FAILURE() << "no 0x200 frame";
        return 0;
    }

    void isolate(Component& component) {
        Linker::isolate(&component, newly_failed, failed_count);
    }

    std::shared_ptr<hcs_core::hardware::TransmitThread> tx;
    std::shared_ptr<Left> left;
    std::shared_ptr<Right> right;
    std::shared_ptr<hcs_core::hardware::SafetyLatch> latch;
    std::shared_ptr<Stand> stand;
    std::shared_ptr<Probe> probe;
    std::vector<std::shared_ptr<Component>> components;
    hcs_executor::Wiring wiring;
    std::vector<std::uint32_t> newly_failed;
    std::uint32_t failed_count = 0;
    std::uint64_t sequence = 0;
    std::vector<BoardFrame> left_frames, right_frames;
};

// ── 分发、打包、EP0 配置 ─────────────────────────────────────────────────

TEST(Board, FeedbackReachesOnlyTheDeviceOnThatBus) {
    Rig rig;
    // 同一个 id 打到另一块板上：左腿不能收到。
    rig.right->sdk()->inject_can(DataId::kCan2, 0x06, dm_feedback(0x02));
    rig.tick_and_wait();
    EXPECT_FALSE(rig.left->joint.received());

    rig.left->sdk()->inject_can(DataId::kCan1, 0x06, dm_feedback(0x02));
    rig.left->sdk()->inject_can(DataId::kCan1, 0x201, {});
    rig.tick_and_wait();
    EXPECT_TRUE(rig.left->joint.online());
    EXPECT_TRUE(rig.left->wheel.received());
    EXPECT_FALSE(rig.right->yaw.received());
}

// 板载 IMU：样本从 SDK 的回调进来，经板的 IMU 口到设备，下一拍由控制线程解算。
TEST(Board, OnboardImuSamplesReachTheDeviceThroughTheSdkCallback) {
    constexpr std::int16_t kOneG = 5461; // 32767 / 6
    Rig rig;
    EXPECT_EQ(rig.right->onboard_imu.label(), "right_board/imu");

    rig.right->sdk()->inject_accelerometer(0, 0, kOneG, 4000);
    rig.right->sdk()->inject_gyroscope(0, 0, 0, 6000);
    EXPECT_FALSE(rig.right->onboard.initialized()) << "solved on the IO thread";
    EXPECT_FALSE(rig.right->onboard.received());

    rig.tick_and_wait();
    EXPECT_TRUE(rig.right->onboard.initialized());
    EXPECT_TRUE(rig.right->onboard.received());
    EXPECT_TRUE(rig.right->onboard.online());
    ASSERT_TRUE(rig.right->onboard.snapshot().has_value());
    EXPECT_EQ(rig.right->onboard.snapshot()->timestamp.time_since_epoch().count(), 6000);

    // 另一块板没有声明 IMU 口：它的 SDK 照样会回调，样本无处可去，不出事。
    rig.left->sdk()->inject_gyroscope(1, 2, 3, 8000);
    rig.tick_and_wait();
}

TEST(Board, EachBoardSendsItsOwnFrames) {
    Rig rig;
    rig.tick_and_wait();

    ASSERT_EQ(rig.left_frames.size(), 2U); // DM 独占帧 + DJI 共享帧（排在最后）
    EXPECT_EQ(rig.left_frames[0].can_id, 0x02U);
    EXPECT_EQ(rig.left_frames[0].port, 1);
    EXPECT_EQ(rig.left_frames[1].can_id, 0x200U);
    ASSERT_EQ(rig.right_frames.size(), 1U);
    EXPECT_EQ(rig.right_frames[0].can_id, 0x145U);
    EXPECT_EQ(rig.right_frames[0].port, 2);
    EXPECT_NE(rig.wheel_current(), 0); // 未锁存：控制力矩照发
}

TEST(Board, Ep0ConfigurationIsExplicitAndComesFromTheWiring) {
    namespace vc = libhcs::board::hcs::vc;
    Rig rig;
    const auto& left = rig.left->sdk()->configuration();
    const auto& right = rig.right->sdk()->configuration();

    // CAN：声明了哪路就配哪路（下标 = 描述符表下标），帧型与速率就是声明的那个
    ASSERT_TRUE(left.can[0].has_value());
    EXPECT_FALSE(left.can[0]->fd);
    EXPECT_EQ(left.can[0]->arbitration_baudrate, 1'000'000U);
    EXPECT_FALSE(left.can[1].has_value());
    EXPECT_FALSE(right.can[0].has_value());
    EXPECT_TRUE(right.can[1].has_value());

    // 串口：设置来自设备（CH040 模块 flash 里的速率），帧格式逐项写明，不留"沿用固件"
    EXPECT_FALSE(left.uart[0].has_value());
    ASSERT_TRUE(right.uart[0].has_value());
    EXPECT_EQ(right.uart[0]->baudrate, 921600U);
    EXPECT_EQ(right.uart[0]->word_length, vc::kUartWordLength8);
    EXPECT_EQ(right.uart[0]->parity, vc::kUartParityNone);
    EXPECT_EQ(right.uart[0]->stop_bits, vc::kUartStopBits1);
}

// ── 安全锁存（跨板）──────────────────────────────────────────────────────

TEST(Board, CriticalDeviceLossLatchesUntilRearmed) {
    Rig rig;
    rig.left->sdk()->inject_can(DataId::kCan1, 0x06, dm_feedback(0x02));
    rig.tick_and_wait();
    ASSERT_TRUE(rig.left->joint.online());
    EXPECT_FALSE(*rig.probe->safe);
    EXPECT_NE(rig.wheel_current(), 0);

    // 反馈停了，看门狗走完 → 掉线 → 锁存，各板改发全失能帧
    for (int i = 0; i < 101; ++i)
        rig.tick_and_wait();
    ASSERT_FALSE(rig.left->joint.online());
    EXPECT_TRUE(*rig.probe->safe);
    EXPECT_EQ(rig.wheel_current(), 0);

    // 还不健康时拨 DOWN 再拨回来：不复位
    *rig.stand->rearm_switch = hcs_msgs::Switch::DOWN;
    rig.tick_and_wait();
    *rig.stand->rearm_switch = hcs_msgs::Switch::MIDDLE;
    rig.tick_and_wait();
    EXPECT_EQ(rig.wheel_current(), 0);

    // 恢复之后 DOWN → 离开 DOWN：解除
    rig.left->sdk()->inject_can(DataId::kCan1, 0x06, dm_feedback(0x02));
    *rig.stand->rearm_switch = hcs_msgs::Switch::DOWN;
    rig.tick_and_wait();
    *rig.stand->rearm_switch = hcs_msgs::Switch::MIDDLE;
    rig.tick_and_wait();
    EXPECT_FALSE(*rig.probe->safe);
    EXPECT_NE(rig.wheel_current(), 0);
}

// 板组件被隔离：它的 health 复位成"上过线、不在线"，锁存立刻生效（而不是当成从未上线放过）；
// 它自己的 Lane 变陈旧，发送线程只给它补发一次全失能，另一块板照常发。
TEST(Board, IsolatedBoardFailsSafe) {
    Rig rig;
    rig.tick_and_wait();
    EXPECT_FALSE(*rig.probe->safe);

    rig.isolate(*rig.right);
    const auto right_before = rig.right->sdk()->packets();

    // 右板不再 update：它的批次停在上一拍。锁存看到它的设备"掉线"，左板改发失能帧。
    rig.tick();
    rig.left_frames = rig.left->sdk()->wait_packet(rig.sequence);
    EXPECT_TRUE(*rig.probe->safe) << "an isolated board must read as a lost device";
    EXPECT_EQ(rig.wheel_current(), 0);

    // 陈旧超过 stale_after_ms：发送线程只给右板补发一次它自己的全失能批次
    std::this_thread::sleep_for(std::chrono::milliseconds{20});
    rig.tick();
    rig.left_frames = rig.left->sdk()->wait_packet(rig.sequence);
    const auto safe = rig.right->sdk()->wait_packet(right_before + 1);
    ASSERT_EQ(safe.size(), 1U);
    EXPECT_EQ(safe[0].can_id, 0x145U); // LK 失能（0 电流）

    rig.tick();
    rig.left_frames = rig.left->sdk()->wait_packet(rig.sequence);
    EXPECT_EQ(rig.right->sdk()->packets(), right_before + 1) << "safe batch is sent once";
}

// ── 端口包装层：Can<Driver> / Serial<Driver> ────────────────────────────
//
// 驱动只写协议，而且全部跑在控制线程上。所有设备共有的那几件事只在包装层写了一遍，
// 也就只在这里测一遍：
//   - 事件域（IO 线程）收到的帧 / 字节不在那条线程上解码，交到周期域才交给驱动；
//   - CAN 只留最新一帧，串口字节按序全留、放不下的丢弃并报出来；
//   - 驱动 accepts() 拒掉的帧不算反馈，只计数；
//   - 掉线计数、<名字>/online 与 <名字>/health。
// 帧走端口的 receive()（IO 线程进来的那个入口，含按 id 查表），拍走设备的 update_status()。

struct WrapperWiring : Board<FakeSdk> {
    CanBus can1{*this, Spec::kCans.kCan1, kClassic1M};
    SerialPort uart0{*this, Spec::kUarts.kUart0};
    LkMotor yaw{can1, "/w/yaw", {
        .motor_type = LkType::kMG5010Ei10, .control_mode = LkMode::kTorque, .can_id = 0x145,
        .offline_timeout = 3,
    }};
    DmMotor joint{can1, "/w/joint", {
        .control_mode = DmMode::kTorque, .esc_id = 0x02, .master_id = 0x06, .mit_range = kJ4310,
    }};
    Hipnuc imu{uart0, "/w/imu", {.baudrate = 921600, .offline_timeout = 3}};

    ImuPort onboard_imu{*this};
    Bmi088Ekf onboard{onboard_imu, "/w/onboard", Bmi088Ekf::Config{}.set_offline_timeout(3)};
};

/// 读设备健康输出的探针：别的组件（安全锁存）看到的就是这两个。
/// 这张图里没有安全锁存，板要读的 /hardware/safe 也由它顶上（恒为"不锁存"）。
class HealthProbe : public Component {
public:
    HealthProbe() {
        register_input("/w/yaw/online", yaw_online);
        register_input("/w/yaw/health", yaw_health);
        register_input("/w/imu/online", imu_online);
        register_input("/w/imu/health", imu_health);
        register_output("/hardware/safe", safe_, false);
    }
    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {}

    InputInterface<bool> yaw_online, imu_online;
    InputInterface<hcs_core::hardware::util::DeviceHealth> yaw_health, imu_health;

private:
    OutputInterface<bool> safe_;
};

using ImuKind = hcs_core::hardware::device::ImuSample::Kind;

/// 一块不打开的板（enabled 缺省为 false，不碰 SDK）加一个探针，接成图。
/// 发送线程只是为了让图接得上（板要读它的输出）：这里不发帧，也不跑它。
struct Wrapped {
    Wrapped() {
        static int instance = 0;
        const std::string suffix = std::to_string(instance++);
        tx = make_component<hcs_core::hardware::TransmitThread>("wrapper_tx_" + suffix);
        board = make_component<BoardComponent<WrapperWiring>>("wrapper_board_" + suffix);
        probe = make_component<HealthProbe>("wrapper_probe_" + suffix);
        const std::vector<std::shared_ptr<Component>> components{
            tx, board, board->partner_components().at(0), probe};
        const auto linked = Linker::link(components);
        EXPECT_TRUE(linked.has_value()) << linked.error().message;
    }

    /// 控制线程的一拍里设备这一段：BoardCore::update() 对每台设备做的就是这一句。
    void tick() {
        const hcs_sync::Tick tick{
            .scheduled = hcs_sync::Clock::now(), .dt = std::chrono::milliseconds{1},
            .sequence = sequence++};
        board->yaw.update_status(tick);
        board->joint.update_status(tick);
        board->imu.update_status(tick);
        board->onboard.update_status(tick);
    }

    /// IO 线程收到一帧 CAN。
    void can(std::uint32_t can_id, std::span<const std::uint8_t> data) {
        board->can1.receive(can_id, std::as_bytes(data));
    }
    /// IO 线程收到一段串口字节。
    void uart(std::span<const std::uint8_t> data) { board->uart0.receive(std::as_bytes(data)); }
    /// IO 线程收到板载 IMU 的一个样本。时间以毫秒给，换成板上的四分之一微秒。
    void accelerometer(double milliseconds) {
        board->onboard_imu.receive(
            {.kind = ImuKind::kAccelerometer, .x = 0, .y = 0, .z = 5461,
             .timestamp_quarter_us = static_cast<std::uint32_t>(milliseconds * 4000)});
    }
    void gyroscope(double milliseconds, std::int16_t z = 0) {
        board->onboard_imu.receive(
            {.kind = ImuKind::kGyroscope, .x = 0, .y = 0, .z = z,
             .timestamp_quarter_us = static_cast<std::uint32_t>(milliseconds * 4000)});
    }

    std::shared_ptr<hcs_core::hardware::TransmitThread> tx;
    std::shared_ptr<BoardComponent<WrapperWiring>> board;
    std::shared_ptr<HealthProbe> probe;
    std::uint64_t sequence = 0;
};

/// LK 回复帧：command 决定布局（0xA1 = 力矩控制的回复，按"状态 2"解）；编码器在最后两字节。
std::array<std::uint8_t, 8> lk_reply(std::uint8_t command, std::uint16_t encoder) {
    return {command, 30, 0, 0, 0, 0, static_cast<std::uint8_t>(encoder),
            static_cast<std::uint8_t>(encoder >> 8)};
}

TEST(DeviceWrapper, NothingIsOnlineBeforeTheFirstFrame) {
    Wrapped rig;
    for (int i = 0; i < 5; ++i)
        rig.tick();

    EXPECT_FALSE(rig.board->yaw.received());
    EXPECT_FALSE(rig.board->yaw.online());
    EXPECT_FALSE(rig.board->imu.received());
    EXPECT_FALSE(rig.board->imu.online());
    EXPECT_DOUBLE_EQ(rig.board->yaw.angle(), 0.0);
    EXPECT_EQ(rig.board->imu.frame_count(), 0u);

    EXPECT_FALSE(*rig.probe->yaw_online);
    EXPECT_FALSE(rig.probe->yaw_health->received); // "从未上线"，与"上过线又掉了"是两回事
    EXPECT_FALSE(rig.probe->yaw_health->online);
}

// 帧在 IO 线程上只是被放进通道；解码发生在控制线程取走它的那一拍。
TEST(DeviceWrapper, CanFrameIsDecodedOnTheControlThread) {
    Wrapped rig;
    const auto raw_before = rig.board->yaw.last_raw_angle();
    rig.can(0x145, lk_reply(0xA1, 1000));
    EXPECT_FALSE(rig.board->yaw.received());
    EXPECT_EQ(rig.board->yaw.last_raw_angle(), raw_before) << "decoded on the IO thread";

    rig.tick();
    EXPECT_TRUE(rig.board->yaw.received());
    EXPECT_TRUE(rig.board->yaw.online());
    EXPECT_EQ(rig.board->yaw.last_raw_angle(), 1000);
    EXPECT_TRUE(*rig.probe->yaw_online);
    EXPECT_TRUE(rig.probe->yaw_health->received);
    EXPECT_TRUE(rig.probe->yaw_health->online);
    EXPECT_FALSE(rig.probe->yaw_health->faulted);
}

// 一拍之内到了几帧，驱动只看到最新那一帧：反馈是状态，不是事件流。
TEST(DeviceWrapper, OnlyTheLatestCanFrameSurvivesBetweenTicks) {
    Wrapped rig;
    rig.can(0x145, lk_reply(0xA1, 1000));
    rig.can(0x145, lk_reply(0xA1, 2000));
    rig.can(0x145, lk_reply(0xA1, 3000));
    rig.tick();
    EXPECT_EQ(rig.board->yaw.last_raw_angle(), 3000);

    // 没有新帧的一拍不会把旧帧再解一遍（多圈计数那种带状态的解码会被它带偏）。
    const double angle = rig.board->yaw.angle();
    rig.tick();
    EXPECT_DOUBLE_EQ(rig.board->yaw.angle(), angle);
}

// 掉线按拍计：连续 offline_timeout（这里是 3）拍没有有效反馈才算掉线，来一帧就回来。
TEST(DeviceWrapper, CanDeviceGoesOfflineAfterItsTimeoutAndComesBack) {
    Wrapped rig;
    rig.can(0x145, lk_reply(0xA1, 0));
    rig.tick(); // 新帧：计数装满（3）
    EXPECT_TRUE(rig.board->yaw.online());

    rig.tick(); // 2
    rig.tick(); // 1
    EXPECT_TRUE(rig.board->yaw.online());
    EXPECT_TRUE(*rig.probe->yaw_online);
    rig.tick(); // 0
    EXPECT_FALSE(rig.board->yaw.online());
    EXPECT_TRUE(rig.board->yaw.received()) << "掉线不清除“曾经收到过”";
    EXPECT_FALSE(*rig.probe->yaw_online);
    EXPECT_TRUE(rig.probe->yaw_health->received);
    EXPECT_FALSE(rig.probe->yaw_health->online);

    rig.can(0x145, lk_reply(0xA1, 0));
    rig.tick();
    EXPECT_TRUE(rig.board->yaw.online());
    EXPECT_TRUE(*rig.probe->yaw_online);
}

// 驱动不认的帧（同 id 上别的回复、别的电机）不算反馈：不解码、不续命，只计数，由尽力域报出来。
TEST(DeviceWrapper, FramesTheDriverRejectsAreCountedAndAreNotFeedback) {
    Wrapped rig;

    // LK：同 id 上布局不同的回复（0x9A 读状态 1）。编码器字段填满，被当成状态解的话角度会跳。
    for (int i = 0; i < 3; ++i)
        rig.can(0x145, lk_reply(0x9A, 0xFFFF));
    rig.tick();
    EXPECT_EQ(rig.board->yaw.rejected_frames(), 3u);
    EXPECT_FALSE(rig.board->yaw.received());
    EXPECT_FALSE(rig.board->yaw.online());
    EXPECT_DOUBLE_EQ(rig.board->yaw.angle(), 0.0);

    // 在线的设备也一样：被拒的帧不续掉线计数。
    rig.can(0x145, lk_reply(0xA1, 100));
    rig.tick();
    ASSERT_TRUE(rig.board->yaw.online());
    for (int i = 0; i < 3; ++i) {
        rig.can(0x145, lk_reply(0x9A, 0xFFFF));
        rig.tick();
    }
    EXPECT_FALSE(rig.board->yaw.online());
    EXPECT_EQ(rig.board->yaw.last_raw_angle(), 100);

    // DM：两台电机的 MST_ID 刷成了一样，另一台（ESC 0x05）的反馈落到这台的 id 上。
    EXPECT_FALSE(rig.board->joint.take_problem().has_value());
    rig.can(0x06, dm_feedback(0x05));
    rig.can(0x06, dm_feedback(0x05));
    rig.tick();
    EXPECT_FALSE(rig.board->joint.received());
    const auto problem = rig.board->joint.take_problem();
    ASSERT_TRUE(problem.has_value());
    EXPECT_NE(problem->find("2 frames on feedback id 0x6"), std::string::npos) << *problem;
    EXPECT_FALSE(rig.board->joint.take_problem().has_value()) << "already reported";

    // 长度不对的帧在进通道之前就被扔掉，既不算反馈也不算"驱动拒掉的"。
    const std::array<std::uint8_t, 7> truncated{0x12};
    rig.can(0x06, truncated);
    rig.tick();
    EXPECT_FALSE(rig.board->joint.received());
    EXPECT_FALSE(rig.board->joint.take_problem().has_value());
}

// 串口字节在 IO 线程上原样搬进通道，不解析；控制线程按到达顺序喂给驱动，帧边界不要求对齐。
TEST(DeviceWrapper, SerialBytesAreParsedOnTheControlThreadInArrivalOrder) {
    Wrapped rig;
    const auto frame = std::span<const std::uint8_t>{hipnuc_test::kManualFrame};
    rig.uart(frame.first(1));
    rig.uart(frame.subspan(1, 40));
    rig.uart(frame.subspan(41));
    EXPECT_EQ(rig.board->imu.frame_count(), 0u) << "parsed on the IO thread";
    EXPECT_FALSE(rig.board->imu.received());

    rig.tick();
    EXPECT_EQ(rig.board->imu.frame_count(), 1u);
    EXPECT_TRUE(rig.board->imu.received());
    EXPECT_TRUE(rig.board->imu.online());
    EXPECT_DOUBLE_EQ(rig.board->imu.temperature(), 35.0);
    EXPECT_TRUE(*rig.probe->imu_online);

    // 两拍之间到了两帧：都解，输出是后到的那一帧。
    rig.uart(hipnuc_test::make_frame(10));
    rig.uart(hipnuc_test::make_frame(20));
    rig.tick();
    EXPECT_EQ(rig.board->imu.frame_count(), 3u);
    EXPECT_DOUBLE_EQ(rig.board->imu.temperature(), 20.0);
}

// 掉线只认"解出了有效帧"：一直有字节进来、但都是坏帧，照样掉线。
TEST(DeviceWrapper, SerialDeviceGoesOfflineWithoutValidFramesAndComesBack) {
    Wrapped rig;
    rig.uart(hipnuc_test::kManualFrame);
    rig.tick(); // 新帧：计数装满（3）
    ASSERT_TRUE(rig.board->imu.online());

    hipnuc_test::Frame corrupted = hipnuc_test::kManualFrame;
    corrupted[hipnuc_test::kTemperatureOffset] ^= 0x01;
    for (int i = 0; i < 2; ++i) {
        rig.uart(corrupted);
        rig.tick();
    }
    EXPECT_TRUE(rig.board->imu.online());
    rig.uart(corrupted);
    rig.tick();
    EXPECT_FALSE(rig.board->imu.online());
    EXPECT_TRUE(rig.board->imu.received());
    EXPECT_GE(rig.board->imu.crc_error_count(), 3u);
    EXPECT_FALSE(*rig.probe->imu_online);
    EXPECT_TRUE(rig.probe->imu_health->received);

    rig.uart(hipnuc_test::make_frame(40));
    rig.tick();
    EXPECT_TRUE(rig.board->imu.online());
    EXPECT_DOUBLE_EQ(rig.board->imu.temperature(), 40.0);
}

// 控制线程来不及取的时候，通道放不下的字节被丢掉——但不是静默的：尽力域报出丢了多少。
TEST(DeviceWrapper, SerialOverrunIsReportedNotSilent) {
    Wrapped rig;
    EXPECT_FALSE(rig.board->imu.take_problem().has_value());

    // 30 帧共 2460 字节，通道只有 2048：最后 412 字节放不下。
    for (int i = 0; i < 30; ++i)
        rig.uart(hipnuc_test::make_frame(static_cast<std::int8_t>(i + 1)));

    const auto problem = rig.board->imu.take_problem();
    ASSERT_TRUE(problem.has_value());
    EXPECT_NE(problem->find("412 more byte(s) dropped, 412 in total"), std::string::npos)
        << *problem;
    EXPECT_FALSE(rig.board->imu.take_problem().has_value()) << "already reported";

    // 进了通道的那 24 个整帧一帧不少；第 25 帧只进来一截，不许被解成什么东西。
    rig.tick();
    EXPECT_EQ(rig.board->imu.frame_count(), 24u);
    EXPECT_DOUBLE_EQ(rig.board->imu.temperature(), 24.0);

    // 丢过字节之后流还能重新对齐：下一个整帧照常解出来。
    rig.uart(hipnuc_test::make_frame(77));
    rig.tick();
    EXPECT_DOUBLE_EQ(rig.board->imu.temperature(), 77.0);
}

// 板载 IMU 的样本在 IO 线程上只是进队列；控制线程按到达顺序交给驱动——顺序要紧：
// 加速度计样本先到才有时基，陀螺仪样本的时间戳才展得开。
TEST(DeviceWrapper, ImuSamplesAreSolvedOnTheControlThreadInArrivalOrder) {
    Wrapped rig;
    rig.accelerometer(1.0);
    rig.gyroscope(1.5);
    rig.gyroscope(2.0);
    EXPECT_FALSE(rig.board->onboard.initialized()) << "solved on the IO thread";

    rig.tick();
    EXPECT_TRUE(rig.board->onboard.received());
    EXPECT_TRUE(rig.board->onboard.online());
    ASSERT_TRUE(rig.board->onboard.snapshot().has_value());
    // 两个陀螺仪样本都用上了：姿态的时刻是后一个的。
    EXPECT_EQ(rig.board->onboard.snapshot()->timestamp.time_since_epoch().count(), 8000);
}

// 掉线只认"姿态前进了一步"：只有加速度计样本、没有陀螺仪样本，照样掉线。
TEST(DeviceWrapper, ImuGoesOfflineWithoutGyroscopeSamplesAndComesBack) {
    Wrapped rig;
    rig.accelerometer(1.0);
    rig.gyroscope(1.5);
    rig.tick(); // 计数装满（3）
    ASSERT_TRUE(rig.board->onboard.online());

    for (int i = 0; i < 2; ++i) {
        rig.accelerometer(2.0 + i);
        rig.tick();
    }
    EXPECT_TRUE(rig.board->onboard.online());
    rig.accelerometer(4.0);
    rig.tick();
    EXPECT_FALSE(rig.board->onboard.online());
    EXPECT_TRUE(rig.board->onboard.received());

    rig.gyroscope(4.5);
    rig.tick();
    EXPECT_TRUE(rig.board->onboard.online());
}

// 控制线程来不及取的时候，队列放不下的样本被丢掉——但不是静默的。
TEST(DeviceWrapper, ImuOverrunIsReportedNotSilent) {
    Wrapped rig;
    EXPECT_FALSE(rig.board->onboard.take_problem().has_value());

    rig.accelerometer(1.0);
    for (int i = 0; i < 400; ++i) // 队列 256 个
        rig.gyroscope(1.0 + 0.5 * (i + 1));

    const auto problem = rig.board->onboard.take_problem();
    ASSERT_TRUE(problem.has_value());
    EXPECT_NE(problem->find("more sample(s) dropped"), std::string::npos) << *problem;
    EXPECT_FALSE(rig.board->onboard.take_problem().has_value()) << "already reported";

    rig.tick(); // 进了队列的照常解算
    EXPECT_TRUE(rig.board->onboard.online());
}

// IO 线程不停地写，控制线程不停地取。每一帧的温度和气压由同一个 tag 生成，
// 只要驱动拿到过半新半旧、或者顺序乱掉的字节，两者就对不上。
//
// IO 线程每写 8 帧（656 字节）就等控制线程走过一拍再写：两条线程一直在同时碰通道，
// 但积压到不了通道的容量（2048 字节）——所以一个字节都不许丢，一帧都不许少。
// 不这样节流的话写的一方快得多，测到的只是"丢字节"（上一个测试），不是"交接"。
TEST(DeviceWrapper, ConcurrentSerialFeedIsNeverTornAndLosesNothing) {
    constexpr int kFrames = 50'000;
    constexpr int kBurst = 8;

    std::array<hipnuc_test::Frame, 100> frames;
    for (std::size_t tag = 0; tag < frames.size(); ++tag)
        frames[tag] = hipnuc_test::make_frame(static_cast<std::int8_t>(tag + 1));

    Wrapped rig;
    std::atomic<std::uint64_t> ticks{0};
    std::atomic<bool> done{false};
    std::thread io_thread{[&] {
        for (int i = 0; i < kFrames; ++i) {
            rig.uart(frames[static_cast<std::size_t>(i) % frames.size()]);
            if (i % kBurst == kBurst - 1) {
                const auto seen = ticks.load(std::memory_order_acquire);
                while (ticks.load(std::memory_order_acquire) == seen)
                    std::this_thread::yield();
            }
        }
        done.store(true, std::memory_order_release);
    }};

    std::uint64_t torn = 0;
    const auto check = [&] {
        rig.tick();
        ticks.fetch_add(1, std::memory_order_release);
        if (!rig.board->imu.received())
            return;
        const auto& imu = rig.board->imu;
        const bool whole = imu.air_pressure() == imu.temperature() * 100.0
                        && imu.temperature() >= 1.0 && imu.temperature() <= 100.0;
        if (!whole)
            ++torn;
    };
    while (!done.load(std::memory_order_acquire))
        check();
    io_thread.join();
    for (int i = 0; i < 4; ++i) // IO 线程收尾之后通道里剩下的也取完
        check();

    EXPECT_EQ(torn, 0u);
    EXPECT_FALSE(rig.board->imu.take_problem().has_value()) << "bytes were dropped";
    EXPECT_EQ(rig.board->imu.frame_count(), static_cast<std::uint32_t>(kFrames));
    EXPECT_EQ(rig.board->imu.crc_error_count(), 0u);
    EXPECT_DOUBLE_EQ(rig.board->imu.temperature(), ((kFrames - 1) % 100) + 1.0);
}

// ── 登记时的接线检查 ─────────────────────────────────────────────────────

/// 构造应当以 invalid_argument 失败，且原因是 reason（不是别的什么错）。
template <class Wiring>
void expect_rejected(std::string_view reason) {
    static int instance = 0;
    try {
        make_component<BoardComponent<Wiring>>("rejected_" + std::to_string(instance++));
        ADD_FAILURE() << "accepted; expected: " << reason;
    } catch (const std::invalid_argument& error) {
        EXPECT_NE(std::string_view{error.what()}.find(reason), std::string_view::npos)
            << error.what();
    }
}

struct SameFeedbackId : Board<FakeSdk> {
    CanBus can1{*this, Spec::kCans.kCan1, kClassic1M};
    DmMotor a{can1, "/a", {.control_mode = DmMode::kTorque, .esc_id = 0x01, .master_id = 0x11, .mit_range = kJ4310}};
    DmMotor b{can1, "/b", {.control_mode = DmMode::kTorque, .esc_id = 0x02, .master_id = 0x11, .mit_range = kJ4310}};
};

struct CommandIdIsAnotherFeedbackId : Board<FakeSdk> {
    CanBus can1{*this, Spec::kCans.kCan1, kClassic1M};
    DmMotor a{can1, "/a", {.control_mode = DmMode::kTorque, .esc_id = 0x01, .master_id = 0x11, .mit_range = kJ4310}};
    DmMotor b{can1, "/b", {.control_mode = DmMode::kTorque, .esc_id = 0x11, .master_id = 0x12, .mit_range = kJ4310}};
};

struct ExclusiveOnSharedCommandId : Board<FakeSdk> {
    CanBus can1{*this, Spec::kCans.kCan1, kClassic1M};
    DjiMotor wheel{can1, "/wheel", {.motor_type = DjiType::kM3508, .id = 1}};
    DmMotor dm{can1, "/dm", {.control_mode = DmMode::kTorque, .esc_id = 0x200, .master_id = 0x30, .mit_range = kJ4310}};
};

struct DjiSameSlot : Board<FakeSdk> {
    CanBus can1{*this, Spec::kCans.kCan1, kClassic1M};
    DjiMotor a{can1, "/a", {.motor_type = DjiType::kM3508, .id = 5}};         // 0x1FF 槽 0
    DjiMotor b{can1, "/b", {.motor_type = DjiType::kGM6020Voltage, .id = 1}}; // 0x1FF 槽 0
};

struct MotorOnAnFdBus : Board<FakeSdk> {
    CanBus can1{*this, Spec::kCans.kCan1, libhcs::board::hcs::kFd1M5M};
    DmMotor a{can1, "/a", {.control_mode = DmMode::kTorque, .esc_id = 0x01, .master_id = 0x11, .mit_range = kJ4310}};
};

struct PortDeclaredTwice : Board<FakeSdk> {
    CanBus can1{*this, Spec::kCans.kCan1, kClassic1M};
    CanBus again{*this, Spec::kCans.kCan1, kClassic1M};
};

struct TwoDevicesOnOneSerialPort : Board<FakeSdk> {
    SerialPort uart0{*this, Spec::kUarts.kUart0};
    Hipnuc a{uart0, "/a", {.baudrate = 921600}};
    Hipnuc b{uart0, "/b", {.baudrate = 921600}};
};

struct TwoSolversOnOneImu : Board<FakeSdk> {
    ImuPort imu{*this};
    Bmi088 a{imu, "/a", {}};
    Bmi088Ekf b{imu, "/b", {}};
};

struct ImuDeclaredTwice : Board<FakeSdk> {
    ImuPort imu{*this};
    ImuPort again{*this};
};

TEST(Board, RejectsWiringThatWouldFailSilently) {
    expect_rejected<SameFeedbackId>("/a and /b both report on feedback id 0x11");
    expect_rejected<CommandIdIsAnotherFeedbackId>("mix a command id with a feedback id");
    expect_rejected<ExclusiveOnSharedCommandId>("both command on id 0x200");
    // M3508 id 5 与电压模式 GM6020 id 1：反馈都在 0x205，指令都写 0x1FF 的槽 0
    expect_rejected<DjiSameSlot>("both report on feedback id 0x205");
    expect_rejected<MotorOnAnFdBus>("needs a classic CAN bus at 1000000 bit/s");
    expect_rejected<PortDeclaredTwice>("/can1 declared twice");
    expect_rejected<TwoDevicesOnOneSerialPort>("/a and /b on one serial port");
    expect_rejected<TwoSolversOnOneImu>("/a and /b on one onboard imu");
    expect_rejected<ImuDeclaredTwice>("/imu declared twice");
}

// mc02 用自己的描述符：三路 CAN、DBUS。组件名与丝印一起出现在日志标签里。
// DBUS 口上接 DR16：线路参数是驱动按协议给出的（100 kbit/s、偶校验、1 停止位），
// 要不要在板上取反由接线表说。
struct Mc02Wiring : Board<libhcs::board::Mc02> {
    CanBus can3{*this, Spec::kCans.kCan3, kClassic1M};
    SerialPort dbus{*this, Spec::kUarts.kDbus};
    DjiMotor wheel{can3, "/wheel", {.motor_type = DjiType::kM3508, .id = 1}};
    Dr16Remote remote{dbus, "/remote", {.rx_inverted = true}};
};

TEST(Board, Mc02UsesItsOwnDescriptors) {
    namespace vc = libhcs::board::hcs::vc;
    const auto board = make_component<BoardComponent<Mc02Wiring>>("m");
    EXPECT_EQ(board->can3.label(), "m/can3");
    EXPECT_EQ(board->can3.index(), 2U);
    EXPECT_EQ(board->dbus.label(), "m/dbus");
    EXPECT_EQ(board->dbus.index(), 0U);
    ASSERT_TRUE(board->dbus.setting().has_value());
    EXPECT_EQ(board->dbus.setting()->baudrate, 100'000U);
    EXPECT_EQ(board->dbus.setting()->parity, vc::kUartParityEven);
    EXPECT_EQ(board->dbus.setting()->stop_bits, vc::kUartStopBits1);
    EXPECT_EQ(board->dbus.setting()->rx_polarity, vc::kUartRxPolarityInverted);
}

// 两种遥控接收机接在同一块板上：都要写 /remote/*。没有定过谁说了算，所以不许悄悄地抢着写——
// 构造就失败，原因说得出是哪个输出重了。
struct TwoRemotesOnOneBoard : Board<libhcs::board::Mc02> {
    SerialPort dbus{*this, Spec::kUarts.kDbus};
    SerialPort uart1{*this, Spec::kUarts.kUart1};
    Dr16Remote dr16{dbus, "/remote", {}};
    Vt13Remote vt13{uart1, "/remote", {}};
};

TEST(Board, TwoRemoteReceiversOnOneBoardAreRejected) {
    try {
        make_component<BoardComponent<TwoRemotesOnOneBoard>>("two_remotes");
        ADD_FAILURE() << "accepted two drivers that both publish /remote/*";
    } catch (const std::exception& error) {
        EXPECT_NE(std::string_view{error.what()}.find("/remote/"), std::string_view::npos)
            << error.what();
    }
}

// ── 真实接线表 ───────────────────────────────────────────────────────────

// 三块板全部 enabled:false（参数缺省）：不碰 USB，接线检查照样跑完，接进图里空跑。
TEST(BalanceInfantry, BoardsLinkWithTransmitterAndSafetyLatch) {
    std::vector<std::shared_ptr<Component>> components{
        make_component<hcs_core::hardware::TransmitThread>("tx_balance"),
        make_component<hcs_core::hardware::BalanceGimbalBoard>("gimbal_board"),
        make_component<hcs_core::hardware::BalanceChassisBoard>("chassis_board"),
        make_component<hcs_core::hardware::BalanceAuxBoard>("aux_board"),
        make_component<hcs_core::hardware::SafetyLatch>("safety_latch"),
    };
    for (std::size_t i = 1; i <= 3; ++i)
        components.push_back(components[i]->partner_components().at(0));

    const auto wiring = Linker::link(components);
    ASSERT_TRUE(wiring.has_value()) << wiring.error().message;
    for (const auto& component : components)
        component->before_updating();

    for (std::uint64_t sequence = 0; sequence < 10; ++sequence) {
        for (const auto& entry : wiring->latches)
            entry.latch(entry.interface);
        const hcs_sync::Tick tick{
            .scheduled = hcs_sync::Clock::now(), .dt = std::chrono::milliseconds{1},
            .sequence = sequence};
        for (auto* component : wiring->updating_order)
            component->update(tick);
    }
}

} // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    setenv("ROS_LOG_DIR", "/tmp/hcs_test_ros_logs", 0);

    // 测试车的两块假板打开（FakeSdk 不碰 USB）；平衡步兵的真板保持缺省的禁用。
    const std::string params = testing::TempDir() + "test_board_params.yaml";
    std::ofstream{params} << R"(
left_board:
  ros__parameters: {enabled: true}
right_board:
  ros__parameters: {enabled: true}
test_latch:
  ros__parameters:
    critical_devices: [/left/joint, /right/yaw]
    rearm_switch: /test/switch
safety_latch:
  ros__parameters:
    critical_devices: [/chassis/left_front_joint, /chassis/left_back_joint,
                       /chassis/right_front_joint, /chassis/right_back_joint,
                       /chassis/left_wheel, /chassis/right_wheel, /chassis/imu]
    rearm_switch: /remote/switch/left
)";
    std::vector<char*> arguments{argv, argv + argc};
    for (const char* argument : {"--ros-args", "--params-file", params.c_str()})
        arguments.push_back(const_cast<char*>(argument));
    rclcpp::init(static_cast<int>(arguments.size()), arguments.data());
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
