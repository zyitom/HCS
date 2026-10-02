// DR16 遥控（DBUS）的单测：
//   - 18 字节一帧解成摇杆、拨杆、鼠标、键盘、拨轮，方向与量程对得上
//   - 帧没有帧头也没有校验，对齐靠两条：一拍没字节就扔掉半帧；凑满却不像一帧就丢一个字节重试
//   - 任意字节流都不许把缓存写穿（ASan 下这一条才真正有牙齿）
//   - 拨轮当三挡开关用时带回差，停在分界线上不来回跳
//   - 失联的那一拍全部归零、拨杆 UNKNOWN：读它的人拿到的永远不是掉线前的最后一帧
//
// 驱动只有协议：字节按到达顺序交给 on_bytes()，它运行在控制线程上，可以当单线程代码测。
// 字节怎么从 IO 线程过来、多少拍没有帧算失联，在端口包装层（board::Serial<>），测在 test_board。

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <hcs_msgs/switch.hpp>

#include "hardware/device/dr16.hpp"
#include "hardware/device/dr16_remote.hpp"
#include "hardware/device/serial_line.hpp"
#include "hcs_executor/component.hpp"
#include "hcs_executor/wiring.hpp"

namespace {

using hcs_core::hardware::device::Dr16;
using hcs_core::hardware::device::Dr16Remote;
using hcs_core::hardware::device::SerialLine;
using hcs_executor::Component;
using hcs_msgs::Switch;

using Frame = std::array<std::uint8_t, Dr16::kFrameSize>;

/// 一帧里的各个字段，按线上的原始值填。通道中位 1024，满量程 ±660；拨杆 1 上 / 2 下 / 3 中。
struct Fields {
    std::uint16_t channel0 = 1024; ///< 右摇杆 左右
    std::uint16_t channel1 = 1024; ///< 右摇杆 前后
    std::uint16_t channel2 = 1024; ///< 左摇杆 左右
    std::uint16_t channel3 = 1024; ///< 左摇杆 前后
    std::uint8_t switch_right = 3;
    std::uint8_t switch_left = 3;
    std::int16_t mouse_x = 0;
    std::int16_t mouse_y = 0;
    std::int16_t mouse_z = 0;
    std::uint8_t mouse_left = 0;
    std::uint8_t mouse_right = 0;
    std::uint16_t keyboard = 0;
    std::uint16_t rotary_knob = 1024;
};

/// 按协议的位布局拼一帧。自己再拼一遍而不是去借驱动里的结构体：借来的话，
/// 结构体排错了测试也跟着错。
Frame make_frame(const Fields& fields) {
    Frame frame{};
    const std::uint64_t bits = std::uint64_t{fields.channel0} | std::uint64_t{fields.channel1} << 11
                             | std::uint64_t{fields.channel2} << 22
                             | std::uint64_t{fields.channel3} << 33
                             | std::uint64_t{fields.switch_right} << 44
                             | std::uint64_t{fields.switch_left} << 46;
    for (std::size_t i = 0; i < 6; ++i)
        frame[i] = static_cast<std::uint8_t>(bits >> (8 * i));

    const auto put_u16 = [&](std::size_t at, std::uint16_t value) {
        frame[at] = static_cast<std::uint8_t>(value);
        frame[at + 1] = static_cast<std::uint8_t>(value >> 8);
    };
    put_u16(6, static_cast<std::uint16_t>(fields.mouse_x));
    put_u16(8, static_cast<std::uint16_t>(fields.mouse_y));
    put_u16(10, static_cast<std::uint16_t>(fields.mouse_z));
    frame[12] = fields.mouse_left;
    frame[13] = fields.mouse_right;
    put_u16(14, fields.keyboard);
    put_u16(16, fields.rotary_knob);
    return frame;
}

bool feed(Dr16& dr16, std::span<const std::uint8_t> bytes) {
    return dr16.on_bytes(std::as_bytes(bytes));
}

/// 一拍：喂这一拍到的字节（可以没有），然后拍尾。
bool tick(Dr16& dr16, std::span<const std::uint8_t> bytes = {}) {
    const bool decoded = bytes.empty() ? false : feed(dr16, bytes);
    dr16.end_of_tick();
    return decoded;
}

} // namespace

TEST(Dr16, DecodesJoysticksWithDirectionAndFullScale) {
    Dr16 dr16;
    Fields fields;
    fields.channel0 = 1024 + 660; // 右摇杆打到右
    fields.channel1 = 1024 + 330; // 右摇杆前推一半
    fields.channel2 = 1024 - 660; // 左摇杆打到左
    fields.channel3 = 1024 - 330; // 左摇杆后拉一半

    EXPECT_TRUE(feed(dr16, make_frame(fields)));

    // 输出坐标：x 向前，y 向左。
    EXPECT_DOUBLE_EQ(dr16.joystick_right().x(), 0.5);
    EXPECT_DOUBLE_EQ(dr16.joystick_right().y(), -1.0);
    EXPECT_DOUBLE_EQ(dr16.joystick_left().x(), -0.5);
    EXPECT_DOUBLE_EQ(dr16.joystick_left().y(), 1.0);
}

TEST(Dr16, DecodesSwitchesMouseKeyboardAndKnob) {
    Dr16 dr16;
    EXPECT_EQ(dr16.switch_left(), Switch::UNKNOWN);

    Fields fields;
    fields.switch_right = 1;
    fields.switch_left = 2;
    fields.mouse_x = 16384;
    fields.mouse_y = -8192;
    fields.mouse_z = 3277;
    fields.mouse_right = 1;
    fields.keyboard = 0b0000'0000'0001'0001; // W + Shift
    fields.rotary_knob = 1024 + 330;
    EXPECT_TRUE(feed(dr16, make_frame(fields)));

    EXPECT_EQ(dr16.switch_right(), Switch::UP);
    EXPECT_EQ(dr16.switch_left(), Switch::DOWN);
    EXPECT_DOUBLE_EQ(dr16.mouse_velocity().x(), 0.25); // -mouse_y / 32768
    EXPECT_DOUBLE_EQ(dr16.mouse_velocity().y(), -0.5); // -mouse_x / 32768
    EXPECT_DOUBLE_EQ(dr16.mouse_wheel(), -3277 / 32768.0);
    EXPECT_FALSE(dr16.mouse().left);
    EXPECT_TRUE(dr16.mouse().right);
    EXPECT_TRUE(dr16.keyboard().w);
    EXPECT_TRUE(dr16.keyboard().shift);
    EXPECT_FALSE(dr16.keyboard().ctrl);
    EXPECT_DOUBLE_EQ(dr16.rotary_knob(), 0.5);

    fields.switch_right = 3;
    EXPECT_TRUE(feed(dr16, make_frame(fields)));
    EXPECT_EQ(dr16.switch_right(), Switch::MIDDLE);
}

// 一帧被拆成两段、落在同一拍或相邻两拍里：照样解。
TEST(Dr16, FrameSplitAcrossChunksStillDecodes) {
    Dr16 dr16;
    Fields fields;
    fields.channel1 = 1024 + 660;
    const Frame frame = make_frame(fields);
    const auto bytes = std::span<const std::uint8_t>{frame};

    EXPECT_FALSE(tick(dr16, bytes.first(7)));
    EXPECT_TRUE(tick(dr16, bytes.subspan(7)));
    EXPECT_DOUBLE_EQ(dr16.joystick_right().x(), 1.0);
    EXPECT_EQ(dr16.rejected_frames(), 0u);
}

// 上电时从一帧的中间接上：那半帧后面跟着的是静默，静默的那一拍把它扔掉，下一帧就对齐了。
// 没有这条的话，半帧会和下一帧的前半段拼成一个错位的"帧"，而且永远错下去。
TEST(Dr16, AnIdleTickDropsThePartialFrameSoTheNextOneAligns) {
    Dr16 dr16;
    Fields fields;
    fields.channel1 = 1024 + 660;
    const Frame frame = make_frame(fields);
    const auto bytes = std::span<const std::uint8_t>{frame};

    EXPECT_FALSE(tick(dr16, bytes.subspan(11))); // 一帧的后 7 个字节
    EXPECT_FALSE(tick(dr16));                    // 帧间的静默
    EXPECT_TRUE(tick(dr16, frame));              // 下一帧：整帧，对齐
    EXPECT_DOUBLE_EQ(dr16.joystick_right().x(), 1.0);
    EXPECT_EQ(dr16.rejected_frames(), 0u) << "aligned by the idle gap, not by trial and error";
}

// 控制回路慢到每一拍都有字节、看不到静默的时候，靠合理性兜底：错位拼出来的 18 字节里
// 通道或拨杆的值不合法，丢一个字节再试，几帧之内重新对齐。
TEST(Dr16, AMisalignedStreamRealignsOnImplausibleValues) {
    Dr16 dr16;
    Fields fields;
    fields.channel1 = 1024 + 660;
    fields.keyboard = 0xFFFF; // 错位之后这些 1 会落进通道 / 拨杆的位置，凑不出合法值
    fields.mouse_x = -1;
    const Frame frame = make_frame(fields);

    std::vector<std::uint8_t> stream(frame.begin() + 5, frame.end()); // 从帧中间开始
    for (int i = 0; i < 6; ++i)
        stream.insert(stream.end(), frame.begin(), frame.end());

    EXPECT_TRUE(feed(dr16, stream)); // 一口气喂进去，中间没有拍尾
    EXPECT_GT(dr16.rejected_frames(), 0u);
    EXPECT_DOUBLE_EQ(dr16.joystick_right().x(), 1.0);
    EXPECT_TRUE(dr16.keyboard().w);
}

// 不合法的帧不解码：通道超量程、拨杆为 0、鼠标键不是 0 / 1。
TEST(Dr16, ImplausibleFramesAreNotDecoded) {
    const auto rejected = [](const Fields& fields) {
        Dr16 dr16;
        const bool decoded = tick(dr16, make_frame(fields));
        return !decoded && dr16.rejected_frames() > 0 && dr16.joystick_right().isZero()
            && dr16.switch_left() == Switch::UNKNOWN;
    };

    Fields fields;
    fields.channel2 = 1024 + 661;
    EXPECT_TRUE(rejected(fields));

    fields = {};
    fields.channel0 = 363;
    EXPECT_TRUE(rejected(fields));

    fields = {};
    fields.switch_left = 0;
    EXPECT_TRUE(rejected(fields));

    fields = {};
    fields.mouse_left = 2;
    EXPECT_TRUE(rejected(fields));
}

// 随便什么字节流、随便怎么切、随便隔几拍，都不许把缓存写穿，流过去之后还能重新对上帧。
TEST(Dr16, ArbitraryBytesNeverOverrunTheBufferAndTheStreamRecovers) {
    std::mt19937 random{20261002};
    std::uniform_int_distribution<int> any_byte{0, 255};
    std::uniform_int_distribution<std::size_t> chunk_size{0, 70};

    Dr16 dr16;
    std::vector<std::uint8_t> chunk;
    for (int round = 0; round < 5000; ++round) {
        chunk.resize(chunk_size(random));
        for (auto& byte : chunk)
            byte = static_cast<std::uint8_t>(any_byte(random));
        (void)tick(dr16, chunk);
    }

    Fields fields;
    fields.channel1 = 1024 - 660;
    (void)tick(dr16); // 静默
    EXPECT_TRUE(tick(dr16, make_frame(fields)));
    EXPECT_DOUBLE_EQ(dr16.joystick_right().x(), -1.0);
}

// 拨轮当开关：过 ±0.7 换挡；离开当前挡要多走 0.05，停在分界线附近不来回跳。
TEST(Dr16, RotaryKnobSwitchHasHysteresis) {
    Dr16 dr16;
    const auto knob = [&](double value) {
        Fields fields;
        fields.rotary_knob = static_cast<std::uint16_t>(1024 + value * 660);
        EXPECT_TRUE(feed(dr16, make_frame(fields)));
        return dr16.rotary_knob_switch();
    };

    EXPECT_EQ(knob(0.0), Switch::MIDDLE);
    // 输出的开关方向与拨轮读数相反：读数往负走是 UP。
    EXPECT_EQ(knob(-0.72), Switch::MIDDLE) << "from MIDDLE the threshold is 0.75";
    EXPECT_EQ(knob(-0.80), Switch::UP);
    EXPECT_EQ(knob(-0.68), Switch::UP) << "from UP the threshold back is 0.65";
    EXPECT_EQ(knob(-0.60), Switch::MIDDLE);
    EXPECT_EQ(knob(0.80), Switch::DOWN);
    EXPECT_EQ(knob(0.68), Switch::DOWN);
    EXPECT_EQ(knob(0.60), Switch::MIDDLE);
}

// 失联的那一拍归零。只在"在线 → 失联"的沿上做一次：之后来的帧照常生效。
TEST(Dr16, GoingOfflineZeroesEverything) {
    Dr16 dr16;
    Fields fields;
    fields.channel1 = 1024 + 660;
    fields.switch_left = 1;
    fields.mouse_left = 1;
    fields.keyboard = 0x0001;
    fields.rotary_knob = 1024 - 660;
    ASSERT_TRUE(feed(dr16, make_frame(fields)));
    dr16.set_online(true);
    EXPECT_TRUE(dr16.valid());
    EXPECT_EQ(dr16.rotary_knob_switch(), Switch::UP);

    dr16.set_online(false);
    EXPECT_FALSE(dr16.valid());
    EXPECT_TRUE(dr16.joystick_right().isZero());
    EXPECT_EQ(dr16.switch_left(), Switch::UNKNOWN);
    EXPECT_FALSE(dr16.mouse().left);
    EXPECT_FALSE(dr16.keyboard().w);
    EXPECT_DOUBLE_EQ(dr16.rotary_knob(), 0.0);
    EXPECT_EQ(dr16.rotary_knob_switch(), Switch::UNKNOWN);

    ASSERT_TRUE(feed(dr16, make_frame(fields)));
    dr16.set_online(true);
    EXPECT_DOUBLE_EQ(dr16.joystick_right().x(), 1.0);
}

// ── Dr16Remote：解出来的量怎么变成 /remote/* ─────────────────────────────

namespace {

class Host : public Component {
public:
    explicit Host(std::string name)
        : Component{std::move(name)} {}
    void update(const hcs_sync::Tick&) override {}
};

class Reader : public Component {
public:
    Reader()
        : Component{"reader"} {
        register_input("/remote/joystick/right", joystick_right);
        register_input("/remote/switch/left", switch_left);
        register_input("/remote/switch/right", switch_right);
        register_input("/remote/keyboard", keyboard);
        register_input("/remote/rotary_knob", rotary_knob);
        register_input("/remote/rotary_knob_switch", rotary_knob_switch);
    }
    void update(const hcs_sync::Tick&) override {}

    InputInterface<Eigen::Vector2d> joystick_right;
    InputInterface<Switch> switch_left, switch_right, rotary_knob_switch;
    InputInterface<hcs_msgs::Keyboard> keyboard;
    InputInterface<double> rotary_knob;
};

struct RemoteBench {
    RemoteBench()
        : host{std::make_shared<Host>("remote")}
        , remote{*host, "/remote", {}}
        , reader{std::make_shared<Reader>()} {
        EXPECT_TRUE(hcs_executor::Linker::link({host, reader}).has_value());
    }

    /// 端口包装层一拍里对驱动做的事：喂字节，然后告诉它在不在线。
    void tick(std::span<const std::uint8_t> bytes, bool online) {
        if (!bytes.empty())
            (void)remote.on_bytes(std::as_bytes(bytes));
        remote.on_tick(online);
    }

    std::shared_ptr<Host> host;
    Dr16Remote remote;
    std::shared_ptr<Reader> reader;
};

} // namespace

// DBUS 的线路参数是协议定死的；要不要在板上取反留给接线表说（各板的硬件不一样）。
TEST(Dr16Remote, DeclaresTheDbusLine) {
    auto host = std::make_shared<Host>("line");
    const Dr16Remote by_default{*host, "/remote", {}};
    EXPECT_EQ(by_default.serial_line().baudrate, 100'000u);
    EXPECT_EQ(by_default.serial_line().parity, SerialLine::Parity::kEven);
    EXPECT_EQ(by_default.serial_line().stop_bits, 1);
    EXPECT_FALSE(by_default.serial_line().rx_inverted.has_value());

    auto other = std::make_shared<Host>("line2");
    const Dr16Remote inverted{*other, "/remote", {.rx_inverted = true}};
    EXPECT_EQ(inverted.serial_line().rx_inverted, true);
}

TEST(Dr16Remote, OnlineFramesReachTheOutputsAsTheyAre) {
    RemoteBench bench;
    Fields fields;
    fields.channel1 = 1024 + 660;
    fields.switch_left = 2;
    fields.switch_right = 1;
    fields.keyboard = 0x0001;
    fields.rotary_knob = 1024 + 660;

    bench.tick(make_frame(fields), true);
    EXPECT_DOUBLE_EQ(bench.reader->joystick_right->x(), 1.0);
    EXPECT_EQ(*bench.reader->switch_left, Switch::DOWN);
    EXPECT_EQ(*bench.reader->switch_right, Switch::UP);
    EXPECT_TRUE(bench.reader->keyboard->w);
    EXPECT_DOUBLE_EQ(*bench.reader->rotary_knob, 1.0);
    EXPECT_EQ(*bench.reader->rotary_knob_switch, Switch::DOWN);
    EXPECT_EQ(bench.remote.switch_left(), Switch::DOWN);
}

// 首帧之前、以及失联之后：输出是归零的，拨杆 UNKNOWN——不是掉线前的最后一帧。
TEST(Dr16Remote, OfflineMeansZeroedOutputsNotTheLastFrame) {
    RemoteBench bench;
    bench.tick({}, false);
    EXPECT_TRUE(bench.reader->joystick_right->isZero());
    EXPECT_EQ(*bench.reader->switch_left, Switch::UNKNOWN);

    Fields fields;
    fields.channel1 = 1024 + 660;
    fields.rotary_knob = 1024 + 660;
    bench.tick(make_frame(fields), true);
    ASSERT_DOUBLE_EQ(bench.reader->joystick_right->x(), 1.0);

    bench.tick({}, true); // 没有新帧但还在超时之内：保持
    EXPECT_DOUBLE_EQ(bench.reader->joystick_right->x(), 1.0);

    bench.tick({}, false); // 失联
    EXPECT_TRUE(bench.reader->joystick_right->isZero());
    EXPECT_EQ(*bench.reader->switch_left, Switch::UNKNOWN);
    EXPECT_EQ(*bench.reader->switch_right, Switch::UNKNOWN);
    EXPECT_DOUBLE_EQ(*bench.reader->rotary_knob, 0.0);
    EXPECT_EQ(*bench.reader->rotary_knob_switch, Switch::UNKNOWN);

    bench.tick(make_frame(fields), true); // 回来了
    EXPECT_DOUBLE_EQ(bench.reader->joystick_right->x(), 1.0);
    EXPECT_EQ(*bench.reader->switch_left, Switch::MIDDLE);
}

// 流没对齐不是静默的：凑满却不像一帧的次数每涨一次，尽力域报一次。
TEST(Dr16Remote, MisalignmentIsReportedNotSilent) {
    RemoteBench bench;
    EXPECT_FALSE(bench.remote.take_problem().has_value());

    Fields fields;
    fields.switch_left = 0; // 不合法
    bench.tick(make_frame(fields), false);

    const auto problem = bench.remote.take_problem();
    ASSERT_TRUE(problem.has_value());
    EXPECT_NE(problem->find("did not look like DR16 frames"), std::string::npos) << *problem;
    EXPECT_FALSE(bench.remote.take_problem().has_value()) << "already reported";
}
