// VT13 图传遥控的单测：
//   - 遥控帧（0xA9 0x53，21 字节，CRC16）解成摇杆、挡位、鼠标、键盘，方向与量程对得上
//   - 帧拆成几段喂、前面混着垃圾、CRC 错，都不影响后面的好帧
//   - 同一路串口上的裁判系统格式帧（0xA5 开头）按帧长跳过，不算"收到遥控帧"
//   - 任意字节流都不许把缓存写穿（ASan 下这一条才真正有牙齿）
//   - 失联的那一拍摇杆、鼠标、键盘归零、拨杆 UNKNOWN：读它的人拿到的永远不是掉线前的最后一帧
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

#include <hcs_base/protocol/dji_crc.hpp>
#include <hcs_msgs/switch.hpp>

#include "hardware/device/vt13.hpp"
#include "hardware/device/vt13_remote.hpp"
#include "hcs_executor/component.hpp"
#include "hcs_executor/wiring.hpp"

namespace {

using hcs_core::hardware::device::Vt13;
using hcs_core::hardware::device::Vt13Remote;
using hcs_executor::Component;

constexpr std::size_t kRemoteFrameSize = 21;
using RemoteFrame = std::array<std::uint8_t, kRemoteFrameSize>;

/// 遥控帧里的各个字段，按线上的原始值填。摇杆中位 1024，满量程 ±660。
struct Fields {
    std::uint16_t channel0 = 1024; ///< 右摇杆 左右
    std::uint16_t channel1 = 1024; ///< 右摇杆 前后
    std::uint16_t channel2 = 1024; ///< 左摇杆 前后
    std::uint16_t channel3 = 1024; ///< 左摇杆 左右
    std::uint8_t mode_switch = 1;  ///< 0 = C，1 = N，2 = S
    std::int16_t mouse_x = 0;
    std::int16_t mouse_y = 0;
    std::int16_t mouse_z = 0;
    std::uint8_t mouse_left = 0;
    std::uint8_t mouse_right = 0;
    std::uint16_t keyboard = 0;
};

/// 按协议的位布局拼一帧（位域从低位排起）：
///   字节 0-1    帧头 A9 53
///   位 0-43     四个 11 位摇杆通道
///   位 44-45    挡位；46 暂停键；47 / 48 自定义键；49-59 拨轮；60 扳机；61-63 保留
///   字节 10-15  鼠标 x / y / z，int16
///   字节 16     鼠标左 / 右 / 中键，各 2 位
///   字节 17-18  键盘位图
///   字节 19-20  CRC16
/// 这里自己再拼一遍而不是去借驱动里的那个结构体：借来的话，结构体排错了测试也跟着错。
RemoteFrame make_frame(const Fields& fields) {
    RemoteFrame frame{};
    frame[0] = 0xA9;
    frame[1] = 0x53;

    const std::uint64_t bits = std::uint64_t{fields.channel0} | std::uint64_t{fields.channel1} << 11
                             | std::uint64_t{fields.channel2} << 22
                             | std::uint64_t{fields.channel3} << 33
                             | std::uint64_t{fields.mode_switch} << 44
                             | std::uint64_t{1024} << 49; // 拨轮居中
    for (std::size_t i = 0; i < 8; ++i)
        frame[2 + i] = static_cast<std::uint8_t>(bits >> (8 * i));

    const auto put_i16 = [&](std::size_t at, std::int16_t value) {
        frame[at] = static_cast<std::uint8_t>(value);
        frame[at + 1] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(value) >> 8);
    };
    put_i16(10, fields.mouse_x);
    put_i16(12, fields.mouse_y);
    put_i16(14, fields.mouse_z);
    frame[16] = static_cast<std::uint8_t>(fields.mouse_left | fields.mouse_right << 2);
    frame[17] = static_cast<std::uint8_t>(fields.keyboard);
    frame[18] = static_cast<std::uint8_t>(fields.keyboard >> 8);

    hcs_utility::dji_crc::append_crc16(frame.data(), frame.size());
    return frame;
}

/// 裁判系统格式的一帧：A5 | 数据长度 u16 | 序号 | CRC8 | 命令字 2 | 数据 | CRC16 2。
/// 驱动只验帧头的 CRC8，然后按长度整帧跳过。
std::vector<std::uint8_t> make_referee_frame(std::uint16_t data_length) {
    std::vector<std::uint8_t> frame(5 + 2 + data_length + 2, 0x77);
    frame[0] = 0xA5;
    frame[1] = static_cast<std::uint8_t>(data_length);
    frame[2] = static_cast<std::uint8_t>(data_length >> 8);
    frame[3] = 0x01;
    hcs_utility::dji_crc::append_crc8(frame.data(), 5);
    return frame;
}

bool feed(Vt13& vt13, std::span<const std::uint8_t> bytes) {
    return vt13.on_bytes(std::as_bytes(bytes));
}

Fields sport() {
    Fields fields;
    fields.mode_switch = 2;
    return fields;
}

} // namespace

TEST(Vt13, DecodesJoysticksWithDirectionAndFullScale) {
    Vt13 vt13;
    Fields fields = sport();
    fields.channel0 = 1024 + 660; // 右摇杆打到右
    fields.channel1 = 1024 + 330; // 右摇杆前推一半
    fields.channel2 = 1024 - 660; // 左摇杆拉到底
    fields.channel3 = 1024 - 330; // 左摇杆向左一半

    EXPECT_TRUE(feed(vt13, make_frame(fields)));
    EXPECT_EQ(vt13.mode_switch(), Vt13::ModeSwitch::kSport);

    // 输出坐标：x 向前，y 向左。
    EXPECT_DOUBLE_EQ(vt13.joystick_right().x(), 0.5);
    EXPECT_DOUBLE_EQ(vt13.joystick_right().y(), -1.0);
    EXPECT_DOUBLE_EQ(vt13.joystick_left().x(), -1.0);
    EXPECT_DOUBLE_EQ(vt13.joystick_left().y(), 0.5);
}

TEST(Vt13, CentredSticksReadZeroAndOutOfRangeChannelsAreIgnored) {
    Vt13 vt13;
    EXPECT_TRUE(feed(vt13, make_frame(sport())));
    EXPECT_TRUE(vt13.joystick_left().isZero());
    EXPECT_TRUE(vt13.joystick_right().isZero());

    // 量程之外的原始值（接收机刚上电、通道还没校准时会出现）读作 0，而不是一个大于 1 的摇杆量。
    Fields fields = sport();
    fields.channel1 = 2047;
    fields.channel2 = 0;
    EXPECT_TRUE(feed(vt13, make_frame(fields)));
    EXPECT_DOUBLE_EQ(vt13.joystick_right().x(), 0.0);
    EXPECT_DOUBLE_EQ(vt13.joystick_left().x(), 0.0);
}

TEST(Vt13, DecodesModeSwitchMouseAndKeyboard) {
    Vt13 vt13;
    EXPECT_EQ(vt13.mode_switch(), Vt13::ModeSwitch::kUnknown);

    Fields fields;
    fields.mode_switch = 0;
    EXPECT_TRUE(feed(vt13, make_frame(fields)));
    EXPECT_EQ(vt13.mode_switch(), Vt13::ModeSwitch::kCine);
    fields.mode_switch = 1;
    EXPECT_TRUE(feed(vt13, make_frame(fields)));
    EXPECT_EQ(vt13.mode_switch(), Vt13::ModeSwitch::kNormal);

    fields = sport();
    fields.mouse_x = 16384;
    fields.mouse_y = -8192;
    fields.mouse_z = 3277;
    fields.mouse_left = 1;
    fields.keyboard = 0b0000'0000'0001'0001; // W + Shift
    EXPECT_TRUE(feed(vt13, make_frame(fields)));

    EXPECT_DOUBLE_EQ(vt13.mouse_velocity().x(), 0.25); // -mouse_y / 32768
    EXPECT_DOUBLE_EQ(vt13.mouse_velocity().y(), -0.5); // -mouse_x / 32768
    EXPECT_DOUBLE_EQ(vt13.mouse_wheel(), -3277 / 32768.0);
    EXPECT_TRUE(vt13.mouse().left);
    EXPECT_FALSE(vt13.mouse().right);
    EXPECT_TRUE(vt13.keyboard().w);
    EXPECT_TRUE(vt13.keyboard().shift);
    EXPECT_FALSE(vt13.keyboard().s);
    EXPECT_FALSE(vt13.keyboard().ctrl);
}

// 帧边界和字节段的边界对不上是常态：半帧留在缓存里，等后半段到了再解。
TEST(Vt13, FrameSplitAcrossChunksStillDecodes) {
    Vt13 vt13;
    Fields fields = sport();
    fields.channel1 = 1024 + 660;
    const RemoteFrame frame = make_frame(fields);
    const auto bytes = std::span<const std::uint8_t>{frame};

    EXPECT_FALSE(feed(vt13, bytes.first(1)));
    EXPECT_FALSE(feed(vt13, bytes.subspan(1, 12)));
    EXPECT_EQ(vt13.mode_switch(), Vt13::ModeSwitch::kUnknown);
    EXPECT_TRUE(feed(vt13, bytes.subspan(13)));
    EXPECT_DOUBLE_EQ(vt13.joystick_right().x(), 1.0);
}

// CRC 错的帧不解码，也不许把紧跟着的好帧带坏。
TEST(Vt13, CorruptedFrameIsDroppedAndTheNextOneStillDecodes) {
    Vt13 vt13;
    Fields bad = sport();
    bad.channel1 = 1024 + 660;
    RemoteFrame corrupted = make_frame(bad);
    corrupted[5] ^= 0x10;

    EXPECT_FALSE(feed(vt13, corrupted));
    EXPECT_EQ(vt13.mode_switch(), Vt13::ModeSwitch::kUnknown);
    EXPECT_TRUE(vt13.joystick_right().isZero());

    Fields good = sport();
    good.channel1 = 1024 - 660;
    EXPECT_TRUE(feed(vt13, make_frame(good)));
    EXPECT_DOUBLE_EQ(vt13.joystick_right().x(), -1.0);
}

// 帧前面混着别的东西（含假帧头）：逐字节重新对齐。
TEST(Vt13, ResynchronizesPastGarbage) {
    Vt13 vt13;
    std::vector<std::uint8_t> chunk{0x00, 0xA9, 0x12, 0xA9, 0x53, 0xFF, 0x34, 0xA5, 0x00};
    Fields fields = sport();
    fields.channel3 = 1024 + 660;
    const RemoteFrame frame = make_frame(fields);
    chunk.insert(chunk.end(), frame.begin(), frame.end());
    // 垃圾里的假帧头会让解析先"等够一帧再说"，再多给一帧它才会把前面的吐掉、对上真正的那一帧。
    chunk.insert(chunk.end(), frame.begin(), frame.end());

    EXPECT_TRUE(feed(vt13, chunk));
    EXPECT_DOUBLE_EQ(vt13.joystick_left().y(), -1.0);
}

// 同一路串口上还有裁判系统格式的帧：整帧跳过，不算遥控帧，也不挡住后面的遥控帧。
TEST(Vt13, RefereeStyleFramesAreSkippedNotCountedAsRemoteFrames) {
    Vt13 vt13;
    const auto referee = make_referee_frame(30);
    EXPECT_FALSE(feed(vt13, referee)) << "a skipped frame is not a remote frame";
    EXPECT_EQ(vt13.mode_switch(), Vt13::ModeSwitch::kUnknown);

    std::vector<std::uint8_t> chunk = make_referee_frame(8);
    const RemoteFrame frame = make_frame(sport());
    chunk.insert(chunk.end(), frame.begin(), frame.end());
    const auto more = make_referee_frame(100);
    chunk.insert(chunk.end(), more.begin(), more.end());

    EXPECT_TRUE(feed(vt13, chunk));
    EXPECT_EQ(vt13.mode_switch(), Vt13::ModeSwitch::kSport);
}

// 随便什么字节流、随便怎么切，都不许把缓存写穿，流过去之后还能重新对上帧。
// 帧头字节特意掺得很密：每一个都会让解析走一遍"是不是一帧"的判断。
TEST(Vt13, ArbitraryBytesNeverOverrunTheCacheAndTheStreamRecovers) {
    std::mt19937 random{20261002};
    std::uniform_int_distribution<int> any_byte{0, 255};
    std::uniform_int_distribution<int> kind{0, 9};
    std::uniform_int_distribution<std::size_t> chunk_size{1, 700};

    Vt13 vt13;
    std::vector<std::uint8_t> chunk;
    for (int round = 0; round < 2000; ++round) {
        chunk.resize(chunk_size(random));
        for (auto& byte : chunk) {
            const int pick = kind(random);
            byte = pick == 0 ? 0xA9 : pick == 1 ? 0xA5 : pick == 2 ? 0x53
                 : static_cast<std::uint8_t>(any_byte(random));
        }
        (void)feed(vt13, chunk);
    }

    // 一段连着的好帧：前面残留的半帧最多吃掉开头几帧，后面的必须解得出来。
    Fields fields = sport();
    fields.channel1 = 1024 + 660;
    const RemoteFrame frame = make_frame(fields);
    bool decoded = false;
    for (int i = 0; i < 40; ++i)
        decoded |= feed(vt13, frame);
    EXPECT_TRUE(decoded);
    EXPECT_DOUBLE_EQ(vt13.joystick_right().x(), 1.0);
}

// 失联的那一拍归零。只在"在线 → 失联"的沿上做一次：之后来的帧照常生效。
TEST(Vt13, GoingOfflineZeroesEverything) {
    Vt13 vt13;
    Fields fields = sport();
    fields.channel1 = 1024 + 660;
    fields.mouse_left = 1;
    fields.keyboard = 0x0001;
    ASSERT_TRUE(feed(vt13, make_frame(fields)));
    vt13.set_online(true);
    EXPECT_TRUE(vt13.valid());
    EXPECT_DOUBLE_EQ(vt13.joystick_right().x(), 1.0);

    vt13.set_online(false);
    EXPECT_FALSE(vt13.valid());
    EXPECT_EQ(vt13.mode_switch(), Vt13::ModeSwitch::kUnknown);
    EXPECT_TRUE(vt13.joystick_right().isZero());
    EXPECT_FALSE(vt13.mouse().left);
    EXPECT_FALSE(vt13.keyboard().w);

    ASSERT_TRUE(feed(vt13, make_frame(fields)));
    vt13.set_online(true);
    EXPECT_DOUBLE_EQ(vt13.joystick_right().x(), 1.0);
}

// ── Vt13Remote：解出来的量怎么变成 /remote/* ─────────────────────────────

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
    }
    void update(const hcs_sync::Tick&) override {}

    InputInterface<Eigen::Vector2d> joystick_right;
    InputInterface<hcs_msgs::Switch> switch_left, switch_right;
    InputInterface<hcs_msgs::Keyboard> keyboard;
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
    Vt13Remote remote;
    std::shared_ptr<Reader> reader;
};

} // namespace

// 挡位决定输出：S = 遥控生效，两个拨杆 MIDDLE；C = 摇杆归零，两个拨杆 DOWN（安全锁存的复位手势
// 读的就是它）；N = 全部归零，拨杆 UNKNOWN。
TEST(Vt13Remote, ModeSwitchDecidesWhatReachesTheOutputs) {
    RemoteBench bench;
    Fields fields = sport();
    fields.channel1 = 1024 + 660;
    fields.keyboard = 0x0001;

    bench.tick(make_frame(fields), true);
    EXPECT_DOUBLE_EQ(bench.reader->joystick_right->x(), 1.0);
    EXPECT_TRUE(bench.reader->keyboard->w);
    EXPECT_EQ(*bench.reader->switch_left, hcs_msgs::Switch::MIDDLE);
    EXPECT_EQ(*bench.reader->switch_right, hcs_msgs::Switch::MIDDLE);
    EXPECT_EQ(bench.remote.switch_left(), hcs_msgs::Switch::MIDDLE);

    fields.mode_switch = 0; // C
    bench.tick(make_frame(fields), true);
    EXPECT_TRUE(bench.reader->joystick_right->isZero()) << "sticks must not drive in C";
    EXPECT_FALSE(bench.reader->keyboard->w);
    EXPECT_EQ(*bench.reader->switch_left, hcs_msgs::Switch::DOWN);
    EXPECT_EQ(*bench.reader->switch_right, hcs_msgs::Switch::DOWN);

    fields.mode_switch = 1; // N
    bench.tick(make_frame(fields), true);
    EXPECT_TRUE(bench.reader->joystick_right->isZero());
    EXPECT_EQ(*bench.reader->switch_left, hcs_msgs::Switch::UNKNOWN);
}

// 首帧之前、以及失联之后：输出是归零的，拨杆 UNKNOWN——不是掉线前的最后一帧。
TEST(Vt13Remote, OfflineMeansZeroedOutputsNotTheLastFrame) {
    RemoteBench bench;
    bench.tick({}, false);
    EXPECT_TRUE(bench.reader->joystick_right->isZero());
    EXPECT_EQ(*bench.reader->switch_left, hcs_msgs::Switch::UNKNOWN);

    Fields fields = sport();
    fields.channel1 = 1024 + 660;
    bench.tick(make_frame(fields), true);
    ASSERT_DOUBLE_EQ(bench.reader->joystick_right->x(), 1.0);

    // 没有新帧但还在超时之内：保持。
    bench.tick({}, true);
    EXPECT_DOUBLE_EQ(bench.reader->joystick_right->x(), 1.0);

    // 失联。
    bench.tick({}, false);
    EXPECT_TRUE(bench.reader->joystick_right->isZero());
    EXPECT_EQ(*bench.reader->switch_left, hcs_msgs::Switch::UNKNOWN);
    EXPECT_EQ(*bench.reader->switch_right, hcs_msgs::Switch::UNKNOWN);

    // 回来了。
    bench.tick(make_frame(fields), true);
    EXPECT_DOUBLE_EQ(bench.reader->joystick_right->x(), 1.0);
    EXPECT_EQ(*bench.reader->switch_left, hcs_msgs::Switch::MIDDLE);
}
