// HiPNUC（HI91）驱动的单测：
//   - 手册 7.34 的示例帧能解出来，拆成几段喂也一样
//   - CRC 错的帧只计数、不解码
//   - 一段字节里有几帧时，输出是最新那帧
//   - 混在帧前面的垃圾字节被跳过
//
// 驱动只有协议：字节按到达顺序交给 on_bytes()，它运行在控制线程上，可以当单线程代码测。
// 跨线程交接（字节怎么从 IO 线程过来）、掉线计数与健康输出在端口包装层
// （board::Serial<Hipnuc>），测在 test_board。

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "hardware/device/hipnuc.hpp"
#include "hcs_executor/component.hpp"
#include "hcs_executor/wiring.hpp"

#include "hipnuc_test_frames.hpp"

namespace {

using hcs_core::hardware::device::Hipnuc;
using hcs_executor::Component;
using hipnuc_test::Frame;
using hipnuc_test::kManualFrame;
using hipnuc_test::kTemperatureOffset;
using hipnuc_test::make_frame;

class Host : public Component {
public:
    explicit Host(std::string name)
        : Component{std::move(name)} {}
    void update(const hcs_sync::Tick&) override {}
};

struct Bench {
    Bench()
        : host{std::make_shared<Host>("imu")}
        , imu{*host, "/imu", {.baudrate = 921600}} {
        EXPECT_TRUE(hcs_executor::Linker::link({host}).has_value());
    }

    /// 喂一段字节。返回这一段里有没有解出至少一帧（包装层的掉线计数只认它）。
    bool feed(std::span<const std::uint8_t> bytes) { return imu.on_bytes(std::as_bytes(bytes)); }

    std::shared_ptr<Host> host;
    Hipnuc imu;
};

} // namespace

// 首帧之前什么都不解：姿态是单位四元数，其余读零。
TEST(Hipnuc, NothingDecodedBeforeFirstFrame) {
    Bench bench;
    EXPECT_EQ(bench.imu.frame_count(), 0u);
    EXPECT_EQ(bench.imu.crc_error_count(), 0u);
    EXPECT_TRUE(bench.imu.quaternion().isApprox(Eigen::Quaterniond::Identity()));
    EXPECT_DOUBLE_EQ(bench.imu.temperature(), 0.0);
}

TEST(Hipnuc, ManualExampleFrameDecodes) {
    Bench bench;
    EXPECT_TRUE(bench.feed(kManualFrame));
    EXPECT_EQ(bench.imu.frame_count(), 1u);
    EXPECT_EQ(bench.imu.crc_error_count(), 0u);
    EXPECT_DOUBLE_EQ(bench.imu.temperature(), 35.0);
    EXPECT_NEAR(bench.imu.air_pressure(), 100676.07, 0.01);
    EXPECT_NEAR(bench.imu.quaternion().norm(), 1.0, 1e-9);
}

// 帧边界和字节段的边界对不上是常态：半帧留在驱动里，等后半段到了再解。
TEST(Hipnuc, FrameSplitAcrossChunksStillDecodes) {
    Bench bench;
    const auto bytes = std::span<const std::uint8_t>{kManualFrame};
    EXPECT_FALSE(bench.feed(bytes.first(1)));
    EXPECT_FALSE(bench.feed(bytes.subspan(1, 40)));
    EXPECT_EQ(bench.imu.frame_count(), 0u);
    EXPECT_TRUE(bench.feed(bytes.subspan(41)));

    EXPECT_EQ(bench.imu.frame_count(), 1u);
    EXPECT_DOUBLE_EQ(bench.imu.temperature(), 35.0);
}

TEST(Hipnuc, CorruptedFrameIsCountedAndNotDecoded) {
    Bench bench;
    Frame frame = kManualFrame;
    frame[kTemperatureOffset] ^= 0x01;

    EXPECT_FALSE(bench.feed(frame));
    EXPECT_EQ(bench.imu.crc_error_count(), 1u);
    EXPECT_EQ(bench.imu.frame_count(), 0u);
    EXPECT_DOUBLE_EQ(bench.imu.temperature(), 0.0);

    // 坏帧不许把后面的好帧也带坏。
    EXPECT_TRUE(bench.feed(make_frame(40)));
    EXPECT_EQ(bench.imu.frame_count(), 1u);
    EXPECT_DOUBLE_EQ(bench.imu.temperature(), 40.0);
}

// 控制线程一拍取到的一段里可能有不止一帧：都解，输出留最新的。
TEST(Hipnuc, SeveralFramesInOneChunkDecodeTheNewest) {
    Bench bench;
    std::vector<std::uint8_t> chunk;
    for (const std::int8_t tag : {10, 20, 30}) {
        const Frame frame = make_frame(tag);
        chunk.insert(chunk.end(), frame.begin(), frame.end());
    }

    EXPECT_TRUE(bench.feed(chunk));
    EXPECT_EQ(bench.imu.frame_count(), 3u);
    EXPECT_DOUBLE_EQ(bench.imu.temperature(), 30.0);
    EXPECT_DOUBLE_EQ(bench.imu.air_pressure(), 3000.0);
}

// 帧前面混着别的东西（上电时的半帧、别的协议的输出）：逐字节重新对齐，后面的整帧照样解出来。
TEST(Hipnuc, ResynchronizesPastGarbage) {
    Bench bench;
    std::vector<std::uint8_t> chunk{0x00, 0x5A, 0x13, 0xA5, 0x5A, 0x5A, 0xFF}; // 含假帧头
    const Frame frame = make_frame(25);
    chunk.insert(chunk.end(), frame.begin(), frame.end());

    EXPECT_TRUE(bench.feed(chunk));
    EXPECT_EQ(bench.imu.frame_count(), 1u);
    EXPECT_DOUBLE_EQ(bench.imu.temperature(), 25.0);
}

// 解码的结果要写到输出上，别的组件读的是输出而不是驱动的成员。
TEST(Hipnuc, DecodedFrameIsPublishedToTheOutputs) {
    class Reader : public Component {
    public:
        Reader()
            : Component{"reader"} {
            register_input("/imu/quaternion", quaternion);
            register_input("/imu/angular_velocity/z", yaw_rate);
            register_input("/imu/euler/yaw", yaw);
        }
        void update(const hcs_sync::Tick&) override {}

        InputInterface<Eigen::Quaterniond> quaternion;
        InputInterface<double> yaw_rate;
        InputInterface<double> yaw;
    };

    auto host = std::make_shared<Host>("imu");
    Hipnuc imu{*host, "/imu", {.baudrate = 921600}};
    auto reader = std::make_shared<Reader>();
    ASSERT_TRUE(hcs_executor::Linker::link({host, reader}).has_value());

    // 首帧之前：姿态角没有"零姿态"这种合法默认，是 NaN；角速度的零是合法的。
    EXPECT_TRUE(std::isnan(*reader->yaw));
    EXPECT_DOUBLE_EQ(*reader->yaw_rate, 0.0);

    ASSERT_TRUE(imu.on_bytes(std::as_bytes(std::span{kManualFrame})));
    EXPECT_TRUE(reader->quaternion->isApprox(imu.quaternion()));
    EXPECT_DOUBLE_EQ(*reader->yaw_rate, imu.angular_velocity().z());
    EXPECT_DOUBLE_EQ(*reader->yaw, imu.euler_angles()[2]);
    EXPECT_FALSE(std::isnan(*reader->yaw));
}
