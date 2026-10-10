// LK 驱动的单测，对照瓴控 CAN 协议 V2.36 钉住反馈侧的几条规矩：
//   - 只有 0x9C 与 0xA1~0xA8 的回复按"状态 2"布局解码；同 id 的其它回复（0x9A 状态 1、
//     0x90 编码器、0x92 多圈角度……）布局不同，混进来会把多圈计数带跳——accepts() 拒掉
//   - 首帧到达前不解码：全零缓冲解成 encoder 0，多圈模式会从一个电机从没报过的位置起算
//   - 转矩电流满量程按系列：MG ±33 A，MF/MH ±16.5 A，指令与反馈同一个系数
//
// 驱动只有协议：帧先过 accepts() 再交给 on_frame()，与端口包装层的顺序相同。
// 被拒帧的计数、跨线程交接与掉线计数在包装层（board::Can<LkMotor>），测在 test_board。

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "hardware/device/lk_motor.hpp"
#include "hcs_executor/component.hpp"
#include "hcs_executor/wiring.hpp"

namespace {

using hcs_core::hardware::device::CanPacket8;
using hcs_core::hardware::device::LkMotor;
constexpr auto kTorque = LkMotor::ControlMode::kTorque;
using hcs_executor::Component;

class Host : public Component {
public:
    explicit Host(std::string name)
        : Component{std::move(name)} {}
    void update(const hcs_sync::Tick&) override {}
};

struct Bench {
    explicit Bench(const LkMotor::Config& config)
        : host{std::make_shared<Host>("lk")}
        , motor{*host, *host, "/yaw", config} {
        // 输出要先接上存储才能写；控制输入全都没人提供
        EXPECT_TRUE(hcs_executor::Linker::link({host}).has_value());
    }

    /// 一帧反馈。返回驱动是否认这一帧；认了才解码。
    bool receive(std::uint8_t command, std::int16_t iq, std::int16_t speed, std::uint16_t encoder,
                 std::int8_t temperature = 30) {
        const std::array<std::uint8_t, 8> frame{
            command,
            static_cast<std::uint8_t>(temperature),
            static_cast<std::uint8_t>(iq),
            static_cast<std::uint8_t>(static_cast<std::uint16_t>(iq) >> 8),
            static_cast<std::uint8_t>(speed),
            static_cast<std::uint8_t>(static_cast<std::uint16_t>(speed) >> 8),
            static_cast<std::uint8_t>(encoder),
            static_cast<std::uint8_t>(encoder >> 8)};
        const CanPacket8 packet{std::as_bytes(std::span{frame})};
        if (!motor.accepts(packet))
            return false;
        motor.on_frame(packet);
        return true;
    }

    std::shared_ptr<Host> host;
    LkMotor motor;
};

constexpr double kCountToRad = 2 * std::numbers::pi / 65536;

} // namespace

TEST(LkMotor, DecodesStatusTwoLayout) {
    Bench bench{LkMotor::Config{LkMotor::Type::kMG5010Ei10, kTorque, 0x145}};
    EXPECT_TRUE(bench.receive(0x9C, 1024, 360, 16384, 42));

    EXPECT_DOUBLE_EQ(bench.motor.temperature(), 42.0);
    EXPECT_NEAR(bench.motor.angle(), std::numbers::pi / 2, 1e-12);
    // 360 dps 电机侧，减速比 10
    EXPECT_NEAR(bench.motor.velocity(), 2 * std::numbers::pi / 10, 1e-12);
    // 1024 / 2048 · 33 A · 0.1 N·m/A · 10
    EXPECT_NEAR(bench.motor.torque(), 16.5, 1e-12);
}

TEST(LkMotor, EveryControlReplyIsStatus) {
    for (const std::uint8_t command : {0x9C, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8}) {
        Bench bench{LkMotor::Config{LkMotor::Type::kMG5010Ei10, kTorque, 0x145}};
        EXPECT_TRUE(bench.receive(command, 0, 0, 1000)) << "command 0x" << std::hex << +command;
        EXPECT_EQ(bench.motor.last_raw_angle(), 1000) << "command 0x" << std::hex << +command;
    }
}

TEST(LkMotor, OtherRepliesDoNotCorruptTheTurnCount) {
    Bench bench{
        LkMotor::Config{LkMotor::Type::kMG5010Ei10, kTorque, 0x145}.enable_multi_turn_angle()};
    bench.receive(0xA1, 0, 0, 100);
    const double angle = bench.motor.angle();

    // 同 id、布局不同的回复：读状态 1 / 清错 / 状态 3 / 编码器 / 多圈 / 单圈 / 设零 / 参数 / 关停
    for (const std::uint8_t command :
         {0x9A, 0x9B, 0x9D, 0x90, 0x92, 0x94, 0x19, 0x95, 0xC0, 0xC1, 0x40, 0x42, 0x44, 0x80,
          0x81, 0x88, 0x8C}) {
        // 编码器字段填到半圈外，若被当成状态解码，多圈计数必然跳
        EXPECT_FALSE(bench.receive(command, 0x7FFF, 0x7FFF, 100 + 40000))
            << "command 0x" << std::hex << +command;
        EXPECT_DOUBLE_EQ(bench.motor.angle(), angle) << "command 0x" << std::hex << +command;
        EXPECT_EQ(bench.motor.last_raw_angle(), 100) << "command 0x" << std::hex << +command;
    }
}

TEST(LkMotor, NothingDecodedBeforeFirstFrame) {
    Bench bench{LkMotor::Config{LkMotor::Type::kMG5010Ei10, kTorque, 0x145}
                    .set_encoder_zero_point(9051)
                    .enable_multi_turn_angle()};
    EXPECT_DOUBLE_EQ(bench.motor.angle(), 0.0);

    // 首帧落在零点前一格：多圈计数必须直接从 -1 起，而不是从全零缓冲那一"帧"绕过来
    bench.receive(0xA1, 0, 0, 9050);
    EXPECT_NEAR(bench.motor.angle(), -kCountToRad, 1e-12);
}

// 多圈模式：跨过编码器回绕点时按最短路径累加，正反都一样。
TEST(LkMotor, MultiTurnCountsAcrossTheEncoderWrap) {
    Bench bench{
        LkMotor::Config{LkMotor::Type::kMG5010Ei10, kTorque, 0x145}.enable_multi_turn_angle()};
    bench.receive(0xA1, 0, 0, 65000);
    const double start = bench.motor.angle();

    bench.receive(0xA1, 0, 0, 200); // 向前越过 65535 → 0：+736 格
    EXPECT_NEAR(bench.motor.angle() - start, 736 * kCountToRad, 1e-12);
    bench.receive(0xA1, 0, 0, 65000); // 原路退回
    EXPECT_NEAR(bench.motor.angle(), start, 1e-12);
}

TEST(LkMotor, ReversedZeroPointAndSingleTurnRange) {
    Bench bench{LkMotor::Config{LkMotor::Type::kMG5010Ei10, kTorque, 0x145}
                    .set_encoder_zero_point(9051)
                    .set_reversed()};
    bench.receive(0xA1, 0, 0, 9051);
    EXPECT_DOUBLE_EQ(bench.motor.angle(), 0.0);

    // 反转后零点前一格读作 +1 格，单圈模式落在 [0, 2π)
    bench.receive(0xA1, 0, 0, 9052);
    EXPECT_NEAR(bench.motor.angle(), 2 * std::numbers::pi - kCountToRad, 1e-12);
    bench.receive(0xA1, 0, 0, 9050);
    EXPECT_NEAR(bench.motor.angle(), kCountToRad, 1e-12);
}

TEST(LkMotor, CurrentFullScaleFollowsSeries) {
    // MG：±2048 ↔ ±33 A
    Bench mg{LkMotor::Config{LkMotor::Type::kMG5010Ei10, kTorque, 0x145}};
    mg.receive(0xA1, 2048, 0, 0);
    EXPECT_NEAR(mg.motor.torque(), 33.0 * 0.1 * 10, 1e-9);

    // MF/MH：±2048 ↔ ±16.5 A
    Bench mh{LkMotor::Config{LkMotor::Type::kMHF7015, kTorque, 0x141}};
    mh.receive(0xA1, 2048, 0, 0);
    EXPECT_NEAR(mh.motor.torque(), 16.5 * 0.51, 1e-9);

    // 指令与反馈同一系数：同一个力矩写出去再读回来是同一个计数
    struct [[gnu::packed]] TorqueFrame {
        std::uint8_t id;
        std::uint8_t placeholder[3];
        std::int16_t current;
        std::uint8_t tail[2];
    };
    const auto frame = std::bit_cast<TorqueFrame>(mh.motor.generate_torque_command(16.5 * 0.51 / 2));
    EXPECT_EQ(frame.id, 0xA1);
    EXPECT_EQ(frame.current, 1024);
}

TEST(LkMotor, UnwiredTorqueSendsZeroCurrent) {
    Bench bench{LkMotor::Config{LkMotor::Type::kMG5010Ei10, kTorque, 0x145}};
    auto frame = bench.motor.generate_torque_command();
    const auto bytes = frame.as_bytes();
    EXPECT_EQ(std::to_integer<std::uint8_t>(bytes[0]), 0xA1);
    for (std::size_t i = 1; i < bytes.size(); ++i)
        EXPECT_EQ(std::to_integer<std::uint8_t>(bytes[i]), 0) << "byte " << i;
}
