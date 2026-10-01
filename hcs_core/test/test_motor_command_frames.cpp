// 三种电机驱动的指令帧：显式控制模式、统一失能规则、DJI 合帧。
//
// 期望字节全部录自重构前的实际输出（同样的电机配置照抄 balance_infantry.cpp 的接线表，
// 按改造前 BalanceInfantry::pack_frames 的规则：DM 力矩 NaN 或 safe 发 0xFD、否则
// generate_command()；LK 走 generate_torque_command()；DJI 同 send_id 合帧）。
// 唯一有意的差异是 pitch：它的 /control_velocity 上游是角度环输出，改造前 DM 驱动见它接了
// 就按"速度形状"编 MIT 帧（kd = 0 时物理上无效，但字段里带着速度）；力矩模式下不再注册
// 这个输入，速度字段回到中点。

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>

#include <gtest/gtest.h>

#include <hcs_executor/component.hpp>
#include <hcs_executor/wiring.hpp>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/util/board_transmitter.hpp"

namespace {

using namespace hcs_core::hardware::device;
using hcs_core::hardware::util::CommandFrames;
using hcs_core::hardware::util::TransmitBatch;
using hcs_executor::Component;
using libhcs::board::hcs::CanPort;
using Bytes = std::array<std::uint8_t, 8>;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr auto kLkTorque = LkMotor::ControlMode::kTorque;

class Host : public Component {
public:
    explicit Host(std::string name)
        : Component{std::move(name)} {}
    void update(const hcs_sync::Tick&) override {}
};

/// 上游控制器：按名字给出指令输出。
class Producer : public Component {
public:
    Producer()
        : Component{"producer"} {}
    std::size_t add(const std::string& name) {
        register_output(name, outputs_[count_], kNan);
        return count_++;
    }
    void set(std::size_t index, double value) { *outputs_[index] = value; }
    void update(const hcs_sync::Tick&) override {}

private:
    std::array<OutputInterface<double>, 8> outputs_;
    std::size_t count_ = 0;
};

Bytes bytes_of(CanPacket8 packet) {
    Bytes out{};
    const auto view = packet.as_bytes();
    for (std::size_t i = 0; i < out.size(); ++i)
        out[i] = std::to_integer<std::uint8_t>(view[i]);
    return out;
}

/// 一台电机一拍发出的唯一一帧（独占帧驱动）。
template <class Motor>
Bytes single_frame(Motor& motor, bool safe) {
    TransmitBatch batch;
    CommandFrames frames{batch};
    auto bus = frames.on(CanPort::kCan1);
    motor.append_command(bus, safe);
    frames.flush();
    EXPECT_EQ(batch.frame_count, 1U);
    EXPECT_EQ(batch.frames[0].can_id, motor.send_id());
    return bytes_of(CanPacket8{std::span<const std::byte, 8>{batch.frames[0].data}});
}

DmMotor::Config leg_joint() {
    DmMotor::Config config{
        DmMotor::ControlMode::kTorque, 0x02, 0x06, DmMotor::MitRange{6.283185, 45.0, 40.0}};
    config.set_zero_angle(0.717242718)
        .enable_multi_turn_angle();
    return config;
}

DmMotor::Config pitch(DmMotor::ControlMode mode = DmMotor::ControlMode::kTorque) {
    return DmMotor::Config{mode, 0x04, 0x03, DmMotor::MitRange{12.566, 30.0, 40.0}}
        .set_zero_angle(0.0285701752)
        .set_reversed();
}

void dm_feedback(DmMotor& motor, std::uint32_t esc_id, std::uint32_t master_id, std::uint8_t err) {
    const std::array<std::uint8_t, 8> frame{
        static_cast<std::uint8_t>((esc_id & 0x0F) | (err << 4)), 0x80, 0x00, 0x80, 0x07, 0xFF, 30,
        30};
    motor.match_then_store_status(master_id, std::as_bytes(std::span{frame}));
    motor.update_status();
}

constexpr Bytes kDmEnable{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
constexpr Bytes kDmDisable{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
constexpr Bytes kDmClearError{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};
constexpr Bytes kLkZeroCurrent{0xa1, 0, 0, 0, 0, 0, 0, 0};

} // namespace

// ── DM ─────────────────────────────────────────────────────────────────────

TEST(DmCommand, TorqueModeMatchesPreRefactorFrames) {
    auto host = std::make_shared<Host>("host");
    auto producer = std::make_shared<Producer>();
    const auto torque = producer->add("/leg/control_torque");
    DmMotor leg{*host, *host, "/leg", leg_joint()};
    ASSERT_TRUE(hcs_executor::Linker::link({host, producer}).has_value());
    producer->set(torque, 3.2);

    EXPECT_EQ(single_frame(leg, false), kDmEnable)
        << "power-on default state must be enabled first";
    dm_feedback(leg, 0x02, 0x06, 0x1);
    EXPECT_EQ(single_frame(leg, false), (Bytes{0x8e, 0x9c, 0x80, 0x00, 0x00, 0x00, 0x08, 0xa3}));
    EXPECT_EQ(single_frame(leg, true), kDmDisable);
    producer->set(torque, kNan);
    EXPECT_EQ(single_frame(leg, false), kDmDisable) << "NaN setpoint = do not command";

    producer->set(torque, 3.2);
    dm_feedback(leg, 0x02, 0x06, 0xB);
    EXPECT_EQ(single_frame(leg, false), kDmClearError);
    EXPECT_EQ(single_frame(leg, false), kDmDisable) << "backs off between clear-error attempts";
}

TEST(DmCommand, TorqueModeIgnoresAnUpstreamVelocitySignal) {
    auto host = std::make_shared<Host>("host");
    auto producer = std::make_shared<Producer>();
    const auto torque = producer->add("/pitch/control_torque");
    const auto velocity = producer->add("/pitch/control_velocity"); // 角度环输出，给上位机速度环
    DmMotor motor{*host, *host, "/pitch", pitch()};
    ASSERT_TRUE(hcs_executor::Linker::link({host, producer}).has_value());
    producer->set(torque, -0.8);
    producer->set(velocity, 1.7);

    dm_feedback(motor, 0x04, 0x03, 0x1);
    // 改造前是 80 4a 78 b0 ...（速度字段带着 1.7）；力矩模式下速度字段是中点 80 0。
    EXPECT_EQ(single_frame(motor, false), (Bytes{0x80, 0x4a, 0x80, 0x00, 0x00, 0x00, 0x08, 0x28}));
}

TEST(DmCommand, VelocityModeIsExplicitAndUsesKd) {
    auto host = std::make_shared<Host>("host");
    auto producer = std::make_shared<Producer>();
    const auto velocity = producer->add("/joint/control_velocity");
    DmMotor motor{
        *host, *host, "/joint",
        pitch(DmMotor::ControlMode::kVelocity).set_gain(0.0, 1.0)};
    ASSERT_TRUE(hcs_executor::Linker::link({host, producer}).has_value());
    EXPECT_EQ(motor.control_mode(), DmMotor::ControlMode::kVelocity);

    dm_feedback(motor, 0x04, 0x03, 0x1);
    producer->set(velocity, 1.7);
    const auto frame = single_frame(motor, false);
    const unsigned raw_velocity = (frame[2] << 4) | (frame[3] >> 4);
    const unsigned raw_kp = ((frame[3] & 0x0F) << 8) | frame[4];
    const unsigned raw_kd = (frame[5] << 4) | (frame[6] >> 4);
    EXPECT_NE(raw_velocity, 0x7FFU) << "velocity setpoint must reach the frame";
    EXPECT_EQ(raw_kp, 0U) << "no position loop in velocity mode";
    EXPECT_GT(raw_kd, 0U) << "kd from the config closes the velocity loop";

    producer->set(velocity, kNan);
    EXPECT_EQ(single_frame(motor, false), kDmDisable);
}

// ── LK ─────────────────────────────────────────────────────────────────────

TEST(LkCommand, TorqueModeMatchesPreRefactorFrames) {
    auto host = std::make_shared<Host>("host");
    auto producer = std::make_shared<Producer>();
    const auto torque = producer->add("/yaw/control_torque");
    const auto velocity = producer->add("/yaw/control_velocity"); // 同 pitch：给上位机速度环的
    LkMotor yaw{
        *host, *host, "/yaw",
        LkMotor::Config{LkMotor::Type::kMG5010Ei10, kLkTorque, 0x145}
            .set_encoder_zero_point(9051)
            .set_reversed()};
    LkMotor dial{
        *host, *host, "/dial", LkMotor::Config{LkMotor::Type::kMG5010Ei10, kLkTorque, 0x144}};
    ASSERT_TRUE(hcs_executor::Linker::link({host, producer}).has_value());
    producer->set(torque, 0.7);
    producer->set(velocity, 2.5);

    // 改造前靠整车组件绕开 generate_command() 才拿到 0xA1；现在力矩模式本身就只发 0xA1。
    EXPECT_EQ(single_frame(yaw, false), (Bytes{0xa1, 0x00, 0x00, 0x00, 0xd5, 0xff, 0x00, 0x00}));
    EXPECT_EQ(bytes_of(yaw.generate_command())[0], 0xA1)
        << "velocity wiring must not switch the mode";
    EXPECT_EQ(single_frame(yaw, true), kLkZeroCurrent);
    producer->set(torque, kNan);
    EXPECT_EQ(single_frame(yaw, false), kLkZeroCurrent);
    EXPECT_EQ(single_frame(dial, false), kLkZeroCurrent) << "unwired = zero current";
}

TEST(LkCommand, VelocityModeIsExplicit) {
    auto host = std::make_shared<Host>("host");
    auto producer = std::make_shared<Producer>();
    const auto velocity = producer->add("/yaw/control_velocity");
    LkMotor yaw{
        *host, *host, "/yaw",
        LkMotor::Config{LkMotor::Type::kMG5010Ei10, LkMotor::ControlMode::kVelocity, 0x145}};
    ASSERT_TRUE(hcs_executor::Linker::link({host, producer}).has_value());
    producer->set(velocity, 2.5);
    EXPECT_EQ(single_frame(yaw, false)[0], 0xA2);
    producer->set(velocity, kNan);
    EXPECT_EQ(single_frame(yaw, false), kLkZeroCurrent);
}

// ── DJI 与合帧 ─────────────────────────────────────────────────────────────

TEST(DjiCommand, SharedFrameMatchesPreRefactorFrames) {
    auto host = std::make_shared<Host>("host");
    auto producer = std::make_shared<Producer>();
    const auto left_torque = producer->add("/left_wheel/control_torque");
    const auto right_torque = producer->add("/right_wheel/control_torque");
    DjiMotor left{
        *host, *host, "/left_wheel",
        DjiMotor::Config{DjiMotor::Type::kM3508, 2}.set_reduction_ratio(13.94)};
    DjiMotor right{
        *host, *host, "/right_wheel",
        DjiMotor::Config{DjiMotor::Type::kM3508, 1}.set_reduction_ratio(13.94).set_reversed()};
    DjiMotor friction_left{*host, *host, "/fl", DjiMotor::Config{DjiMotor::Type::kM3508, 1}};
    DjiMotor friction_right{*host, *host, "/fr", DjiMotor::Config{DjiMotor::Type::kM3508, 2}};
    ASSERT_TRUE(hcs_executor::Linker::link({host, producer}).has_value());

    const auto pack = [](bool safe, auto&... motors) {
        TransmitBatch batch;
        CommandFrames frames{batch};
        auto bus = frames.on(CanPort::kCan1);
        (motors.append_command(bus, safe), ...);
        frames.flush();
        EXPECT_EQ(batch.frame_count, 1U) << "same bus, same send id: one frame";
        EXPECT_EQ(batch.frames[0].can_id, 0x200U);
        return bytes_of(CanPacket8{std::span<const std::byte, 8>{batch.frames[0].data}});
    };

    producer->set(left_torque, 1.1);
    producer->set(right_torque, -0.6);
    EXPECT_EQ(pack(false, left, right), (Bytes{0x08, 0xd1, 0x10, 0x2a, 0, 0, 0, 0}));
    EXPECT_EQ(pack(true, left, right), (Bytes{}));
    producer->set(left_torque, kNan);
    EXPECT_EQ(pack(false, left, right), (Bytes{0x08, 0xd1, 0, 0, 0, 0, 0, 0}));
    EXPECT_EQ(pack(false, friction_left, friction_right), (Bytes{}));
}

TEST(CommandFrames, ExclusiveFramesFirstThenSharedPerBus) {
    auto host = std::make_shared<Host>("host");
    auto producer = std::make_shared<Producer>();
    producer->add("/leg/control_torque");
    DmMotor leg{*host, *host, "/leg", leg_joint()};
    LkMotor yaw{
        *host, *host, "/yaw", LkMotor::Config{LkMotor::Type::kMG5010Ei10, kLkTorque, 0x145}};
    DjiMotor a{*host, *host, "/a", DjiMotor::Config{DjiMotor::Type::kM3508, 1}};
    DjiMotor b{*host, *host, "/b", DjiMotor::Config{DjiMotor::Type::kM3508, 2}};
    DjiMotor c{*host, *host, "/c", DjiMotor::Config{DjiMotor::Type::kM3508, 1}};
    ASSERT_TRUE(hcs_executor::Linker::link({host, producer}).has_value());

    TransmitBatch batch;
    CommandFrames frames{batch};
    auto can1 = frames.on(CanPort::kCan1);
    auto can2 = frames.on(CanPort::kCan2);
    a.append_command(can1, false);
    leg.append_command(can1, false);
    b.append_command(can1, false);
    yaw.append_command(can2, false);
    c.append_command(can2, false); // 同 send_id，不同总线：另起一帧
    frames.flush();

    ASSERT_EQ(batch.frame_count, 4U);
    EXPECT_EQ(batch.frames[0].can_id, leg.send_id());
    EXPECT_EQ(batch.frames[1].can_id, yaw.send_id());
    EXPECT_EQ(batch.frames[2].can_id, 0x200U);
    EXPECT_EQ(batch.frames[2].port, static_cast<std::uint8_t>(CanPort::kCan1));
    EXPECT_EQ(batch.frames[3].can_id, 0x200U);
    EXPECT_EQ(batch.frames[3].port, static_cast<std::uint8_t>(CanPort::kCan2));
}
