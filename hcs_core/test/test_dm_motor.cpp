// DM 驱动的单测，钉住这次补的两件事和上次加的零点换算：
//   - D[0] 低 4 位必须是本电机 ESC_ID：两台电机 MST_ID 刷成一样时，另一台的反馈被丢掉并计数，
//     不能让关节角在两台电机之间跳
//   - 首帧前不解码、离线看门狗
//   - set_zero_angle 按当前 PMAX 反解 raw，同一个物理零点换 PMAX 后仍读零

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

#include <gtest/gtest.h>

#include "hardware/device/dm_motor.hpp"
#include "hcs_executor/component.hpp"
#include "hcs_executor/wiring.hpp"

namespace {

using hcs_core::hardware::device::DmMotor;
using hcs_executor::Component;

class Host : public Component {
public:
    explicit Host(std::string name)
        : Component{std::move(name)} {}
    void update(const hcs_sync::Tick&) override {}
};

struct Bench {
    explicit Bench(const DmMotor::Config& config)
        : host{std::make_shared<Host>("dm")}
        , motor{*host, *host, "/joint", config} {
        EXPECT_TRUE(hcs_executor::Linker::link({host}).has_value());
    }

    /// 协议 V1.4 反馈帧：D[0] = ID | ERR<<4，POS 16 位高位在前，VEL/T 12 位，D[6]/D[7] 温度
    bool receive(std::uint32_t can_id, std::uint8_t reported_id, std::uint8_t err,
                 std::uint16_t position) {
        const std::array<std::uint8_t, 8> frame{
            static_cast<std::uint8_t>((reported_id & 0x0F) | (err << 4)),
            static_cast<std::uint8_t>(position >> 8),
            static_cast<std::uint8_t>(position),
            0x80, 0x07, 0xFF, 30, 30}; // 速度、力矩取中点 = 0
        return motor.match_then_store_status(can_id, std::as_bytes(std::span{frame}));
    }

    std::shared_ptr<Host> host;
    DmMotor motor;
};

DmMotor::Config j4310(std::uint32_t esc_id, std::uint32_t master_id) {
    return DmMotor::Config{
        DmMotor::ControlMode::kTorque, esc_id, master_id, DmMotor::kJ4310Factory};
}

// raw → rad，按协议 V1.4 图 2：raw 0 ↔ -PMAX，raw 65535 ↔ +PMAX
double decode(std::uint16_t raw, double pmax) { return raw * 2 * pmax / 65535 - pmax; }

} // namespace

TEST(DmMotor, AcceptsOwnFeedback) {
    Bench bench{j4310(0x04, 0x14)};
    EXPECT_TRUE(bench.receive(0x14, 0x04, 1, 40000));
    bench.motor.update_status();
    EXPECT_TRUE(bench.motor.received());
    EXPECT_TRUE(bench.motor.online());
    EXPECT_TRUE(bench.motor.enabled());
    EXPECT_EQ(bench.motor.last_raw_angle(), 40000);
    EXPECT_EQ(bench.motor.foreign_frame_count(), 0u);
}

TEST(DmMotor, RejectsAnotherMotorOnSharedMasterId) {
    // 0x02 与 0x05 两台电机的 MST_ID 都被刷成了 0x14
    Bench bench{j4310(0x02, 0x14)};
    EXPECT_TRUE(bench.receive(0x14, 0x02, 1, 30000));
    bench.motor.update_status();
    ASSERT_EQ(bench.motor.last_raw_angle(), 30000);

    // 另一台的帧：帧 id 是我们的，所以认领（返回 true，不让别人解），但不解码
    EXPECT_TRUE(bench.receive(0x14, 0x05, 1, 60000));
    bench.motor.update_status();
    EXPECT_EQ(bench.motor.last_raw_angle(), 30000);
    EXPECT_EQ(bench.motor.foreign_frame_count(), 1u);
}

TEST(DmMotor, ForeignFramesDoNotKeepMotorOnline) {
    Bench bench{j4310(0x02, 0x14).set_offline_timeout(3)};
    bench.receive(0x14, 0x02, 1, 30000);
    bench.motor.update_status();
    // 本电机没了，只剩另一台还在这个 id 上发
    for (int i = 0; i < 3; ++i) {
        bench.receive(0x14, 0x05, 1, 60000);
        bench.motor.update_status();
    }
    EXPECT_FALSE(bench.motor.online());
}

TEST(DmMotor, OnlyLowNibbleOfEscIdIsCompared) {
    // ESC_ID 0x12 在 D[0] 里只剩 0x2（高 4 位是 ERR）
    Bench bench{j4310(0x12, 0x22)};
    EXPECT_TRUE(bench.receive(0x22, 0x02, 1, 12345));
    bench.motor.update_status();
    EXPECT_EQ(bench.motor.last_raw_angle(), 12345);
    EXPECT_EQ(bench.motor.foreign_frame_count(), 0u);
}

TEST(DmMotor, OtherFrameIdIsNotClaimed) {
    Bench bench{j4310(0x02, 0x14)};
    EXPECT_FALSE(bench.receive(0x15, 0x02, 1, 12345));
    EXPECT_EQ(bench.motor.foreign_frame_count(), 0u);
}

TEST(DmMotor, FaultedIsNeitherEnableState) {
    Bench bench{j4310(0x02, 0x14)};
    bench.receive(0x14, 0x02, 0x0, 32768);
    bench.motor.update_status();
    EXPECT_FALSE(bench.motor.faulted()); // 失能

    bench.receive(0x14, 0x02, 0x1, 32768);
    bench.motor.update_status();
    EXPECT_FALSE(bench.motor.faulted()); // 使能

    bench.receive(0x14, 0x02, 0xA, 32768);
    bench.motor.update_status();
    EXPECT_TRUE(bench.motor.faulted()); // 过流
}

TEST(DmMotor, NothingDecodedBeforeFirstFrame) {
    Bench bench{j4310(0x02, 0x14).enable_multi_turn_angle().set_zero_angle(0.7)};
    for (int i = 0; i < 5; ++i)
        bench.motor.update_status();
    EXPECT_FALSE(bench.motor.received());
    EXPECT_FALSE(bench.motor.online());
    EXPECT_DOUBLE_EQ(bench.motor.angle(), 0.0);
}

// 同一个物理零点（驱动自己坐标系里的 org_pos 弧度），不管 PMAX 是多少都读作 0
TEST(DmMotor, ZeroAngleFollowsPositionMax) {
    constexpr double kOrgPos = 0.717242718;
    for (const double pmax : {6.283185, 12.5, 12.566}) {
        auto config = DmMotor::Config{
            DmMotor::ControlMode::kTorque, 0x02, 0x14, DmMotor::MitRange{pmax, 30.0, 10.0}};
        Bench bench{config.set_zero_angle(kOrgPos)};
        const auto raw = static_cast<std::uint16_t>(std::lround((kOrgPos + pmax) * 65535 / (2 * pmax)));
        ASSERT_NEAR(decode(raw, pmax), kOrgPos, pmax / 65535) << "pmax " << pmax;

        bench.receive(0x14, 0x02, 1, raw);
        bench.motor.update_status();
        EXPECT_NEAR(bench.motor.angle(), 0.0, 1e-12) << "pmax " << pmax;
    }
}
