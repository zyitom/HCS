// DJI 驱动反馈 DATA[7] 错误码的单测：
//   - 新固件把错误码放在 DATA[7]，旧固件该字节为 null（0），两者都解成 kNone
//   - 手册列出的码原样解出，手册没列的码（6、9 以上）不被折成某个已知码
//   - 错误码不影响同帧其余字段的解码

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <span>
#include <string>

#include <gtest/gtest.h>

#include "hardware/device/dji_motor.hpp"
#include "hcs_executor/component.hpp"
#include "hcs_executor/wiring.hpp"

namespace {

using hcs_core::hardware::device::DjiMotor;
using hcs_executor::Component;

class Host : public Component {
public:
    explicit Host(std::string name)
        : Component{std::move(name)} {}
    void update(const hcs_sync::Tick&) override {}
};

struct Bench {
    Bench()
        : host{std::make_shared<Host>("dji")}
        , motor{*host, *host, "/wheel", DjiMotor::Config{DjiMotor::Type::kM3508, 1}} {
        EXPECT_TRUE(hcs_executor::Linker::link({host}).has_value());
    }

    // DJI 反馈：角度/转速/电流为大端 int16，DATA[6] 温度，DATA[7] 错误码
    void store(std::uint16_t angle, std::int16_t rpm, std::uint8_t temperature,
               std::uint8_t error) {
        const auto speed = static_cast<std::uint16_t>(rpm);
        const std::array<std::uint8_t, 8> frame{
            static_cast<std::uint8_t>(angle >> 8), static_cast<std::uint8_t>(angle),
            static_cast<std::uint8_t>(speed >> 8), static_cast<std::uint8_t>(speed),
            0, 0, temperature, error};
        motor.store_status(std::as_bytes(std::span{frame}));
    }

    void receive(std::uint16_t angle, std::int16_t rpm, std::uint8_t temperature,
                 std::uint8_t error) {
        store(angle, rpm, temperature, error);
        motor.update_status();
    }

    std::shared_ptr<Host> host;
    DjiMotor motor;
};

} // namespace

TEST(DjiMotor, NothingDecodedBeforeFirstFrame) {
    Bench bench;
    for (int i = 0; i < 5; ++i)
        bench.motor.update_status();
    EXPECT_FALSE(bench.motor.received());
    EXPECT_FALSE(bench.motor.online());
    EXPECT_DOUBLE_EQ(bench.motor.velocity(), 0.0);
}

// 电调 1 kHz 自发反馈，与控制拍不同源：有的拍收到两帧、有的拍一帧都没有。
// 看门狗只数"连续没有新帧的拍"，零星的空拍不算掉线。
TEST(DjiMotor, FreeRunningFeedbackJitterStaysOnline) {
    Bench bench;
    bench.receive(0, 0, 30, 0);
    for (int cycle = 0; cycle < 1000; ++cycle) {
        if (cycle % 7 == 3)
            ; // 这一拍没收到
        else if (cycle % 11 == 5) {
            bench.store(0, 0, 30, 0); // 这一拍收到两帧
            bench.store(0, 0, 30, 0);
        } else
            bench.store(0, 0, 30, 0);
        bench.motor.update_status();
        ASSERT_TRUE(bench.motor.online()) << "cycle " << cycle;
    }
}

TEST(DjiMotor, GoesOfflineAfterTimeoutAndBack) {
    Bench bench;
    bench.receive(0, 0, 30, 0);
    for (int i = 0; i < 99; ++i)
        bench.motor.update_status();
    EXPECT_TRUE(bench.motor.online());
    bench.motor.update_status();
    EXPECT_FALSE(bench.motor.online());

    bench.receive(0, 0, 30, 0);
    EXPECT_TRUE(bench.motor.online());
}

TEST(DjiMotor, OverheatWarnsButIsNotAFault) {
    Bench bench;
    bench.receive(0, 0, 130, static_cast<std::uint8_t>(DjiMotor::Error::kMotorOverheat));
    EXPECT_FALSE(bench.motor.faulted());
    bench.receive(0, 0, 30, static_cast<std::uint8_t>(DjiMotor::Error::kPhaseDisconnected));
    EXPECT_TRUE(bench.motor.faulted());
    bench.receive(0, 0, 30, 6); // 手册没列的码按故障处理
    EXPECT_TRUE(bench.motor.faulted());
    bench.receive(0, 0, 30, 0);
    EXPECT_FALSE(bench.motor.faulted());
}

TEST(DjiMotor, OldFirmwareNullByteIsNoError) {
    Bench bench;
    bench.receive(0, 0, 30, 0);
    EXPECT_EQ(bench.motor.error(), DjiMotor::Error::kNone);
}

TEST(DjiMotor, DocumentedCodesDecodeAsIs) {
    for (const auto error :
         {DjiMotor::Error::kStorageUnreachable, DjiMotor::Error::kSupplyOverVoltage,
          DjiMotor::Error::kPhaseDisconnected, DjiMotor::Error::kPositionSensorLost,
          DjiMotor::Error::kMotorOverTemperature, DjiMotor::Error::kCalibrationFailed,
          DjiMotor::Error::kMotorOverheat}) {
        Bench bench;
        bench.receive(0, 0, 30, static_cast<std::uint8_t>(error));
        EXPECT_EQ(bench.motor.error(), error);
    }
}

TEST(DjiMotor, UndocumentedCodesAreKept) {
    for (const std::uint8_t code : {6, 9, 0xFF}) {
        Bench bench;
        bench.receive(0, 0, 30, code);
        EXPECT_EQ(static_cast<std::uint8_t>(bench.motor.error()), code);
    }
}

TEST(DjiMotor, ErrorByteDoesNotDisturbOtherFields) {
    Bench bench;
    bench.receive(2048, 600, 125, static_cast<std::uint8_t>(DjiMotor::Error::kMotorOverheat));
    EXPECT_EQ(bench.motor.error(), DjiMotor::Error::kMotorOverheat);
    EXPECT_DOUBLE_EQ(bench.motor.temperature(), 125.0);
    EXPECT_EQ(bench.motor.last_raw_angle(), 2048);
    // 600 rpm 转子侧 / 3591:187
    EXPECT_NEAR(bench.motor.velocity(), 600.0 / 60 * 2 * std::numbers::pi * 187.0 / 3591.0, 1e-12);

    // 故障消失后下一帧回到 kNone，驱动不锁存
    bench.receive(2048, 600, 90, 0);
    EXPECT_EQ(bench.motor.error(), DjiMotor::Error::kNone);
}
