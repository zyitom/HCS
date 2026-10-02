// 板载 BMI088 的两种解算，以及它们用到的板上时钟展开：
//
//   Bmi088（Mahony）   量程换算；有了重力参考才出姿态；静止不漂；倾斜能收敛；按样本的时间戳积分；
//                      安装方向的换算；间隔过长的样本不用；32 位时间戳回绕不影响
//   Bmi088Ekf          第一个加速度计样本定初始姿态；按样本自己的时间戳积分（与一拍里攒了几个无关）；
//                      时间戳跳变、倒退的样本不用；32 位时间戳回绕不影响
//   BoardClockLifter   回绕、早于 / 晚于时基的时间戳
//
// 驱动只有解算：样本按到达顺序交给 on_sample()，它运行在控制线程上，可以当单线程代码测。
// 样本怎么从 IO 线程过来、多少拍没有样本算掉线，在端口包装层（board::Imu<>），测在 test_board。

#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include <hcs_msgs/board_clock.hpp>
#include <hcs_msgs/imu_snapshot.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/imu_sample.hpp"
#include "hcs_executor/component.hpp"
#include "hcs_executor/wiring.hpp"

namespace {

using hcs_core::hardware::device::Bmi088;
using hcs_core::hardware::device::Bmi088Ekf;
using hcs_core::hardware::device::BoardClockLifter;
using hcs_core::hardware::device::ImuSample;
using hcs_executor::Component;

constexpr double kPi = std::numbers::pi;
constexpr double kGravity = 9.80665;

// 量程：加速度计 ±6 g、陀螺仪 ±2000 °/s，都是 16 位满量程。这里按手册自己算一遍换算。
constexpr double kRawPerG = 32767.0 / 6.0;
constexpr double kRawPerRadPerSec = 32767.0 / (2000.0 * kPi / 180.0);

/// 四分之一微秒。
constexpr std::uint32_t quarter_us(double milliseconds) {
    return static_cast<std::uint32_t>(milliseconds * 4000.0);
}

ImuSample accelerometer(double x_g, double y_g, double z_g, std::uint32_t timestamp = 0) {
    return {
        .kind = ImuSample::Kind::kAccelerometer,
        .x = static_cast<std::int16_t>(std::lround(x_g * kRawPerG)),
        .y = static_cast<std::int16_t>(std::lround(y_g * kRawPerG)),
        .z = static_cast<std::int16_t>(std::lround(z_g * kRawPerG)),
        .timestamp_quarter_us = timestamp};
}

ImuSample gyroscope(double x, double y, double z, std::uint32_t timestamp = 0) {
    return {
        .kind = ImuSample::Kind::kGyroscope,
        .x = static_cast<std::int16_t>(std::lround(x * kRawPerRadPerSec)),
        .y = static_cast<std::int16_t>(std::lround(y * kRawPerRadPerSec)),
        .z = static_cast<std::int16_t>(std::lround(z * kRawPerRadPerSec)),
        .timestamp_quarter_us = timestamp};
}

class Host : public Component {
public:
    explicit Host(std::string name)
        : Component{std::move(name)} {}
    void update(const hcs_sync::Tick&) override {}
};

/// 读 IMU 那组输出：别的组件看到的就是它们。
class Reader : public Component {
public:
    Reader()
        : Component{"reader"} {
        register_input("/imu/quaternion", quaternion);
        register_input("/imu/angular_velocity", angular_velocity);
        register_input("/imu/acceleration", acceleration);
        register_input("/imu/euler/roll", roll);
        register_input("/imu/euler/yaw", yaw);
        register_input("/imu/angular_velocity/z", yaw_rate);
    }
    void update(const hcs_sync::Tick&) override {}

    InputInterface<Eigen::Quaterniond> quaternion;
    InputInterface<Eigen::Vector3d> angular_velocity, acceleration;
    InputInterface<double> roll, yaw, yaw_rate;
};

template <class Driver>
struct Bench {
    explicit Bench(const typename Driver::Config& config)
        : host{std::make_shared<Host>("imu")}
        , imu{*host, "/imu", config}
        , reader{std::make_shared<Reader>()} {
        EXPECT_TRUE(hcs_executor::Linker::link({host, reader}).has_value());
    }

    std::shared_ptr<Host> host;
    Driver imu;
    std::shared_ptr<Reader> reader;
};

Bmi088::Config mahony(double kp = 0.2) { return Bmi088::Config{}.set_gain(kp, 0.0); }

/// 给 Mahony 版喂一段恒定的输入：加速度计一个样本，然后 count 个陀螺仪样本，间隔 0.5 ms。
/// 返回其中让姿态前进了一步的样本数。
int feed(Bmi088& imu, const ImuSample& accel, ImuSample gyro, int count, std::uint32_t start = 0) {
    imu.on_sample(accel);
    int produced = 0;
    for (int i = 0; i < count; ++i) {
        gyro.timestamp_quarter_us = start + quarter_us(0.5 * i);
        produced += imu.on_sample(gyro);
    }
    return produced;
}

} // namespace

// ── Bmi088（Mahony）────────────────────────────────────────────────────────

TEST(Bmi088, ConvertsRawSamplesToPhysicalUnits) {
    Bench<Bmi088> bench{mahony()};
    feed(bench.imu, accelerometer(0.0, 0.0, 1.0), gyroscope(0.1, -0.2, 0.3), 2);

    EXPECT_NEAR(bench.imu.acceleration_g().z(), 1.0, 1e-3);
    EXPECT_NEAR(bench.reader->acceleration->z(), kGravity, 1e-2) << "outputs are in m/s^2";
    EXPECT_NEAR(bench.reader->angular_velocity->x(), 0.1, 1e-3);
    EXPECT_NEAR(bench.reader->angular_velocity->y(), -0.2, 1e-3);
    EXPECT_NEAR(*bench.reader->yaw_rate, 0.3, 1e-3);

    // 满量程。
    bench.imu.on_sample({ImuSample::Kind::kAccelerometer, 32767, 0, 0, 0});
    bench.imu.on_sample({ImuSample::Kind::kGyroscope, 0, 0, -32767, quarter_us(1.0)});
    EXPECT_DOUBLE_EQ(bench.imu.acceleration_g().x(), 6.0);
    EXPECT_NEAR(bench.imu.angular_velocity().z(), -2000.0 * kPi / 180.0, 1e-12);
}

// 加速度计样本到过之前不解算：只有角速度就开始积分的话，姿态是从一个没有重力参考的状态起步的。
// 第一个陀螺仪样本也不算：没有上一个，算不出步长。
TEST(Bmi088, NothingIsSolvedUntilThereIsGravityAndAStepToTake) {
    Bench<Bmi088> bench{mahony()};
    for (int i = 0; i < 100; ++i)
        EXPECT_FALSE(bench.imu.on_sample(gyroscope(0.0, 0.0, 1.0, quarter_us(0.5 * i))));

    EXPECT_TRUE(bench.imu.quaternion().isApprox(Eigen::Quaterniond::Identity()));
    EXPECT_TRUE(std::isnan(*bench.reader->yaw)) << "no attitude yet: not a valid zero";

    EXPECT_FALSE(bench.imu.on_sample(accelerometer(0.0, 0.0, 1.0))) << "not a step by itself";
    EXPECT_TRUE(bench.imu.on_sample(gyroscope(0.0, 0.0, 1.0, quarter_us(50.0))));
    EXPECT_FALSE(std::isnan(*bench.reader->yaw));
}

TEST(Bmi088, LevelAndStillStaysAtIdentity) {
    Bench<Bmi088> bench{mahony()};
    feed(bench.imu, accelerometer(0.0, 0.0, 1.0), gyroscope(0.0, 0.0, 0.0), 5000);

    EXPECT_TRUE(bench.imu.quaternion().isApprox(Eigen::Quaterniond::Identity(), 1e-9));
    EXPECT_NEAR(*bench.reader->roll, 0.0, 1e-9);
    EXPECT_NEAR(*bench.reader->yaw, 0.0, 1e-9);
}

// 机体绕 x 轴滚了 0.3 rad 静止着：加速度计看到的重力方向是 (0, sin, cos)，姿态要收敛到这个滚转角。
TEST(Bmi088, ConvergesToTheTiltGravityShows) {
    constexpr double kRoll = 0.3;
    Bench<Bmi088> bench{mahony(5.0)}; // 增益调大只是为了让测试几秒内收敛
    feed(
        bench.imu, accelerometer(0.0, std::sin(kRoll), std::cos(kRoll)),
        gyroscope(0.0, 0.0, 0.0), 10000);

    EXPECT_NEAR(*bench.reader->roll, kRoll, 1e-3);
    EXPECT_NEAR(*bench.reader->yaw, 0.0, 1e-3);
    EXPECT_NEAR(bench.imu.quaternion().norm(), 1.0, 1e-12);
}

// 水平着以 1 rad/s 绕 z 转一秒：航向走过 1 rad。步长取自样本的时间戳：2001 个样本、间隔 0.5 ms，
// 和这些样本分几拍交进来无关。
TEST(Bmi088, IntegratesAngularVelocityOverSampleTimestamps) {
    Bench<Bmi088> bench{mahony()};
    const int produced =
        feed(bench.imu, accelerometer(0.0, 0.0, 1.0), gyroscope(0.0, 0.0, 1.0), 2001);
    const double rate = bench.imu.angular_velocity().z(); // 量化之后的实际值

    EXPECT_EQ(produced, 2000) << "every gyroscope sample but the first is a step";
    EXPECT_NEAR(*bench.reader->yaw, rate * 1.0, 1e-4);
    EXPECT_NEAR(*bench.reader->roll, 0.0, 1e-6);
}

// 板子怎么装由 sensor_to_body 说：这里传感器的 x 是机体的 z。
TEST(Bmi088, AppliesTheMountingRotationToBothSensors) {
    Eigen::Matrix3d sensor_to_body;
    sensor_to_body << 0, 1, 0, //
        0, 0, 1,               //
        1, 0, 0;
    Bench<Bmi088> bench{mahony().set_sensor_to_body(sensor_to_body)};
    bench.imu.on_sample(accelerometer(1.0, 0.0, 0.0));
    bench.imu.on_sample(gyroscope(0.5, 0.0, 0.0));

    EXPECT_NEAR(bench.imu.acceleration_g().z(), 1.0, 1e-3);
    EXPECT_NEAR(bench.imu.acceleration_g().x(), 0.0, 1e-12);
    EXPECT_NEAR(bench.imu.angular_velocity().z(), 0.5, 1e-3);
}

// 隔了很久才来的陀螺仪样本（掉线之后回来、重连后板上缓冲里吐出来的旧数据）不拿来积分：
// 否则等于凭空转过 角速度 × 间隔 那么大的一个角。
TEST(Bmi088, ALongGapBetweenSamplesIsSkippedNotIntegrated) {
    Bench<Bmi088> bench{mahony()};
    feed(bench.imu, accelerometer(0.0, 0.0, 1.0), gyroscope(0.0, 0.0, 1.0), 201); // 100 ms
    const double yaw = *bench.reader->yaw;
    ASSERT_GT(yaw, 0.05);

    // 500 ms 之后：1 rad/s × 0.5 s，若被积分航向会多出 0.5 rad。
    EXPECT_FALSE(bench.imu.on_sample(gyroscope(0.0, 0.0, 1.0, quarter_us(600.0))));
    EXPECT_DOUBLE_EQ(*bench.reader->yaw, yaw);

    // 从这个样本接着走。
    EXPECT_TRUE(bench.imu.on_sample(gyroscope(0.0, 0.0, 1.0, quarter_us(600.5))));
    EXPECT_NEAR(*bench.reader->yaw - yaw, 0.0005, 1e-5);
}

// 板上的时间戳是 32 位的，约 18 分钟绕一圈：跨过回绕点时步长照样算得对。
TEST(Bmi088, KeepsIntegratingAcrossTheTimestampWrap) {
    Bench<Bmi088> bench{mahony()};
    const std::uint32_t start = 0xFFFF'FFFFU - quarter_us(2.0); // 离回绕还有 2 ms
    const int produced =
        feed(bench.imu, accelerometer(0.0, 0.0, 1.0), gyroscope(0.0, 0.0, 1.0), 21, start);

    EXPECT_EQ(produced, 20);
    EXPECT_NEAR(*bench.reader->yaw, 0.010, 1e-4);
}

// ── BoardClockLifter ───────────────────────────────────────────────────────

TEST(BoardClockLifter, NothingCanBeLiftedBeforeTheTimebaseExists) {
    BoardClockLifter clock;
    EXPECT_FALSE(clock.timebase().has_value());
    EXPECT_FALSE(clock.lift_timestamp(1234).has_value());

    const auto first = clock.advance_timebase(1000);
    EXPECT_EQ(first.time_since_epoch().count(), 1000);
    EXPECT_EQ(clock.timebase(), first);
}

TEST(BoardClockLifter, CountsAcrossThe32BitWrap) {
    BoardClockLifter clock;
    const std::uint32_t near_the_end = 0xFFFF'FF00U;
    const auto before = clock.advance_timebase(near_the_end);
    const auto after = clock.advance_timebase(near_the_end + 0x200U); // 回绕到 0x100

    EXPECT_EQ((after - before).count(), 0x200);
    EXPECT_EQ(after.time_since_epoch().count(), std::int64_t{0xFFFF'FF00} + 0x200);

    // 另一路的时间戳，回绕前后各一个：都展开到和时基同一圈。
    const auto earlier = clock.lift_timestamp(0xFFFF'FFF0U);
    const auto later = clock.lift_timestamp(0x180U);
    ASSERT_TRUE(earlier && later);
    EXPECT_EQ((after - *earlier).count(), 0x110);
    EXPECT_EQ((*later - after).count(), 0x80);
}

// ── Bmi088Ekf ──────────────────────────────────────────────────────────────

TEST(Bmi088Ekf, WaitsForAnAccelerometerSampleBeforeProducingAnything) {
    Bench<Bmi088Ekf> bench{Bmi088Ekf::Config{}};
    EXPECT_FALSE(bench.imu.on_sample(gyroscope(0.0, 0.0, 1.0, quarter_us(1.0))));
    EXPECT_FALSE(bench.imu.initialized());
    EXPECT_FALSE(bench.imu.snapshot().has_value());

    // 加速度计样本只是定初始姿态、推进时基，自己不算"姿态前进了一步"。
    EXPECT_FALSE(bench.imu.on_sample(accelerometer(0.0, 0.0, 1.0, quarter_us(2.0))));
    EXPECT_TRUE(bench.imu.initialized());
    ASSERT_TRUE(bench.imu.snapshot().has_value());
    EXPECT_TRUE(bench.imu.snapshot()->orientation.isApprox(Eigen::Quaterniond::Identity()));

    EXPECT_TRUE(bench.imu.on_sample(gyroscope(0.0, 0.0, 0.0, quarter_us(2.5))));
    EXPECT_EQ(bench.imu.snapshot()->timestamp.time_since_epoch().count(), quarter_us(2.5));
}

// 初始姿态取自第一个加速度计样本看到的重力方向。
TEST(Bmi088Ekf, InitialAttitudeComesFromGravity) {
    constexpr double kRoll = 0.3;
    Bench<Bmi088Ekf> bench{Bmi088Ekf::Config{}};
    bench.imu.on_sample(accelerometer(0.0, std::sin(kRoll), std::cos(kRoll), quarter_us(1.0)));
    ASSERT_TRUE(bench.imu.on_sample(gyroscope(0.0, 0.0, 0.0, quarter_us(1.5))));

    EXPECT_NEAR(*bench.reader->roll, kRoll, 1e-3);
    EXPECT_NEAR(*bench.reader->yaw, 0.0, 1e-3);
    EXPECT_NEAR(bench.reader->acceleration->z(), std::cos(kRoll) * kGravity, 1e-2);
}

// 按样本自己的时间戳积分：1 rad/s 转 1 s，样本间隔 0.5 ms，航向走过 1 rad——
// 不管这些样本是一拍一个交进来的，还是攒了一堆一起交进来的。
TEST(Bmi088Ekf, IntegratesOnSampleTimestamps) {
    Bench<Bmi088Ekf> bench{Bmi088Ekf::Config{}};
    bench.imu.on_sample(accelerometer(0.0, 0.0, 1.0, quarter_us(0.0)));

    double rate = 0.0;
    int produced = 0;
    for (int i = 1; i <= 2000; ++i) {
        const double ms = 0.5 * i;
        if (i % 2 == 0) // 加速度计 1 kHz，时基由它推进
            bench.imu.on_sample(accelerometer(0.0, 0.0, 1.0, quarter_us(ms)));
        produced += bench.imu.on_sample(gyroscope(0.0, 0.0, 1.0, quarter_us(ms)));
        rate = bench.reader->angular_velocity->z();
    }

    EXPECT_EQ(produced, 2000);
    EXPECT_NEAR(*bench.reader->yaw, rate * 1.0, 2e-3);
    EXPECT_NEAR(*bench.reader->roll, 0.0, 1e-3);
    ASSERT_TRUE(bench.imu.snapshot().has_value());
    EXPECT_EQ(bench.imu.snapshot()->timestamp.time_since_epoch().count(), quarter_us(1000.0));
}

// 时间戳一下跳出去很远的陀螺仪样本（重连后板上缓冲里吐出来的旧数据）不拿来积分：
// 否则等于凭空转过 角速度 × 跳变 那么大的一个角。
TEST(Bmi088Ekf, ATimestampJumpIsSkippedNotIntegrated) {
    Bench<Bmi088Ekf> bench{Bmi088Ekf::Config{}};
    bench.imu.on_sample(accelerometer(0.0, 0.0, 1.0, quarter_us(0.0)));
    ASSERT_TRUE(bench.imu.on_sample(gyroscope(0.0, 0.0, 1.0, quarter_us(0.5))));
    const double yaw = *bench.reader->yaw;

    // 500 ms 之后的一个样本：1 rad/s × 0.5 s，若被积分航向会多出 0.5 rad。
    EXPECT_FALSE(bench.imu.on_sample(gyroscope(0.0, 0.0, 1.0, quarter_us(500.5))));
    EXPECT_DOUBLE_EQ(*bench.reader->yaw, yaw);

    // 从跳变之后的时刻接着走。
    EXPECT_TRUE(bench.imu.on_sample(gyroscope(0.0, 0.0, 1.0, quarter_us(501.0))));
    EXPECT_NEAR(*bench.reader->yaw - yaw, 0.0005, 1e-5);

    // 时间倒退的样本也不用。
    EXPECT_FALSE(bench.imu.on_sample(gyroscope(0.0, 0.0, 1.0, quarter_us(400.0))));
}

// 板上的时间戳是 32 位的，约 18 分钟绕一圈：跨过回绕点时积分照常。
TEST(Bmi088Ekf, KeepsIntegratingAcrossTheTimestampWrap) {
    Bench<Bmi088Ekf> bench{Bmi088Ekf::Config{}};
    const std::uint32_t start = 0xFFFF'FFFFU - quarter_us(2.0); // 离回绕还有 2 ms
    bench.imu.on_sample(accelerometer(0.0, 0.0, 1.0, start));

    int produced = 0;
    for (int i = 1; i <= 20; ++i) { // 10 ms，中途回绕
        const std::uint32_t timestamp = start + quarter_us(0.5 * i);
        if (i % 2 == 0)
            bench.imu.on_sample(accelerometer(0.0, 0.0, 1.0, timestamp));
        produced += bench.imu.on_sample(gyroscope(0.0, 0.0, 1.0, timestamp));
    }

    EXPECT_EQ(produced, 20);
    EXPECT_NEAR(*bench.reader->yaw, 0.010, 1e-4);
}

// 带板上时刻的快照也作为输出发布，给要和同一块板上别的时间戳（相机触发）对齐的消费者。
TEST(Bmi088Ekf, PublishesTheTimestampedSnapshot) {
    class SnapshotReader : public Component {
    public:
        SnapshotReader()
            : Component{"snapshot_reader"} {
            register_input("/imu/snapshot", snapshot);
        }
        void update(const hcs_sync::Tick&) override {}
        InputInterface<hcs_msgs::ImuSnapshot> snapshot;
    };

    auto host = std::make_shared<Host>("imu");
    Bmi088Ekf imu{*host, "/imu", {}};
    auto reader = std::make_shared<SnapshotReader>();
    ASSERT_TRUE(hcs_executor::Linker::link({host, reader}).has_value());

    imu.on_sample(accelerometer(0.0, 0.0, 1.0, quarter_us(1.0)));
    ASSERT_TRUE(imu.on_sample(gyroscope(0.0, 0.2, 0.0, quarter_us(1.5))));

    EXPECT_EQ(reader->snapshot->timestamp.time_since_epoch().count(), quarter_us(1.5));
    EXPECT_NEAR(reader->snapshot->gyro_body.y(), 0.2, 1e-3);
    EXPECT_TRUE(reader->snapshot->orientation.isApprox(imu.snapshot()->orientation));
}
