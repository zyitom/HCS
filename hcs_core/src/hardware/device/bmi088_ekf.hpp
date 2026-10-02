#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <numbers>
#include <optional>
#include <string>

#include <eigen3/Eigen/Geometry>

#include <hcs_executor/component.hpp>
#include <hcs_msgs/board_clock.hpp>
#include <hcs_msgs/imu_snapshot.hpp>

#include "filter/imu_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/imu_outputs.hpp"
#include "hardware/device/imu_sample.hpp"

namespace hcs_core::hardware::device {

/// 板载 BMI088，姿态用扩展卡尔曼滤波解算，**按样本自己的板上时间戳**逐个积分。
///
/// 和 Bmi088（Mahony，每拍一步、步长固定）的区别就在这里：陀螺仪每来一个样本就预测一步，
/// 步长是相邻两个样本时间戳之差，所以控制线程偶尔晚了、一拍里攒了几个样本，姿态也不受影响。
/// 输出里还多一个带板上时刻的快照（<名字>/snapshot），给要和同一块板上别的时间戳（相机触发）
/// 对齐的消费者。
///
/// 只写协议与解算：全部成员都在控制线程上，样本由端口包装层（board::Imu<>）按到达顺序交进来。
///
/// 航向没有参考（没有磁力计），只靠陀螺仪积分，会漂。
class Bmi088Ekf {
public:
    using TimePoint = hcs_msgs::BoardClock::time_point;
    using Snapshot = hcs_msgs::ImuSnapshot;

    struct Config {
        Config& set_body_to_sensor(const Eigen::Matrix3d& value) {
            return body_to_sensor = value, *this;
        }
        Config& set_offline_timeout(int value) { return offline_timeout = value, *this; }

        /// 机体系（FLU）到传感器轴的旋转：sensor = body_to_sensor * body。由板子怎么装决定。
        Eigen::Matrix3d body_to_sensor = Eigen::Matrix3d::Identity();

        /// 连续多少拍没有解出新的姿态算掉线。
        int offline_timeout = 100;
    };

    Bmi088Ekf(
        hcs_executor::Component& status_component, const std::string& name, const Config& config)
        : outputs_(status_component, name)
        , sensor_to_body_(config.body_to_sensor.transpose()) {
        status_component.register_output(name + "/snapshot", snapshot_output_, Snapshot{});
    }

    Bmi088Ekf(const Bmi088Ekf&) = delete;
    Bmi088Ekf& operator=(const Bmi088Ekf&) = delete;

    /// 周期域：一个原始样本，按到达顺序。
    ///
    /// 加速度计的时间戳推进时基，陀螺仪的时间戳靠时基展开（见 BoardClockLifter）。
    /// @return 这个样本有没有让姿态前进一步。只有陀螺仪样本会；滤波器还没初始化、
    ///         或者样本被丢掉（时间倒退、时间戳跳变）时返回 false
    bool on_sample(const ImuSample& sample) noexcept {
        if (sample.kind == ImuSample::Kind::kAccelerometer) {
            push_accelerometer(sample, clock_.advance_timebase(sample.timestamp_quarter_us));
            return false;
        }

        const auto time = clock_.lift_timestamp(sample.timestamp_quarter_us);
        if (!time)
            return false; // 还没有加速度计样本，时基没建立
        const auto snapshot = update_with_gyroscope(sample, *time);
        if (!snapshot)
            return false;

        latest_snapshot_ = *snapshot;
        *snapshot_output_ = *snapshot;
        outputs_.publish(
            {.quaternion = snapshot->orientation,
             .angular_velocity = snapshot->gyro_body,
             .acceleration = acceleration_g_ * kGravity,
             .euler_angles = ImuOutputs::euler_from(snapshot->orientation)});
        return true;
    }

    /// 滤波器有没有从第一个加速度计样本拿到初始姿态。
    [[nodiscard]] bool initialized() const noexcept { return initialized_; }

    /// 最近一次解算的结果；还没初始化则为空。
    [[nodiscard]] std::optional<Snapshot> snapshot() const noexcept {
        if (!initialized_)
            return std::nullopt;
        return latest_snapshot_;
    }

    [[nodiscard]] std::string describe() const { return "onboard BMI088, EKF on board timestamps"; }

private:
    static constexpr double kGravity = 9.80665; // m/s²

    /// 陀螺仪两个样本之间最多隔这么久，再长就认为时间戳跳了（见 update_with_gyroscope）。
    static constexpr double kMaxGyroscopeStep = 1.0 / 1000.0; // 秒

    [[nodiscard]] Eigen::Vector3d accelerometer_g(const ImuSample& sample) const noexcept {
        const Eigen::Vector3d raw{
            static_cast<double>(sample.x), static_cast<double>(sample.y),
            static_cast<double>(sample.z)};
        return sensor_to_body_ * raw / 32767.0 * 6.0;
    }

    [[nodiscard]] Eigen::Vector3d gyroscope_rad_per_sec(const ImuSample& sample) const noexcept {
        const Eigen::Vector3d raw{
            static_cast<double>(sample.x), static_cast<double>(sample.y),
            static_cast<double>(sample.z)};
        return sensor_to_body_ * raw / 32767.0 * 2000.0 / 180.0 * std::numbers::pi;
    }

    /// 陀螺仪有没有哪一轴打到了量程的 98%。
    /// 三个分量手写着比，不用 cwiseAbs().maxCoeff()：后者 clang 推不出"不阻塞"。
    [[nodiscard]] static bool is_saturated(const Eigen::Vector3d& gyro_rad_per_sec) noexcept {
        constexpr double kLimit = 0.98 * 2000.0 / 180.0 * std::numbers::pi;
        return std::abs(gyro_rad_per_sec.x()) >= kLimit || std::abs(gyro_rad_per_sec.y()) >= kLimit
            || std::abs(gyro_rad_per_sec.z()) >= kLimit;
    }

    [[nodiscard]] static double seconds(hcs_msgs::BoardClock::duration duration) noexcept {
        return std::chrono::duration<double>{duration}.count();
    }

    /// 加速度计样本不立刻用：留着，等时间上跨过它的那个陀螺仪样本来了再做修正。
    /// 第一个样本用来给滤波器定初始姿态（重力方向）。
    void push_accelerometer(const ImuSample& sample, TimePoint sample_time) noexcept {
        acceleration_g_ = accelerometer_g(sample);

        if (!initialized_) {
            if (ekf_.reset_from_accel(acceleration_g_)) {
                ekf_state_time_ = sample_time;
                latest_snapshot_ = {
                    .orientation = ekf_.quaternion(),
                    .gyro_body = Eigen::Vector3d::Zero(),
                    .timestamp = ekf_state_time_,
                };
                initialized_ = true;
            }
            return;
        }

        if (sample_time < ekf_state_time_)
            return;
        if (pending_accelerometer_ && sample_time < pending_accelerometer_->sample_time)
            return;
        pending_accelerometer_ = {acceleration_g_, sample_time};
    }

    /// 陀螺仪样本驱动滤波器前进：先预测到挂着的加速度计样本的时刻、做修正，再预测到自己的时刻。
    std::optional<Snapshot> update_with_gyroscope(
        const ImuSample& sample, TimePoint sample_time) noexcept {
        const Eigen::Vector3d gyro = gyroscope_rad_per_sec(sample);

        if (!initialized_)
            return std::nullopt;
        if (sample_time < ekf_state_time_)
            return std::nullopt;

        // 陀螺仪打满量程时读数已经不可信：把姿态的不确定度放回初值，让加速度计重新说了算。
        if (is_saturated(gyro))
            ekf_.inflate_attitude_uncertainty_to_initial();

        // 单趟的 while：只是为了能 break 出去。
        while (pending_accelerometer_) {
            const auto& accel_time = pending_accelerometer_->sample_time;
            if (accel_time < ekf_state_time_ || accel_time > sample_time)
                break;

            if (!ekf_.predict(gyro, seconds(accel_time - ekf_state_time_)))
                break;
            ekf_state_time_ = accel_time;

            // 卡方门限：和预测的重力方向差得太远的加速度（机体在做线加速）不拿来修正。
            const auto correction = ekf_.prepare_correction(pending_accelerometer_->accel_g);
            if (!correction || correction->chi_square() >= 3.0)
                break;
            if (!ekf_.correct(*correction))
                break;

            pending_accelerometer_ = std::nullopt;
            break;
        }

        // 重连之后板上缓冲里攒着的旧样本会一口气吐出来，时间戳一下跳很远。拿它去积分等于
        // 凭空转过一个很大的角度，所以只把滤波器的时刻挪过去，这个样本不用。
        if (seconds(sample_time - ekf_state_time_) > kMaxGyroscopeStep) {
            ekf_state_time_ = sample_time;
            return std::nullopt;
        }

        if (!ekf_.predict(gyro, seconds(sample_time - ekf_state_time_)))
            return std::nullopt;
        ekf_state_time_ = sample_time;

        return Snapshot{
            .orientation = ekf_.quaternion(),
            .gyro_body = gyro,
            .timestamp = ekf_state_time_,
        };
    }

    ImuOutputs outputs_;
    hcs_executor::Component::OutputInterface<Snapshot> snapshot_output_;

    Eigen::Matrix3d sensor_to_body_;
    BoardClockLifter clock_;

    bool initialized_ = false;
    filter::ImuEkf ekf_;
    TimePoint ekf_state_time_{};

    struct Accelerometer {
        Eigen::Vector3d accel_g;
        TimePoint sample_time;
    };
    std::optional<Accelerometer> pending_accelerometer_;

    Eigen::Vector3d acceleration_g_ = Eigen::Vector3d::Zero(); ///< 最后一个加速度计样本，g，机体系
    Snapshot latest_snapshot_;
};

} // namespace hcs_core::hardware::device
