#pragma once

#include <cmath>
#include <cstdint>
#include <format>
#include <numbers>
#include <string>

#include <eigen3/Eigen/Dense>

#include <hcs_executor/component.hpp>

#include "hardware/device/imu_outputs.hpp"
#include "hardware/device/imu_sample.hpp"

namespace hcs_core::hardware::device {

/// 板载 BMI088，姿态用 Mahony 互补滤波解算。
///
/// 加速度计量程 ±6 g，陀螺仪量程 ±2000 °/s，都是 16 位有符号数满量程。
///
/// 只写协议与解算：全部成员都在控制线程上，样本由端口包装层（board::Imu<>）按到达顺序交进来；
/// 掉线判定也在包装层。
///
/// 陀螺仪每来一个样本滤波走一步，步长是它和上一个陀螺仪样本的板上时间戳之差；加速度计用的是
/// 当时最新的那一个。所以控制线程偶尔晚了、一拍里攒了几个样本，姿态也不受影响，也不需要
/// 知道控制频率。和 RMCS 的区别在这里：那边是每拍走一步、步长写死成 1 ms。
///
/// 航向没有参考（没有磁力计），只靠陀螺仪积分，会漂。
class Bmi088 {
public:
    struct Config {
        Config& set_gain(double kp_value, double ki_value) {
            return kp = kp_value, ki = ki_value, *this;
        }
        Config& set_sensor_to_body(const Eigen::Matrix3d& value) {
            return sensor_to_body = value, *this;
        }
        Config& set_offline_timeout(int value) { return offline_timeout = value, *this; }

        /// Mahony 的比例与积分增益。kp 越大越信加速度计（收敛快、对线加速度敏感）；
        /// ki 用来吃掉陀螺仪零偏，0 表示不用。
        double kp = 0.2;
        double ki = 0.0;

        /// 把传感器轴上的量转到机体系（FLU）：body = sensor_to_body * sensor。由板子怎么装决定。
        Eigen::Matrix3d sensor_to_body = Eigen::Matrix3d::Identity();

        /// 连续多少拍没有样本算掉线。BMI088 每毫秒都有样本，100 拍已经很宽。
        int offline_timeout = 100;
    };

    Bmi088(hcs_executor::Component& status_component, const std::string& name, const Config& config)
        : outputs_(status_component, name)
        , double_kp_(2.0 * config.kp)
        , double_ki_(2.0 * config.ki)
        , sensor_to_body_(config.sensor_to_body) {}

    Bmi088(const Bmi088&) = delete;
    Bmi088& operator=(const Bmi088&) = delete;

    /// 周期域：一个原始样本，按到达顺序。
    /// @return 这个样本有没有让姿态前进一步。只有陀螺仪样本会；加速度计样本还没到过、
    ///         第一个陀螺仪样本（没有上一个可以算步长）、时间戳跳变的样本都返回 false
    bool on_sample(const ImuSample& sample) noexcept {
        const Eigen::Vector3d raw{
            static_cast<double>(sample.x), static_cast<double>(sample.y),
            static_cast<double>(sample.z)};
        if (sample.kind == ImuSample::Kind::kAccelerometer) {
            acceleration_ = sensor_to_body_ * raw * kAccelerometerScale;
            accelerometer_received_ = true;
            return false;
        }

        angular_velocity_ = sensor_to_body_ * raw * kGyroscopeScale;

        // 无符号相减：时间戳在 32 位上回绕时差值仍然是对的。
        const std::uint32_t elapsed = sample.timestamp_quarter_us - last_gyroscope_timestamp_;
        const bool has_previous = gyroscope_received_;
        last_gyroscope_timestamp_ = sample.timestamp_quarter_us;
        gyroscope_received_ = true;

        // 没有重力方向就开始积分的话，姿态是从一个没有参考的状态起步的。
        if (!accelerometer_received_ || !has_previous)
            return false;
        // 隔得太久的样本不拿来积分：掉线之后回来、或者重连后板上缓冲里吐出来的旧数据，
        // 拿它乘上这么长的时间等于凭空转过一个很大的角度。
        if (elapsed > kMaxStepQuarterUs)
            return false;

        mahony_step(acceleration_, angular_velocity_, elapsed * kSecondsPerQuarterUs);
        outputs_.publish(
            {.quaternion = quaternion_,
             .angular_velocity = angular_velocity_,
             .acceleration = acceleration_ * kGravity,
             .euler_angles = ImuOutputs::euler_from(quaternion_)});
        return true;
    }

    /// 机体系相对世界系的姿态。首次解算之前是单位四元数。
    [[nodiscard]] const Eigen::Quaterniond& quaternion() const noexcept { return quaternion_; }
    /// rad/s，机体系。
    [[nodiscard]] const Eigen::Vector3d& angular_velocity() const noexcept {
        return angular_velocity_;
    }
    /// 单位是 g，机体系（输出上的是 m/s²）。
    [[nodiscard]] const Eigen::Vector3d& acceleration_g() const noexcept { return acceleration_; }

    [[nodiscard]] std::string describe() const {
        return std::format("onboard BMI088, Mahony kp={} ki={}", double_kp_ / 2, double_ki_ / 2);
    }

private:
    static constexpr double kAccelerometerScale = 6.0 / 32767.0; ///< 原始值 → g
    static constexpr double kGyroscopeScale =
        2000.0 / 32767.0 * std::numbers::pi / 180.0;             ///< 原始值 → rad/s
    static constexpr double kGravity = 9.80665;                  ///< m/s²

    static constexpr double kSecondsPerQuarterUs = 1.0 / 4'000'000.0;
    /// 两个陀螺仪样本之间最多隔这么久（10 ms），再长就不积分。
    static constexpr std::uint32_t kMaxStepQuarterUs = 40'000;

    /// Madgwick 写的 Mahony AHRS（只用加速度计和陀螺仪的那一版）：
    /// http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
    /// @param period 这一步的步长，秒
    void mahony_step(Eigen::Vector3d accel, Eigen::Vector3d gyro, double period) noexcept {
        double q0 = quaternion_.w();
        double q1 = quaternion_.x();
        double q2 = quaternion_.y();
        double q3 = quaternion_.z();

        // 加速度全零时没有方向可言，归一化会出 NaN：这一步只做陀螺仪积分。
        if (accel.squaredNorm() > 0.0) {
            accel.normalize();

            // 按当前姿态估计的重力方向（机体系），只有一半大小。
            const Eigen::Vector3d half_gravity{
                q1 * q3 - q0 * q2, q0 * q1 + q2 * q3, q0 * q0 - 0.5 + q3 * q3};
            // 测到的重力方向和估计的重力方向差多少：两者的叉积。
            const Eigen::Vector3d half_error = accel.cross(half_gravity);

            if (double_ki_ > 0.0) {
                integral_feedback_ += double_ki_ * half_error * period;
                gyro += integral_feedback_;
            } else {
                integral_feedback_.setZero(); // 不用积分项时别让它攒着
            }
            gyro += double_kp_ * half_error;
        }

        // 四元数的变化率积分一步。
        gyro *= 0.5 * period;
        const double qa = q0;
        const double qb = q1;
        const double qc = q2;
        q0 += -qb * gyro.x() - qc * gyro.y() - q3 * gyro.z();
        q1 += qa * gyro.x() + qc * gyro.z() - q3 * gyro.y();
        q2 += qa * gyro.y() - qb * gyro.z() + q3 * gyro.x();
        q3 += qa * gyro.z() + qb * gyro.y() - qc * gyro.x();

        quaternion_ = Eigen::Quaterniond{q0, q1, q2, q3}.normalized();
    }

    ImuOutputs outputs_;

    double double_kp_; ///< 2 × 比例增益
    double double_ki_; ///< 2 × 积分增益
    Eigen::Matrix3d sensor_to_body_;

    Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();     ///< g，机体系，最后一个样本
    Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero(); ///< rad/s，机体系，最后一个样本
    bool accelerometer_received_ = false;
    bool gyroscope_received_ = false;
    std::uint32_t last_gyroscope_timestamp_ = 0; ///< 四分之一微秒，会回绕

    Eigen::Quaterniond quaternion_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d integral_feedback_ = Eigen::Vector3d::Zero(); ///< 已经乘过 ki 的积分误差
};

} // namespace hcs_core::hardware::device
