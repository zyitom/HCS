#pragma once

#include <cstdint>

#include <libhcs/time/sample_time.hpp>

namespace hcs_core::hardware::device {

/// 板载 IMU 的一个原始样本。加速度计和陀螺仪各采各的、速率也不同，所以是两种样本混在一条流里，
/// 各自带着板上打的时间戳（四分之一微秒，32 位，会回绕）。
///
/// time 是 SDK 的 IO 线程在解码时算好的样本时刻（libhcs/time/sample_time.hpp）：board 是这块
/// 板自己的计时器展开到 64 位后的时刻（永不回绕，算相邻样本的 dt 直接相减，不经过任何主机
/// 换算，时间基准没锁定时也在），host 是本机 steady_clock 上的时刻（转换成功时是打戳那一刻，
/// 否则退回到达时刻，看 source 区分）。
struct ImuSample {
    enum class Kind : std::uint8_t { kAccelerometer, kGyroscope };

    Kind kind;
    std::int16_t x;
    std::int16_t y;
    std::int16_t z;
    std::uint32_t timestamp_quarter_us;
    libhcs::time::SampleTime time;
};

/// 驱动对板载 IMU 的要求：量程与输出速率。端口据此经 EP0 下发（板子按它初始化传感器），
/// 驱动自己用同一组数把原始值换成物理量——两头用的是同一个出处，对不上的可能就没有了。
/// 串口设备的 SerialLine 是同一个角色。
struct ImuSetup {
    std::uint8_t accelerometer_range_g;     ///< ±g，16 位有符号数满量程
    std::uint16_t accelerometer_rate_hz;
    std::uint16_t gyroscope_range_dps;      ///< ±°/s，16 位有符号数满量程
    std::uint16_t gyroscope_rate_hz;
    std::uint16_t gyroscope_bandwidth_hz;   ///< 陀螺仪片内低通的带宽，与输出速率成对

    /// 原始值 → g
    [[nodiscard]] constexpr double accelerometer_scale() const noexcept {
        return accelerometer_range_g / 32767.0;
    }
    /// 原始值 → °/s
    [[nodiscard]] constexpr double gyroscope_scale_dps() const noexcept {
        return gyroscope_range_dps / 32767.0;
    }
};

/// BMI088 在本项目里一直用的那一组：±6 g / 1600 Hz，±2000 °/s / 2000 Hz（230 Hz 低通）。
inline constexpr ImuSetup kBmi088Setup{
    .accelerometer_range_g = 6,
    .accelerometer_rate_hz = 1600,
    .gyroscope_range_dps = 2000,
    .gyroscope_rate_hz = 2000,
    .gyroscope_bandwidth_hz = 230,
};

} // namespace hcs_core::hardware::device
