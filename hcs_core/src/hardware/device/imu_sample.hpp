#pragma once

#include <cstdint>

namespace hcs_core::hardware::device {

/// 板载 IMU 的一个原始样本。加速度计和陀螺仪各采各的、速率也不同，所以是两种样本混在一条流里，
/// 各自带着板上打的时间戳（四分之一微秒，32 位，会回绕）。
struct ImuSample {
    enum class Kind : std::uint8_t { kAccelerometer, kGyroscope };

    Kind kind;
    std::int16_t x;
    std::int16_t y;
    std::int16_t z;
    std::uint32_t timestamp_quarter_us;
};

} // namespace hcs_core::hardware::device
