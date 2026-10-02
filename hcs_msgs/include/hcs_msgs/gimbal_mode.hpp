#pragma once

#include <cstdint>

namespace hcs_msgs {

enum class GimbalMode : uint8_t {
    // 云台位置按陀螺仪闭环。
    // 底盘运动时也能保持准确的指向。
    IMU = 0,

    // 云台位置按编码器闭环。
    // 不漂，但只在底盘静止时有效。
    ENCODER = 1,
};

}