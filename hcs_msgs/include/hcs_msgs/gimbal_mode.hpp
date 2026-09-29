#pragma once

#include <cstdint>

namespace hcs_msgs {

enum class GimbalMode : uint8_t {
    // Gyro closed-loop gimbal position.
    // Maintain precise positioning during chassis movement.
    IMU = 0,

    // Encoder closed-loop gimbal position.
    // No drift, but only works when the chassis is stationary.
    ENCODER = 1,
};

}