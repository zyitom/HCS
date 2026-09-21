// 对拍 harness 专用 stub：Chassis_Ctrl（v2 仅持有指针并在非空时发 IBC 遥测；
// harness 传 nullptr，回传路径不参与对拍）。
#pragma once

#include "robot_def.hpp"

namespace app {

class Chassis_Ctrl {
public:
    void IBC_Send_Pack4(
        float /*leg_l*/, float /*leg_r*/, bool /*is_fatal_error*/, float /*angle_l*/,
        float /*angle_r*/, float /*yaw*/, float /*pitch*/, uint8_t /*mode*/) {}
};

} // namespace app
