// 对拍 harness 专用 stub：message_center 的 subMessage。
// 底盘 FSM 只订阅 MST_CHASSIS_CTRL（IBC 云台→底盘命令），由 harness 全局填充。
#pragma once

#include "robot_def.hpp"

inline Chassis_Ctrl_Data_s g_harness_chassis_ctrl{};

inline void subMessage(const char* /*topic*/, Chassis_Ctrl_Data_s& out) {
    out = g_harness_chassis_ctrl;
}
