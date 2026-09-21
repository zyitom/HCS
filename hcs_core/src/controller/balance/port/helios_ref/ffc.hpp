// 对拍 harness 专用 stub：ffc（前馈控制器，云台用，不参与对拍）。
#pragma once

#include <cstdint>

namespace module {
struct FFC_Init_Config_s {
    float a{};
    float b{};
    float c{};
    float d{};
};
} // namespace module
