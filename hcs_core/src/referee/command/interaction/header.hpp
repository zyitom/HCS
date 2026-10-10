#pragma once

#include <bit>
#include <cstdint>

#include <hcs_msgs/full_robot_id.hpp>

// 原生 u16 packed 依赖小端主机。
static_assert(std::endian::native == std::endian::little, "wire structs assume a LE host");

namespace hcs_core::referee::command::interaction {

struct __attribute__((packed)) Header {
    uint16_t command_id;
    uint16_t sender_id;
    uint16_t receiver_id;
};

} // namespace hcs_core::referee::command::interaction