#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>

// 裁判系统多字节字段为小端，原生类型 packed 结构依赖小端主机。
static_assert(std::endian::native == std::endian::little, "wire structs assume a LE host");

namespace hcs_core::referee {

constexpr uint8_t sof_value = 0xa5;
constexpr size_t frame_data_max_length = 1024;

struct __attribute__((packed)) FrameHeader {
    uint8_t sof;
    uint16_t data_length;
    uint8_t sequence;
    uint8_t crc8;
};

struct __attribute__((packed)) FrameBody {
    uint16_t command_id;
    std::byte data[frame_data_max_length];
};

struct __attribute__((packed)) Frame {
    FrameHeader header;
    FrameBody body;
};

} // namespace hcs_core::referee