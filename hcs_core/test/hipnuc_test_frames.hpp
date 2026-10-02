#pragma once

// HI91 帧的测试素材：手册上的示例帧，以及在它基础上改两个字段、重算 CRC 造出来的帧。
// 驱动的单测（test_hipnuc）和包装层的单测（test_board）都要用，所以单放一个头。

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

namespace hipnuc_test {

inline constexpr std::size_t kFrameSize = 82;
inline constexpr std::size_t kCrcOffset = 4;
inline constexpr std::size_t kPayloadOffset = 6;
inline constexpr std::size_t kTemperatureOffset = kPayloadOffset + 3;
inline constexpr std::size_t kAirPressureOffset = kPayloadOffset + 4;

using Frame = std::array<std::uint8_t, kFrameSize>;

/// 手册 7.34 的示例帧（与 hipnuc.hpp 里编译期校验 CRC 用的是同一帧）。
/// 温度 35 ℃，气压 0x47C4A209 ≈ 100676 Pa。
inline constexpr Frame kManualFrame{
    0x5A, 0xA5, 0x4C, 0x00, 0x14, 0xBB, 0x91, 0x08, 0x15, 0x23, 0x09, 0xA2, 0xC4, 0x47,
    0x08, 0x15, 0x1C, 0x00, 0xCC, 0xE8, 0x61, 0xBE, 0x9A, 0x35, 0x56, 0x3E, 0x65, 0xEA,
    0x72, 0x3F, 0x31, 0xD0, 0x7C, 0xBD, 0x75, 0xDD, 0xC5, 0xBB, 0x6B, 0xD7, 0x24, 0xBC,
    0x89, 0x88, 0xFC, 0x40, 0x01, 0x00, 0x6A, 0x41, 0xAB, 0x2A, 0x70, 0xC2, 0x96, 0xD4,
    0x50, 0x41, 0xED, 0x03, 0x43, 0x41, 0x41, 0xF4, 0xF4, 0xC2, 0xCC, 0xCA, 0xF8, 0xBE,
    0x73, 0x6A, 0x19, 0xBE, 0xF0, 0x00, 0x1C, 0x3D, 0x8D, 0x37, 0x5C, 0x3F};

/// CRC16/CCITT（poly 0x1021，init 0）。测试里自己再写一遍，而不是去借驱动的私有实现：
/// 借来的话，驱动算错了测试也跟着错。
constexpr std::uint16_t crc16(std::span<const std::uint8_t> bytes, std::uint16_t crc = 0) {
    for (const std::uint8_t byte : bytes) {
        crc ^= static_cast<std::uint16_t>(byte << 8);
        for (int bit = 0; bit < 8; ++bit)
            crc = static_cast<std::uint16_t>((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
    }
    return crc;
}

/// 在示例帧的基础上改温度和气压，重算 CRC。两个字段由同一个 tag 决定
/// （温度 = tag，气压 = tag × 100）：读到半新半旧的一帧，两者就对不上。
constexpr Frame make_frame(std::int8_t tag) {
    Frame frame = kManualFrame;
    frame[kTemperatureOffset] = static_cast<std::uint8_t>(tag);
    const auto pressure =
        std::bit_cast<std::array<std::uint8_t, 4>>(static_cast<float>(tag) * 100.0F);
    for (std::size_t i = 0; i < pressure.size(); ++i)
        frame[kAirPressureOffset + i] = pressure[i];

    const auto bytes = std::span<const std::uint8_t>{frame};
    const auto crc = crc16(bytes.subspan(kPayloadOffset), crc16(bytes.first(kCrcOffset)));
    frame[kCrcOffset] = static_cast<std::uint8_t>(crc);
    frame[kCrcOffset + 1] = static_cast<std::uint8_t>(crc >> 8);
    return frame;
}

// 上面那份 CRC 先对一下手册：示例帧算出来必须是它自己带的 0xBB14。
static_assert(
    crc16(
        std::span<const std::uint8_t>{kManualFrame}.subspan(kPayloadOffset),
        crc16(std::span<const std::uint8_t>{kManualFrame}.first(kCrcOffset)))
    == 0xBB14);
static_assert(std::endian::native == std::endian::little);

} // namespace hipnuc_test
