#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <numbers>
#include <span>
#include <string>

#include <eigen3/Eigen/Dense>

#include <hcs_executor/component.hpp>
#include <hcs_utility/double_buffer.hpp>
#include <hcs_utility/endian_promise.hpp>
#include <hcs_utility/package_receive.hpp>

namespace hcs_core::hardware::device {
using hcs_executor::Component;

/// @brief HiPNUC IMU of the CH0x0 series, HI91 binary frame
///
/// Frame layout, every multi byte field little endian:
///
///     5A A5 | length:u16 | crc:u16 | payload[length]
///
/// HI91 is the factory default output and the only fixed length frame: length is always 76, so a
/// frame is always 82 bytes, which is what lets receive_package read it into a fixed buffer and
/// resynchronize one byte at a time. The CRC is CRC16/CCITT (poly 0x1021, init 0) taken over the
/// four header bytes and then over the payload, i.e. the CRC field itself is skipped, not zeroed.
///
/// Reference: HiPNUC IMU command and programming manual v1.7.2, sections 2.8 / 7.31 / 7.35.
///
/// Output frame contract: vectors (angular_velocity, acceleration, magnetic_field) and the
/// quaternion body side are always FLU, x front / y left / z up, REP-103, no matter how the
/// module's flash is configured. A module left at the factory CONFIG IMU COORD 0 (ENU) reports
/// its body frame as RFU (x right / y front / z up); this class permutes such output to FLU once,
/// here, so no consumer ever has to know the module config. Tell the constructor which convention
/// the module uses through Config::module_frame — the driver cannot detect it, HI91 carries no
/// coordinate system field, a wrong declaration silently swaps axes.
class Hipnuc {
public:
    /// Bits of the MAIN_STATUS word. The bits not listed here are reserved: the firmware writes
    /// them for its own use and does not promise they are zero, so test by mask and never compare
    /// the whole word.
    enum class Status : uint16_t {
        kGyroBiasNotConverged = 1 << 3,   // WB_CONV, keep the device still for 3~5 s
        kMagneticDisturbance = 1 << 4,    // MAG_DIST, 9 axis only, yaw may be inertial hold
        kAccelerometerSaturated = 1 << 5, // ACC_SAT, now or within the last 2 s
        kGyroscopeSaturated = 1 << 6,     // GYR_SAT, now or within the last 2 s
        kAttitudeNotConverged = 1 << 7,   // ATT_CONV
        kStatic = 1 << 9,                 // STATIC, the module considers itself stationary
        kMagneticAiding = 1 << 10,        // MAG_AIDING, off means 6 axis mode
        kUtcNotSynchronized = 1 << 11,    // UTC_UNSYNC, set means NOT synchronized
        kSyncOutPulse = 1 << 12,          // SOUT_PULSE, this frame matches a SYNC_OUT pulse
    };

    struct Config {
        /// Which convention the module's flash holds, i.e. what CONFIG IMU COORD was last saved
        /// as. This selects the input permutation only: outputs are FLU either way.
        /// - kEnu (default, matches the factory state): body RFU, permuted to FLU here.
        /// - kNwu: body already FLU, passed through untouched.
        enum class ModuleFrame : uint8_t { kEnu, kNwu };

        Config& set_offline_timeout(int value) { return offline_timeout = value, *this; }
        Config& set_module_frame(ModuleFrame value) { return module_frame = value, *this; }

        /// Update cycles without a new frame before the module is considered offline. Counted in
        /// update_status() calls: at a 1 kHz control loop and the default 100 Hz HI91 output one
        /// frame is expected every 10 cycles.
        int offline_timeout = 100;

        ModuleFrame module_frame = ModuleFrame::kEnu;
    };

    Hipnuc(Component& status_component, const std::string& name_prefix) {
        status_component.register_output(
            name_prefix + "/quaternion", quaternion_output_, Eigen::Quaterniond::Identity());
        status_component.register_output(
            name_prefix + "/angular_velocity", angular_velocity_output_, Eigen::Vector3d::Zero());
        status_component.register_output(
            name_prefix + "/acceleration", acceleration_output_, Eigen::Vector3d::Zero());
        status_component.register_output(name_prefix + "/online", online_output_, false);

        configure(Config{});
    }

    Hipnuc(Component& status_component, const std::string& name_prefix, const Config& config)
        : Hipnuc(status_component, name_prefix) {
        configure(config);
    }

    Hipnuc(const Hipnuc&) = delete;
    Hipnuc& operator=(const Hipnuc&) = delete;
    Hipnuc(Hipnuc&&) = delete;
    Hipnuc& operator=(Hipnuc&&) = delete;

    ~Hipnuc() = default;

    void configure(const Config& config) {
        offline_timeout_ = config.offline_timeout;
        module_frame_    = config.module_frame;

        received_ = false;
        online_ = false;
        offline_count_ = 0;
        last_sequence_ = sequence_.load(std::memory_order::relaxed);
    }

    /// Feeds one chunk of received uart bytes, on the transport thread. Frame boundaries do not
    /// have to match chunk boundaries: a partial frame stays in the cache until the rest arrives.
    /// Returns true if at least one HI91 frame was accepted out of this chunk.
    ///
    /// Anything that is not HI91 (an HI83 frame, an NMEA line, noise from a wrong baudrate) is
    /// dropped by the resynchronization. That is silent on purpose, logging is not allowed on this
    /// thread, so the failure shows up in crc_error_count() and online() instead.
    bool store_status(std::span<const std::byte> uart_data) {
        auto stream = ByteSpanStream{uart_data};

        bool success = false;
        while (!stream.data.empty())
            success |= store_status<std::byte>(stream);
        return success;
    }

    bool store_status(const std::byte* uart_data, size_t uart_data_length) {
        return store_status(std::span{uart_data, uart_data_length});
    }

    /// For sources that hand out bytes through a read() of their own instead of a span. Consumes
    /// at most one frame worth of bytes per call.
    template <hcs_utility::is_byte ByteT>
    bool store_status(hcs_utility::is_readable_stream<ByteT> auto& stream) {
        const auto result = hcs_utility::receive_package<sizeof(Package::header), ByteT>(
            stream, package_, cache_size_,
            [](const Package& package) {
                return package.header[0] == 0x5A && package.header[1] == 0xA5;
            },
            [this](const Package& package) { return verify(package); });

        if (result != hcs_utility::ReceiveResult::SUCCESS)
            return false;

        cache_size_ = 0;
        payload_buffer_.write(package_.payload);
        sequence_.fetch_add(1, std::memory_order::release);
        return true;
    }

    /// Must be called once per control cycle: the offline watchdog is counted in calls.
    void update_status() {
        bool updated = false;

        const auto sequence = sequence_.load(std::memory_order::acquire);
        if (sequence != last_sequence_) {
            Hi91Payload payload;
            // A torn read means the transport thread overwrote the buffer mid copy. Keep the old
            // values and pick the frame up next cycle instead of decoding garbage.
            if (payload_buffer_.read(payload)) {
                last_sequence_ = sequence;
                decode(payload);
                updated = true;
            }
        }

        if (updated) {
            received_ = true;
            offline_count_ = offline_timeout_;
        } else if (offline_count_ > 0)
            --offline_count_;
        online_ = offline_count_ > 0;

        *quaternion_output_ = quaternion();
        *angular_velocity_output_ = angular_velocity();
        *acceleration_output_ = acceleration();
        *online_output_ = online();
    }

    // Attitude of the FLU body frame relative to the module world frame. World origin depends on
    // the module config (ENU heading zero = east, NWU heading zero = north) and matters only to
    // absolute heading consumers, which this robot does not have. Identity until first decode.
    const Eigen::Quaterniond& quaternion() const { return quaternion_; }

    // Euler angle unit: rad, fields roll pitch yaw, physical meanings identical in both module
    // configs. What does differ: heading zero (east vs north, same as quaternion()) and the
    // rotation order used to rebuild a rotation from the triple (312 under ENU, 321 under NWU,
    // manual section 2.2) — do not reconstruct rotations from these without knowing the config.
    // Yaw is the raw heading in +-pi, counterclockwise positive, and in 9 axis mode it references
    // magnetic north, not true north.
    const Eigen::Vector3d& euler_angles() const { return euler_angles_; }

    // Acceleration unit: m/s^2, FLU body frame. The wire unit is G.
    const Eigen::Vector3d& acceleration() const { return acceleration_; }

    // Angular velocity unit: rad/s, FLU body frame. The wire unit is deg/s, unlike the same field
    // of an HI83 frame, which is already rad/s.
    const Eigen::Vector3d& angular_velocity() const { return angular_velocity_; }

    // Magnetic field unit: uT, FLU body frame. Only as trustworthy as kMagneticDisturbance says.
    const Eigen::Vector3d& magnetic_field() const { return magnetic_field_; }

    // Air pressure unit: Pa
    double air_pressure() const { return air_pressure_; }

    // Temperature unit: celsius, module average rather than any single sensor die
    double temperature() const { return temperature_; }

    // Device timestamp unit: ms. Without UTC synchronization (kUtcNotSynchronized set) it is a
    // local monotonic counter wrapping every 24 h, with it, milliseconds since 00:00:00 UTC of the
    // current day. Either way it carries no date and is not the board clock, so do not mix the two.
    uint32_t system_time() const { return system_time_; }

    uint16_t main_status() const { return main_status_; }

    bool status(Status bit) const { return (main_status_ & static_cast<uint16_t>(bit)) != 0; }

    /// True once a frame has been decoded and no more than offline_timeout cycles have passed
    /// without a new one. Every accessor above returns stale data while this is false.
    bool online() const { return online_; }

    /// True once any frame has ever been decoded. Everything reads zero, and the quaternion
    /// identity, before that.
    bool received() const { return received_; }

    uint32_t frame_count() const { return sequence_.load(std::memory_order::relaxed); }

    /// Frames that framed correctly (header, length and tag all matched) but failed the CRC. A
    /// count that keeps growing points at the wiring or the baudrate, not at the protocol.
    uint32_t crc_error_count() const { return crc_error_count_.load(std::memory_order::relaxed); }

private:
    static constexpr uint8_t kHi91Tag = 0x91;
    static constexpr double kGravity = 9.80665; // m/s^2
    static constexpr double kHalfSqrt2 = std::numbers::sqrt2 / 2; // Rz(+90 deg) quaternion W/Z

    /// Field table of manual section 7.31. Offsets are 0 tag, 1 main_status, 3 temperature,
    /// 4 air_pressure, 8 system_time, 12 acc, 24 gyr, 36 mag, 48 euler, 60 quat. Every member
    /// is an endian container or a single byte, so the alignment is 1 and no packed attribute
    /// is needed, the sizeof assertion below is what proves the layout has no padding.
    struct Hi91Payload {
        uint8_t tag;
        hcs_utility::le_uint16_t main_status;
        int8_t temperature;                             // celsius
        hcs_utility::le_float32_t air_pressure;        // Pa
        hcs_utility::le_uint32_t system_time;          // ms
        hcs_utility::le_float32_t acceleration[3];     // G, XYZ
        hcs_utility::le_float32_t angular_velocity[3]; // deg/s, XYZ
        hcs_utility::le_float32_t magnetic_field[3];   // uT, XYZ
        hcs_utility::le_float32_t euler_angles[3];     // deg, roll pitch yaw
        hcs_utility::le_float32_t quaternion[4];       // WXYZ
    };
    static_assert(sizeof(Hi91Payload) == 76);

    struct Package {
        uint8_t header[2];                // 5A A5
        hcs_utility::le_uint16_t length; // sizeof(Hi91Payload) for HI91
        hcs_utility::le_uint16_t crc;    // checksum of every field except the crc itself
        Hi91Payload payload;
    };
    static_assert(sizeof(Package) == 82);

    /// Where the payload starts, and how much of the frame comes before the crc field. The crc
    /// covers the first part and the payload, and skips itself.
    static constexpr size_t kPayloadOffset = sizeof(Package) - sizeof(Hi91Payload);
    static constexpr size_t kCrcCoveredHeaderBytes = kPayloadOffset - sizeof(uint16_t);

    /// Adapts a byte span to the read() interface receive_package expects.
    struct ByteSpanStream {
        std::span<const std::byte> data;

        size_t read(std::byte* buffer, size_t size) {
            size = std::min(size, data.size());
            if (size == 0)
                return 0;

            std::memcpy(buffer, data.data(), size);
            data = data.subspan(size);
            return size;
        }
    };

    /// CRC16/CCITT, poly 0x1021, init 0, no reflection and no final xor. The running value is
    /// carried in and out so that one frame can be summed in two pieces, the way the crc16_update()
    /// of the manual does it.
    static constexpr uint16_t crc16(std::span<const std::byte> bytes, uint16_t crc = 0) {
        for (const std::byte byte : bytes) {
            crc ^= static_cast<uint16_t>(std::to_integer<unsigned>(byte) << 8);
            for (int bit = 0; bit < 8; ++bit)
                crc = static_cast<uint16_t>((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
        }
        return crc;
    }

    /// The example frame of manual section 7.34, checked at compile time. crc16() is the one piece
    /// of this decoder that is pure byte math, so it is also the one piece that can be pinned to
    /// the manual without a device, a bus, or a test runner.
    static consteval bool crc16_matches_manual_example() {
        constexpr auto frame = std::array<uint8_t, 82>{
            0x5A, 0xA5, 0x4C, 0x00, 0x14, 0xBB, 0x91, 0x08, 0x15, 0x23, 0x09, 0xA2, 0xC4, 0x47,
            0x08, 0x15, 0x1C, 0x00, 0xCC, 0xE8, 0x61, 0xBE, 0x9A, 0x35, 0x56, 0x3E, 0x65, 0xEA,
            0x72, 0x3F, 0x31, 0xD0, 0x7C, 0xBD, 0x75, 0xDD, 0xC5, 0xBB, 0x6B, 0xD7, 0x24, 0xBC,
            0x89, 0x88, 0xFC, 0x40, 0x01, 0x00, 0x6A, 0x41, 0xAB, 0x2A, 0x70, 0xC2, 0x96, 0xD4,
            0x50, 0x41, 0xED, 0x03, 0x43, 0x41, 0x41, 0xF4, 0xF4, 0xC2, 0xCC, 0xCA, 0xF8, 0xBE,
            0x73, 0x6A, 0x19, 0xBE, 0xF0, 0x00, 0x1C, 0x3D, 0x8D, 0x37, 0x5C, 0x3F};
        static_assert(frame.size() == sizeof(Package));

        const auto bytes = std::bit_cast<std::array<std::byte, sizeof(Package)>>(frame);
        const auto crc = crc16(
            std::span{bytes}.subspan(kPayloadOffset),
            crc16(std::span{bytes}.first(kCrcCoveredHeaderBytes)));

        // The crc the frame carries, read the same way the little endian field is read, and the
        // value the manual prints for it.
        const auto expected = static_cast<uint16_t>(
            std::to_integer<unsigned>(bytes[kCrcCoveredHeaderBytes])
            | std::to_integer<unsigned>(bytes[kCrcCoveredHeaderBytes + 1]) << 8);

        return crc == expected && expected == 0xBB14
            && bytes[kPayloadOffset] == std::byte{kHi91Tag} // payload really is HI91
            && bytes[2] == std::byte{sizeof(Hi91Payload)};  // and its length field says 76
    }

    /// Runs on the transport thread, once per candidate frame whose first two bytes matched the
    /// header. Rejecting here makes receive_package slide one byte and try again, which is how a
    /// frame that is not HI91 gets skipped.
    bool verify(const Package& package) const {
        // The checksum this function relies on is pinned to the manual at build time.
        static_assert(crc16_matches_manual_example());

        // static_cast：EndianContainer 自带的模板 operator!= 和经隐式转换的内置
        // != 在 clang 下二义（gcc 能选出来），显式转一侧，语义不变。
        if (static_cast<std::uint16_t>(package.length) != sizeof(Hi91Payload)
            || package.payload.tag != kHi91Tag)
            return false;

        // bit_cast, not reinterpret_cast: the byte view is a copy of the object, which keeps the
        // checksum clear of the alignment and aliasing questions a casted pointer would raise,
        // and is what lets crc16() also run in the compile time check above.
        const auto bytes = std::bit_cast<std::array<std::byte, sizeof(Package)>>(package);
        const auto crc = crc16(
            std::span{bytes}.subspan(kPayloadOffset),
            crc16(std::span{bytes}.first(kCrcCoveredHeaderBytes)));

        if (crc != static_cast<std::uint16_t>(package.crc)) {
            crc_error_count_.fetch_add(1, std::memory_order::relaxed);
            return false;
        }
        return true;
    }

    void decode(const Hi91Payload& payload) {
        main_status_ = payload.main_status;
        temperature_ = payload.temperature;
        air_pressure_ = payload.air_pressure;
        system_time_ = payload.system_time;

        constexpr double deg_to_rad = std::numbers::pi / 180.0;
        acceleration_ = to_vector(payload.acceleration) * kGravity;
        angular_velocity_ = to_vector(payload.angular_velocity) * deg_to_rad;
        magnetic_field_ = to_vector(payload.magnetic_field);
        euler_angles_ = to_vector(payload.euler_angles) * deg_to_rad;

        // The euler triple is not permuted: roll / pitch / yaw keep their physical meanings in
        // both module configs, only the decomposition order behind them changes (see accessor).

        // Order on the wire is WXYZ. The module already sends a unit quaternion, so renormalizing
        // only cleans up float error, and an all zero quaternion, which a module that is still
        // booting does send, must not be normalized into NaN.
        auto quaternion = Eigen::Quaterniond{
            static_cast<float>(payload.quaternion[0]), static_cast<float>(payload.quaternion[1]),
            static_cast<float>(payload.quaternion[2]),
            static_cast<float>(payload.quaternion[3])};
        if (quaternion.coeffs().squaredNorm() > 1e-6) {
            if (module_frame_ == Config::ModuleFrame::kEnu) {
                // ENU modules report the body frame as RFU. FLU = RFU rotated 90 deg about Z
                // (front = old y, left = -old x), so R_world<-flu = R_world<-rfu * Rz(+90 deg)
                // is a right multiplication. The permutation for vectors is the same rotation:
                // x_flu = y_rfu, y_flu = -x_rfu, z_flu = z_rfu.
                quaternion = quaternion * Eigen::Quaterniond{kHalfSqrt2, 0.0, 0.0, kHalfSqrt2};
                acceleration_    = rfu_to_flu(acceleration_);
                angular_velocity_ = rfu_to_flu(angular_velocity_);
                magnetic_field_  = rfu_to_flu(magnetic_field_);
            }
            quaternion_ = quaternion.normalized();
        }
    }

    static Eigen::Vector3d rfu_to_flu(const Eigen::Vector3d& v) {
        return {v.y(), -v.x(), v.z()};
    }

    static Eigen::Vector3d to_vector(const hcs_utility::le_float32_t (&values)[3]) {
        // The casts are not decoration: EndianContainer has a template conversion operator,
        // and a braced initializer would let it deduce std::initializer_list instead of float.
        return Eigen::Vector3d{
            static_cast<float>(values[0]), static_cast<float>(values[1]),
            static_cast<float>(values[2])};
    }

    // Transport thread side
    Package package_;
    size_t cache_size_ = 0;
    mutable std::atomic<uint32_t> crc_error_count_{0};

    hcs_utility::DoubleBuffer<Hi91Payload, true> payload_buffer_;
    std::atomic<uint32_t> sequence_{0};

    // Component thread side
    uint32_t last_sequence_ = 0;
    int offline_timeout_ = 0;
    Config::ModuleFrame module_frame_ = Config::ModuleFrame::kEnu;
    int offline_count_ = 0;
    bool received_ = false;
    bool online_ = false;

    Eigen::Quaterniond quaternion_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d euler_angles_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d magnetic_field_ = Eigen::Vector3d::Zero();
    double air_pressure_ = 0.0;
    double temperature_ = 0.0;
    uint32_t system_time_ = 0;
    uint16_t main_status_ = 0;

    Component::OutputInterface<Eigen::Quaterniond> quaternion_output_;
    Component::OutputInterface<Eigen::Vector3d> angular_velocity_output_;
    Component::OutputInterface<Eigen::Vector3d> acceleration_output_;
    Component::OutputInterface<bool> online_output_;
};

} // namespace hcs_core::hardware::device
