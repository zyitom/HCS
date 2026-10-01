#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <utility>

#include <hcs_executor/component.hpp>
#include <hcs_base/protocol/endian_promise.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/util/required.hpp"

namespace hcs_core::hardware::device {

class DjiMotor {
public:
    /// The bus this driver speaks: classic CAN 2.0, 8-byte frames, at the motor's factory
    /// 1 Mbps. A bus declared otherwise is refused when the motor is attached to it.
    static constexpr std::uint32_t kCanBitrate = 1'000'000;
    static constexpr bool kCanFd = false;

    enum class Type : uint8_t { kGM6020, kGM6020Voltage, kM3508, kM2006 };

    /// Feedback DATA[7]. Older ESC firmware documents this byte as null and sends 0, which
    /// decodes as kNone, so reading it is safe on either firmware. When several conditions hold
    /// at once the ESC reports the most severe one, i.e. the smallest nonzero code. Codes the
    /// manual does not list (6, 9 and up) are kept as is rather than folded into a known one.
    enum class Error : uint8_t {
        kNone                 = 0,
        kStorageUnreachable   = 1, // power-on self test only
        kSupplyOverVoltage    = 2, // power-on self test only
        kPhaseDisconnected    = 3,
        kPositionSensorLost   = 4,
        kMotorOverTemperature = 5, // >= 180 C
        kCalibrationFailed    = 7,
        kMotorOverheat        = 8, // >= 125 C
    };

    /// An aggregate, so a wiring table names what it sets. motor_type and id are required
    /// (util::Required); reduction_ratio defaults to the model's own gearbox.
    struct Config {
        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reduction_ratio(double value) { return reduction_ratio = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
        Config& set_offline_timeout(int value) { return offline_timeout = value, *this; }

        util::Required<Type> motor_type;
        /// ESC id set by the DIP switches / LED blink count (1..8).
        util::Required<std::uint8_t> id;
        int encoder_zero_point = 0;
        /// Rotor to output shaft. Unset: the model's own gearbox (3591/187 for M3508, 36 for
        /// M2006, 1 for GM6020); set it to replace that, e.g. an external stage.
        std::optional<double> reduction_ratio = std::nullopt;
        bool reversed = false;
        bool multi_turn_angle_enabled = false;
        int offline_timeout = 100;
    };

    static constexpr double default_reduction_ratio(Type type) {
        switch (type) {
        case Type::kGM6020:
        case Type::kGM6020Voltage: return 1.0;
        case Type::kM3508: return 3591.0 / 187.0;
        case Type::kM2006: return 36.0;
        }
        return 1.0;
    }

    DjiMotor(
        hcs_executor::Component& status_component, hcs_executor::Component& command_component,
        const std::string& name_prefix)
        : angle_(0.0)
        , velocity_(0.0)
        , torque_(0.0) {
        status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_output_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);
        status_component.register_output(name_prefix + "/error_code", error_code_output_, uint8_t{0});
        status_component.register_output(name_prefix + "/online", online_output_, false);

        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);
    }

    DjiMotor(
        hcs_executor::Component& status_component, hcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : DjiMotor(status_component, command_component, name_prefix) {
        configure(config);
    }

    DjiMotor(const DjiMotor&) = delete;
    DjiMotor& operator=(const DjiMotor&) = delete;
    DjiMotor(DjiMotor&&) = delete;
    DjiMotor& operator=(DjiMotor&&) = delete;

    ~DjiMotor() = default;

    void configure(const Config& config) {
        type_ = config.motor_type;
        id_ = config.id;
        const double reduction_ratio = config.reduction_ratio.value_or(default_reduction_ratio(type_));
        encoder_zero_point_ = config.encoder_zero_point % kRawAngleMax;
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += kRawAngleMax;

        const double sign = config.reversed ? -1 : 1;

        raw_angle_to_angle_coefficient_ =
            sign / reduction_ratio / kRawAngleMax * 2 * std::numbers::pi;
        angle_to_raw_angle_coefficient_ = 1 / raw_angle_to_angle_coefficient_;

        raw_velocity_to_velocity_coefficient_ =
            sign / reduction_ratio / 60 * 2 * std::numbers::pi;
        velocity_to_raw_velocity_coefficient_ = 1 / raw_velocity_to_velocity_coefficient_;

        double torque_constant, raw_current_max, current_max;
        switch (type_) {
        case Type::kGM6020:
            torque_constant = 0.741;
            raw_current_max = 16384.0;
            current_max = 3.0;
            break;
        case Type::kGM6020Voltage:
            torque_constant = 0.741;
            raw_current_max = 25000.0;
            current_max = 3.0;
            break;
        case Type::kM3508:
            torque_constant = 0.3 * 187.0 / 3591.0;
            raw_current_max = 16384.0;
            current_max = 20.0;
            break;
        case Type::kM2006:
            torque_constant = 0.18 * 1.0 / 36.0;
            raw_current_max = 16384.0;
            current_max = 10.0;
            break;
        default: std::unreachable();
        }

        raw_current_to_torque_coefficient_ =
            sign * reduction_ratio * torque_constant / raw_current_max * current_max;
        torque_to_raw_current_coefficient_ = 1 / raw_current_to_torque_coefficient_;

        max_torque_ = 1 * reduction_ratio * torque_constant * current_max;

        last_raw_angle_ = 0;
        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_ = 0;
        error_ = Error::kNone;

        // The zero initialized packet decodes as angle 0, which in multi turn mode would seed the
        // turn count from a position the motor never reported. Never decode before the first
        // feedback frame has actually arrived.
        received_ = false;
        online_ = false;
        offline_count_ = 0;
        offline_timeout_ = config.offline_timeout;
        last_sequence_ = sequence_.load(std::memory_order::relaxed);

        angle_ = 0.0;
        velocity_ = 0.0;
        torque_ = 0.0;
        temperature_ = 0.0;

        *max_torque_output_ = max_torque();
    }

    void store_status(std::span<const std::byte> can_data) {
        if (can_data.size() != 8) [[unlikely]]
            return;

        // The fixed extent overload is noexcept; the dynamic one throws. This runs on the
        // transport thread, where nothing may throw. Release pairs with the acquire in
        // update_status(): a new sequence must never be seen ahead of the packet it counts.
        can_data_.store(CanPacket8{can_data.first<8>()}, std::memory_order::relaxed);
        sequence_.fetch_add(1, std::memory_order::release);
    }

    static constexpr auto recv_id(Type type, std::uint8_t index) -> std::uint32_t {
        switch (type) {
        case Type::kGM6020:
        case Type::kGM6020Voltage: return 0x204 + index;
        case Type::kM3508:
        case Type::kM2006: return 0x200 + index;
        }
        return 0;
    }

    static constexpr auto send_id(Type type, std::uint8_t index) -> std::uint32_t {
        switch (type) {
        case Type::kGM6020: return index <= 4 ? 0x1FE : 0x2FE;
        case Type::kGM6020Voltage: return index <= 4 ? 0x1FF : 0x2FF;
        case Type::kM3508:
        case Type::kM2006: return index <= 4 ? 0x200 : 0x1FF;
        }
        return 0;
    }

    auto id() const noexcept -> std::uint8_t { return id_; }
    auto recv_id() const noexcept -> std::uint32_t { return recv_id(type_, id_); }
    auto send_id() const noexcept -> std::uint32_t { return send_id(type_, id_); }

    bool match_then_store_status(std::uint32_t can_id, std::span<const std::byte> can_data) {
        if (can_id != recv_id())
            return false;
        store_status(can_data);
        return true;
    }

    /// Must be called once per control cycle: the offline watchdog is counted in calls. The ESC
    /// and the control loop both run at about 1 kHz on unrelated clocks, so a cycle with no new
    /// frame (or with two) is normal; only a run of offline_timeout empty cycles is not.
    void update_status() {
        const auto sequence = sequence_.load(std::memory_order::acquire);
        if (sequence != last_sequence_) {
            last_sequence_ = sequence;
            received_ = true;
            offline_count_ = offline_timeout_;
        } else if (offline_count_ > 0)
            --offline_count_;
        online_ = offline_count_ > 0;

        if (received_) [[likely]]
            decode(std::bit_cast<DjiMotorFeedback>(can_data_.load(std::memory_order::relaxed)));

        *angle_output_ = angle();
        *velocity_output_ = velocity();
        *torque_output_ = torque();
        *error_code_output_ = static_cast<uint8_t>(error());
        *online_output_ = online();
    }

    double control_torque() const {
        if (control_torque_.ready() && id_ != 0) [[likely]]
            return *control_torque_;
        else
            return 0.0;
    }

    CanPacket8::Quarter generate_command() const { return generate_command(control_torque()); }

    CanPacket8::Quarter generate_command(double control_torque) const {
        if (std::isnan(control_torque)) {
            return CanPacket8::Quarter{0};
        }

        control_torque = std::clamp(control_torque, -max_torque_, max_torque_);
        const double current = std::round(torque_to_raw_current_coefficient_ * control_torque);
        const hcs_utility::be_int16_t control_current = static_cast<int16_t>(current);

        return std::bit_cast<CanPacket8::Quarter>(control_current);
    }

    /// Which 2-byte slot of the shared frame send_id() this motor writes; nothing for id 0.
    /// Two motors on one bus must not share both the send id and the slot (an M3508 with id 5
    /// and a voltage GM6020 with id 1 do: both write slot 0 of 0x1FF).
    std::optional<std::size_t> command_slot() const noexcept {
        if (id_ == 0)
            return std::nullopt;
        return static_cast<std::size_t>((id_ - 1) % 4);
    }

    /// @brief Hand this cycle's current to the bus it sits on. The ESC takes four motors per
    /// frame: every motor with the same send id on one bus writes its own 2-byte slot,
    /// (id - 1) % 4, of one shared frame. A safe cycle writes zero current. id 0 (unassigned)
    /// still opens the frame but leaves every slot alone.
    template <class BusFrames>
    void append_command(BusFrames& bus, bool safe) const {
        auto& packet = bus.shared_frame(send_id());
        if (id_ != 0)
            packet.data[(id_ - 1) % 4] = safe ? std::uint16_t{0} : generate_command().data;
    }

    int calibrate_zero_point() {
        angle_multi_turn_ = 0;
        encoder_zero_point_ = last_raw_angle_;
        return encoder_zero_point_;
    }

    int last_raw_angle() const { return last_raw_angle_; }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return max_torque_; }
    double temperature() const { return temperature_; }
    Error error() const { return error_; }
    bool online() const { return online_; }
    /// True once any feedback frame has been decoded.
    bool received() const { return received_; }

    /// Whether error() is a fault rather than a warning. kMotorOverheat (>= 125 C) is the one
    /// code the manual ranks below the rest; every other nonzero code, unlisted ones included, is
    /// treated as a fault. The ESC clears the code by itself once the condition is gone.
    bool faulted() const { return error_ != Error::kNone && error_ != Error::kMotorOverheat; }

private:
    void decode(const auto& feedback) {
        // Temperature unit: celsius
        temperature_ = static_cast<double>(feedback.temperature);

        // The underlying type is fixed, so any byte is a valid Error value, listed or not.
        error_ = static_cast<Error>(feedback.error);

        // Angle unit: rad
        const int raw_angle = feedback.angle;
        int calibrated_raw_angle = raw_angle - encoder_zero_point_;
        if (calibrated_raw_angle < 0)
            calibrated_raw_angle += kRawAngleMax;
        if (!multi_turn_angle_enabled_) {
            angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(calibrated_raw_angle);
            if (angle_ < 0)
                angle_ += 2 * std::numbers::pi;
        } else {
            auto diff = (calibrated_raw_angle - angle_multi_turn_) % kRawAngleMax;
            if (diff <= -kRawAngleMax / 2)
                diff += kRawAngleMax;
            else if (diff > kRawAngleMax / 2)
                diff -= kRawAngleMax;
            angle_multi_turn_ += diff;
            angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(angle_multi_turn_);
        }
        last_raw_angle_ = raw_angle;

        // Velocity unit: rad/s
        velocity_ = raw_velocity_to_velocity_coefficient_ * static_cast<double>(feedback.velocity);

        // Torque unit: N*m
        torque_ = raw_current_to_torque_coefficient_ * static_cast<double>(feedback.current);
    }

    struct alignas(uint64_t) DjiMotorFeedback {
        hcs_utility::be_int16_t angle;
        hcs_utility::be_int16_t velocity;
        hcs_utility::be_int16_t current;
        uint8_t temperature;
        uint8_t error; // Error code; null (0) on older firmware
    };
    static_assert(sizeof(DjiMotorFeedback) == sizeof(CanPacket8));

    Type type_ = Type::kM3508;
    std::uint8_t id_ = 0;
    std::atomic<CanPacket8> can_data_;
    std::atomic<std::uint32_t> sequence_ = 0;
    std::uint32_t last_sequence_ = 0;

    bool received_ = false;
    bool online_ = false;
    int offline_count_ = 0, offline_timeout_ = 0;

    static constexpr int kRawAngleMax = 8192;
    int encoder_zero_point_, last_raw_angle_;

    bool multi_turn_angle_enabled_;
    int64_t angle_multi_turn_;

    double raw_angle_to_angle_coefficient_, angle_to_raw_angle_coefficient_;
    double raw_velocity_to_velocity_coefficient_, velocity_to_raw_velocity_coefficient_;
    double raw_current_to_torque_coefficient_, torque_to_raw_current_coefficient_;

    double angle_;
    double velocity_;
    double torque_;
    double max_torque_;
    double temperature_;
    Error error_ = Error::kNone;

    hcs_executor::Component::OutputInterface<double> angle_output_;
    hcs_executor::Component::OutputInterface<double> velocity_output_;
    hcs_executor::Component::OutputInterface<double> torque_output_;
    hcs_executor::Component::OutputInterface<double> max_torque_output_;
    hcs_executor::Component::OutputInterface<uint8_t> error_code_output_;
    hcs_executor::Component::OutputInterface<bool> online_output_;

    hcs_executor::Component::InputInterface<double> control_torque_;
};

} // namespace hcs_core::hardware::device
