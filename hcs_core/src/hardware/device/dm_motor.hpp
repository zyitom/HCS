#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <utility>

#include <hcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"

namespace hcs_core::hardware::device {

class DmMotor {
public:
    enum class Type : uint8_t { kJ4310 };

    /// Motor status, carried in the high nibble of feedback D[0].
    enum class Error : uint8_t {
        kDisabled            = 0x0,
        kEnabled             = 0x1,
        kOverVoltage         = 0x8,
        kUnderVoltage        = 0x9,
        kOverCurrent         = 0xA,
        kMosOverTemperature  = 0xB,
        kCoilOverTemperature = 0xC,
        kCommunicationLost   = 0xD,
        kOverload            = 0xE,
    };

    struct Config {
        explicit Config(Type motor_type, std::uint32_t esc_id = 0, std::uint32_t master_id = 0)
            : motor_type(motor_type)
            , esc_id(esc_id)
            , master_id(master_id) {
            switch (motor_type) {
            case Type::kJ4310:
                // Factory defaults of registers PMAX(0x15) / VMAX(0x16) / TMAX(0x17).
                //
                // Note: these are NOT model constants, they are writable registers. In MIT mode
                // they are the mapping basis of BOTH the command and the feedback frame, so a
                // mismatch against the value actually stored in the driver silently scales
                // everything and reports no error. Read them back over 0x7FF/RID 0x33 once at
                // bringup rather than trusting these defaults.
                position_max = 12.5;
                velocity_max = 30.0;
                torque_max   = 10.0;
                break;
            }
            this->reversed                 = false;
            this->multi_turn_angle_enabled = false;
        }

        Config& set_position_max(double value) { return position_max = value, *this; }
        Config& set_velocity_max(double value) { return velocity_max = value, *this; }
        Config& set_torque_max(double value) { return torque_max = value, *this; }
        Config& set_gain(double kp_value, double kd_value) {
            return kp = kp_value, kd = kd_value, *this;
        }
        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reduction_ratio(double value) { return reduction_ratio = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
        Config& set_offline_timeout(int value) { return offline_timeout = value, *this; }
        Config& set_error_retry_interval(int value) { return error_retry_interval = value, *this; }

        Type motor_type;

        /// Receive id of the driver (register ESC_ID 0x08). MIT control frames are sent to this
        /// id with no offset. The other three modes add 0x100 / 0x200 / 0x300 and are not
        /// implemented here.
        std::uint32_t esc_id;
        /// Feedback id of the driver (register MST_ID 0x07). Must be unique per motor, and must
        /// not collide with the DJI feedback range (0x201~0x20B) on the same bus. A collision
        /// produces no error, only silence.
        std::uint32_t master_id;

        double position_max; // rad
        double velocity_max; // rad/s
        double torque_max;   // N*m

        /// Default MIT gains, used when /control_kp and /control_kd have no provider.
        /// Protocol range is kp in [0, 500] and kd in [0, 5].
        double kp = 0.0;
        double kd = 0.0;

        /// Raw feedback value treated as zero angle. The default trusts the zero point stored
        /// in the driver itself, i.e. the result of the "save zero position" frame.
        int encoder_zero_point = kRawAngleZero;

        /// External gearbox only. The 10:1 stage inside a J4310 is already accounted for by the
        /// driver, whose feedback is output shaft referred. Note that kp / kd are driver side
        /// gains and are NOT scaled by this.
        double reduction_ratio = 1.0;

        bool reversed;
        /// Off: angle is what the driver reports, spanning +-position_max (about +-4 turns).
        /// On: the wrap at +-position_max is accumulated and angle grows without bound.
        bool multi_turn_angle_enabled;

        /// Update cycles without new feedback before the motor is considered offline.
        int offline_timeout = 100;
        /// Update cycles between two "clear error" frames. The protections (over temperature,
        /// over voltage, over current) all need time to recover, so retrying at the loop rate
        /// only fights the protection logic.
        int error_retry_interval = 500;
    };

    DmMotor(
        hcs_executor::Component& status_component, hcs_executor::Component& command_component,
        const std::string& name_prefix) {
        status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
        status_component.register_output(name_prefix + "/raw_angle", raw_angle_output_, 0);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_output_, 0.0);
        status_component.register_output(name_prefix + "/temperature", temperature_output_, 0.0);
        status_component.register_output(
            name_prefix + "/temperature_mos", temperature_mos_output_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);
        status_component.register_output(name_prefix + "/error_code", error_code_output_, uint8_t{0});
        status_component.register_output(name_prefix + "/online", online_output_, false);

        command_component.register_input( //
            name_prefix + "/control_angle", control_angle_, false);
        command_component.register_input( //
            name_prefix + "/control_velocity", control_velocity_, false);
        command_component.register_input( //
            name_prefix + "/control_torque", control_torque_, false);
        command_component.register_input( //
            name_prefix + "/control_kp", control_kp_, false);
        command_component.register_input( //
            name_prefix + "/control_kd", control_kd_, false);
    }

    DmMotor(
        hcs_executor::Component& status_component, hcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : DmMotor(status_component, command_component, name_prefix) {
        configure(config);
    }

    DmMotor(const DmMotor&) = delete;
    DmMotor& operator=(const DmMotor&) = delete;
    DmMotor(DmMotor&&) = delete;
    DmMotor& operator=(DmMotor&&) = delete;

    ~DmMotor() = default;

    void configure(const Config& config) {
        esc_id_    = config.esc_id;
        master_id_ = config.master_id;

        encoder_zero_point_ = config.encoder_zero_point & (kRawAngleModulus - 1);

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        multi_turn_encoder_count_ = 0;
        last_raw_angle_           = encoder_zero_point_;

        const double sign            = config.reversed ? -1 : 1;
        const double reduction_ratio = config.reduction_ratio;

        raw_angle_to_angle_coefficient_ =
            sign / reduction_ratio * (2 * config.position_max) / kRawAngleMax;
        angle_to_raw_angle_coefficient_ = 1 / raw_angle_to_angle_coefficient_;

        raw_velocity_to_velocity_coefficient_ =
            sign / reduction_ratio * (2 * config.velocity_max) / kRawVelocityMax;
        velocity_to_raw_velocity_coefficient_ = 1 / raw_velocity_to_velocity_coefficient_;

        raw_torque_to_torque_coefficient_ =
            sign * reduction_ratio * (2 * config.torque_max) / kRawTorqueMax;
        torque_to_raw_torque_coefficient_ = 1 / raw_torque_to_torque_coefficient_;

        // Note: unlike LkMotor, max_torque_ here is NOT the datasheet peak torque (11 N*m at 0.8
        // over current, 12.5 N*m at 0.98, rated 3.5 N*m for a J4310). It is the largest torque the
        // MIT frame can actually express, which is TMAX. A controller saturating against the
        // datasheet number would just be clipped again by the protocol.
        max_torque_ = config.torque_max * reduction_ratio;

        kp_ = config.kp;
        kd_ = config.kd;

        offline_timeout_      = config.offline_timeout;
        error_retry_interval_ = config.error_retry_interval;

        // A DM feedback frame of all zero bytes decodes to (-PMAX, -VMAX, -TMAX), i.e. full
        // negative torque at the negative position limit. Never decode before the first frame
        // has actually arrived.
        received_       = false;
        error_          = Error::kDisabled;
        online_         = false;
        offline_count_  = 0;
        error_retry_countdown_ = 0;
        last_sequence_  = sequence_.load(std::memory_order::relaxed);

        angle_           = 0.0;
        velocity_        = 0.0;
        torque_          = 0.0;
        temperature_     = 0.0;
        temperature_mos_ = 0.0;

        *max_torque_output_ = max_torque();
    }

    void store_status(std::span<const std::byte> can_data) {
        if (can_data.size() != 8) [[unlikely]]
            return;

        // The fixed extent overload is noexcept; the dynamic one throws. This runs on the
        // transport thread, where nothing may throw.
        can_packet_.store(CanPacket8{can_data.first<8>()}, std::memory_order::relaxed);
        sequence_.fetch_add(1, std::memory_order::relaxed);
    }

    auto id() const noexcept -> std::uint32_t { return esc_id_; }
    auto recv_id() const noexcept -> std::uint32_t { return master_id_; }
    auto send_id() const noexcept -> std::uint32_t { return esc_id_; }

    bool match_then_store_status(std::uint32_t can_id, std::span<const std::byte> can_data) {
        if (can_id != recv_id())
            return false;
        store_status(can_data);
        return true;
    }

    /// Must be called once per control cycle: the offline watchdog and the clear error backoff
    /// are both counted in calls.
    void update_status() {
        const auto sequence = sequence_.load(std::memory_order::relaxed);
        if (sequence != last_sequence_) {
            last_sequence_ = sequence;
            received_      = true;
            offline_count_ = offline_timeout_;
        } else if (offline_count_ > 0)
            --offline_count_;
        online_ = offline_count_ > 0;

        if (error_retry_countdown_ > 0)
            --error_retry_countdown_;

        if (received_) [[likely]] {
            const struct [[gnu::packed]] {
                uint8_t id_and_error;
                uint8_t angle_high;
                uint8_t angle_low;
                uint8_t velocity_high;
                uint8_t velocity_low_and_torque_high;
                uint8_t torque_low;
                uint8_t temperature_mos;
                uint8_t temperature_rotor;
            } feedback alignas(CanPacket8) =
                std::bit_cast<decltype(feedback)>(can_packet_.load(std::memory_order::relaxed));

            error_ = static_cast<Error>(feedback.id_and_error >> 4);

            // Angle unit: rad. Position is 16 bits, velocity and torque 12 bits each, all mapped
            // linearly onto [-max, +max], so raw zero is the negative limit and not the origin.
            const int raw_angle = (int{feedback.angle_high} << 8) | feedback.angle_low;
            auto calibrated_raw_angle = raw_angle - encoder_zero_point_;
            if (!multi_turn_angle_enabled_) {
                // Normalize into (-modulus/2, modulus/2]. The DM position wraps at +-PMAX,
                // which is about four turns, so there is no single turn range to fold into.
                calibrated_raw_angle =
                    ((calibrated_raw_angle + kRawAngleModulus / 2) & (kRawAngleModulus - 1))
                    - kRawAngleModulus / 2;
                multi_turn_encoder_count_ = calibrated_raw_angle;
            } else {
                // Same minimal difference trick as LkMotor, valid because the modulus is a
                // power of two. Doing this in the raw domain is what keeps the wrap worth
                // 2*PMAX instead of one revolution.
                auto diff =
                    (calibrated_raw_angle - multi_turn_encoder_count_) & (kRawAngleModulus - 1);
                if (diff > (kRawAngleModulus >> 1))
                    diff -= kRawAngleModulus;
                multi_turn_encoder_count_ += diff;
            }
            last_raw_angle_ = raw_angle;
            angle_ =
                raw_angle_to_angle_coefficient_ * static_cast<double>(multi_turn_encoder_count_);

            // Velocity unit: rad/s
            const int raw_velocity = (int{feedback.velocity_high} << 4)
                                   | (feedback.velocity_low_and_torque_high >> 4);
            velocity_ = raw_velocity_to_velocity_coefficient_
                      * (static_cast<double>(raw_velocity) - kRawVelocityZero);

            // Torque unit: N*m
            const int raw_torque = (int{feedback.velocity_low_and_torque_high & 0x0F} << 8)
                                 | feedback.torque_low;
            torque_ = raw_torque_to_torque_coefficient_
                    * (static_cast<double>(raw_torque) - kRawTorqueZero);

            // Temperature unit: celsius. The rotor coil is the one the over temperature
            // protection actually watches.
            temperature_     = static_cast<double>(feedback.temperature_rotor);
            temperature_mos_ = static_cast<double>(feedback.temperature_mos);
        }

        *angle_output_           = angle();
        *raw_angle_output_       = last_raw_angle();
        *velocity_output_        = velocity();
        *torque_output_          = torque();
        *temperature_output_     = temperature();
        *temperature_mos_output_ = temperature_mos();
        *error_code_output_      = static_cast<uint8_t>(error());
        *online_output_          = online();
    }

    int calibrate_zero_point() {
        multi_turn_encoder_count_ = 0;
        encoder_zero_point_       = last_raw_angle_;
        return encoder_zero_point_;
    }

    int last_raw_angle() const { return last_raw_angle_; }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return max_torque_; }
    double temperature() const { return temperature_; }
    double temperature_mos() const { return temperature_mos_; }

    Error error() const { return error_; }
    bool enabled() const { return error_ == Error::kEnabled; }
    bool online() const { return online_; }

    /// @brief Bring the motor out of the power-on default state. Until this is acknowledged the
    /// driver ignores every control frame, so it is the first thing any DM motor needs.
    constexpr static CanPacket8 generate_enable_command() { return generate_control_frame(0xFC); }

    /// @brief Return the motor to the disabled state. Also the safest thing to put on the bus
    /// when there is nothing to command: feedback is poll driven, so a bus that goes quiet stops
    /// reporting status altogether.
    constexpr static CanPacket8 generate_disable_command() { return generate_control_frame(0xFD); }

    /// @brief Clear a latched fault (over temperature and friends).
    constexpr static CanPacket8 generate_clear_error_command() {
        return generate_control_frame(0xFB);
    }

    /// @brief Set the current output shaft position as the driver's zero, and zero the position
    /// setpoint with it.
    /// @note This is a control frame addressed to the motor id, not a 0x7FF register write.
    constexpr static CanPacket8 generate_save_zero_command() { return generate_control_frame(0xFE); }

    /// @brief The host sends this command to drive the motor with the MIT law
    /// tau = kp * (angle - angle_measured) + kd * (velocity - velocity_measured) + torque.
    /// @note Each parameter is clamped into the mapped range before being quantized. Without the
    /// clamp an out of range setpoint wraps around the fixed point field and comes out as the
    /// opposite extreme, which for the 16 bit position field means a full swing command.
    /// @note NaN means "not commanded" and drops the matching gain, so an unwired position input
    /// cannot be mistaken for a command to drive to the zero point.
    CanPacket8 generate_mit_command(
        double control_angle, double control_velocity, double control_torque, double kp,
        double kd) const {
        const bool angle_commanded    = !std::isnan(control_angle);
        const bool velocity_commanded = !std::isnan(control_velocity);

        if (std::isnan(kp))
            kp = kp_;
        if (std::isnan(kd))
            kd = kd_;

        if (!angle_commanded)
            kp = 0.0;
        if (!angle_commanded && !velocity_commanded)
            kd = 0.0;

        const int raw_angle    = to_raw_angle(angle_commanded ? control_angle : 0.0);
        const int raw_velocity = to_raw_velocity(velocity_commanded ? control_velocity : 0.0);
        const int raw_torque   = to_raw_torque(std::isnan(control_torque) ? 0.0 : control_torque);
        const int raw_kp       = to_raw(kp, 0, kKpMax, kRawGainMax);
        const int raw_kd       = to_raw(kd, 0, kKdMax, kRawGainMax);

        // Five parameters bit packed into eight bytes, so a byte array says it better than a
        // struct of bit fields would.
        const std::array<uint8_t, 8> command{
            static_cast<uint8_t>(raw_angle >> 8),
            static_cast<uint8_t>(raw_angle),
            static_cast<uint8_t>(raw_velocity >> 4),
            static_cast<uint8_t>(((raw_velocity & 0x0F) << 4) | (raw_kp >> 8)),
            static_cast<uint8_t>(raw_kp),
            static_cast<uint8_t>(raw_kd >> 4),
            static_cast<uint8_t>(((raw_kd & 0x0F) << 4) | (raw_torque >> 8)),
            static_cast<uint8_t>(raw_torque)};

        return std::bit_cast<CanPacket8>(command);
    }

    CanPacket8 generate_mit_command() const {
        return generate_mit_command(
            control_angle(), control_velocity(), control_torque(), control_kp(), control_kd());
    }

    /// @brief The frame to put on the bus this cycle.
    /// @note Which of the three usual MIT shapes comes out is decided by which inputs are wired,
    /// the same way LkMotor picks between its four command frames:
    /// - only /control_torque: kp = kd = 0, pure torque
    /// - /control_velocity too: kp = 0, constant velocity
    /// - /control_angle too: full position loop, with the other two as feed forward
    /// @note The enable and clear error frames return early, before the MIT frame is built. A
    /// state machine that builds the control frame first and then overwrites it hides from the
    /// caller that no command went out this cycle.
    CanPacket8 generate_command() {
        if (error_ == Error::kDisabled)
            return generate_enable_command();

        if (error_ != Error::kEnabled) [[unlikely]] {
            if (error_retry_countdown_ > 0)
                return generate_disable_command();
            error_retry_countdown_ = error_retry_interval_;
            return generate_clear_error_command();
        }

        return generate_mit_command();
    }

    double control_angle() const {
        // has_provider(), not ready(): an optional input with nobody upstream is bound to a
        // default constructed 0.0 and reads as ready, which would turn "unwired" into "commanded
        // to the zero point".
        if (control_angle_.has_provider()) [[likely]]
            return *control_angle_;
        else
            return kNan;
    }

    double control_velocity() const {
        if (control_velocity_.has_provider()) [[likely]]
            return *control_velocity_;
        else
            return kNan;
    }

    double control_torque() const {
        if (control_torque_.has_provider()) [[likely]]
            return *control_torque_;
        else
            return kNan;
    }

    double control_kp() const {
        if (control_kp_.has_provider()) [[likely]]
            return *control_kp_;
        else
            return kNan;
    }

    double control_kd() const {
        if (control_kd_.has_provider()) [[likely]]
            return *control_kd_;
        else
            return kNan;
    }

private:
    /// Enable, disable, clear error and save zero all share this shape.
    constexpr static CanPacket8 generate_control_frame(uint8_t id) {
        const struct [[gnu::packed]] {
            uint8_t placeholder[7];
            uint8_t id;
        } command alignas(CanPacket8){
            .placeholder = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, .id = id};
        return std::bit_cast<CanPacket8>(command);
    }

    static int to_raw(double value, double min, double max, int raw_max) {
        return clamp_raw((value - min) * raw_max / (max - min), raw_max);
    }

    int to_raw_angle(double angle) const {
        const double raw =
            static_cast<double>(encoder_zero_point_) + angle_to_raw_angle_coefficient_ * angle;
        return clamp_raw(raw, kRawAngleMax);
    }

    int to_raw_velocity(double velocity) const {
        return clamp_raw(
            kRawVelocityZero + velocity_to_raw_velocity_coefficient_ * velocity, kRawVelocityMax);
    }

    int to_raw_torque(double torque) const {
        return clamp_raw(
            kRawTorqueZero + torque_to_raw_torque_coefficient_ * torque, kRawTorqueMax);
    }

    /// Guards NaN before the cast: std::clamp passes NaN straight through and casting it to int
    /// is undefined. Callers filter NaN for meaning, this filters it for safety.
    static int clamp_raw(double raw, int raw_max) {
        if (std::isnan(raw)) [[unlikely]]
            return 0;
        return static_cast<int>(std::round(std::clamp(raw, 0.0, static_cast<double>(raw_max))));
    }

    // Limits
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    // DM maps position onto 16 bits and velocity, torque, kp and kd onto 12 bits each. The
    // modulus is what the counter wraps on, the max is what the linear map spans.
    static constexpr int kRawAngleModulus = 1 << 16;
    static constexpr int kRawAngleMax     = kRawAngleModulus - 1;
    static constexpr int kRawAngleZero    = kRawAngleModulus / 2;
    static constexpr int kRawVelocityMax  = (1 << 12) - 1;
    static constexpr int kRawTorqueMax    = (1 << 12) - 1;
    static constexpr int kRawGainMax      = (1 << 12) - 1;

    // Constants
    // kp and kd ranges are fixed by the protocol, not by the motor, so they are not configurable.
    static constexpr double kKpMax = 500.0;
    static constexpr double kKdMax = 5.0;

    static constexpr double kRawVelocityZero = kRawVelocityMax / 2.0;
    static constexpr double kRawTorqueZero   = kRawTorqueMax / 2.0;

    std::uint32_t esc_id_ = 0, master_id_ = 0;

    bool multi_turn_angle_enabled_;
    int encoder_zero_point_;

    double kp_ = 0.0, kd_ = 0.0;

    // Coefficients
    double raw_angle_to_angle_coefficient_, angle_to_raw_angle_coefficient_;
    double raw_velocity_to_velocity_coefficient_, velocity_to_raw_velocity_coefficient_;
    double raw_torque_to_torque_coefficient_, torque_to_raw_torque_coefficient_;

    // Status
    std::atomic<CanPacket8> can_packet_;
    std::atomic<std::uint32_t> sequence_ = 0;
    std::uint32_t last_sequence_ = 0;

    bool received_ = false;
    Error error_ = Error::kDisabled;
    bool online_ = false;
    int offline_count_ = 0, offline_timeout_ = 0;
    int error_retry_countdown_ = 0, error_retry_interval_ = 0;

    std::int64_t multi_turn_encoder_count_ = 0;
    int last_raw_angle_ = kRawAngleZero;

    double angle_;
    double velocity_;
    double torque_;
    double max_torque_;
    double temperature_;
    double temperature_mos_;

    hcs_executor::Component::OutputInterface<double> angle_output_;
    hcs_executor::Component::OutputInterface<int64_t> raw_angle_output_;
    hcs_executor::Component::OutputInterface<double> velocity_output_;
    hcs_executor::Component::OutputInterface<double> torque_output_;
    hcs_executor::Component::OutputInterface<double> temperature_output_;
    hcs_executor::Component::OutputInterface<double> temperature_mos_output_;
    hcs_executor::Component::OutputInterface<double> max_torque_output_;
    hcs_executor::Component::OutputInterface<uint8_t> error_code_output_;
    hcs_executor::Component::OutputInterface<bool> online_output_;

    hcs_executor::Component::InputInterface<double> control_angle_;
    hcs_executor::Component::InputInterface<double> control_velocity_;
    hcs_executor::Component::InputInterface<double> control_torque_;
    hcs_executor::Component::InputInterface<double> control_kp_;
    hcs_executor::Component::InputInterface<double> control_kd_;
};

} // namespace hcs_core::hardware::device
