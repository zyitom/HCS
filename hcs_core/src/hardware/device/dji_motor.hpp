#pragma once

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <optional>
#include <string>
#include <utility>

#include <hcs_executor/component.hpp>
#include <hcs_base/protocol/endian_promise.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/util/required.hpp"

namespace hcs_core::hardware::device {

/// DJI C6xx / GM6020 电调的协议。只写协议：全部成员都在控制线程上，整帧由端口包装层
/// （board::Can<DjiMotor>）交进来；掉线计数和健康输出也在包装层。
class DjiMotor {
public:
    /// 这个驱动要的总线：经典 CAN 2.0，8 字节帧，电机出厂的 1 Mbps。
    /// 总线声明成别的，电机往上接的时候就会被拒绝。
    static constexpr std::uint32_t kCanBitrate = 1'000'000;
    static constexpr bool kCanFd = false;

    enum class Type : std::uint8_t { kGM6020, kGM6020Voltage, kM3508, kM2006 };

    /// 反馈帧的 DATA[7]。旧版电调固件的手册把这个字节写成空，发的是 0，解出来是 kNone，
    /// 所以新旧固件上读它都是安全的。几种情况同时成立时，电调报最严重的那个，也就是最小的
    /// 非零码。手册没列出的码（6、9 及以上）原样保留，不归并到某个已知的码上。
    enum class Error : std::uint8_t {
        kNone                 = 0,
        kStorageUnreachable   = 1, // 仅上电自检
        kSupplyOverVoltage    = 2, // 仅上电自检
        kPhaseDisconnected    = 3,
        kPositionSensorLost   = 4,
        kMotorOverTemperature = 5, // >= 180 ℃
        kCalibrationFailed    = 7,
        kMotorOverheat        = 8, // >= 125 ℃
    };

    /// 聚合类型：接线表里写了什么一目了然。motor_type 和 id 必填（util::Required）；
    /// reduction_ratio 默认是这个型号自带的减速箱。
    struct Config {
        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reduction_ratio(double value) { return reduction_ratio = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
        Config& set_offline_timeout(int value) { return offline_timeout = value, *this; }

        util::Required<Type> motor_type;
        /// 电调 id，由拨码开关 / 指示灯闪烁次数设定（1..8）。
        util::Required<std::uint8_t> id;
        int encoder_zero_point = 0;
        /// 转子到输出轴的减速比。不设：型号自带的减速箱（M3508 是 3591/187，M2006 是 36，
        /// GM6020 是 1）；设了就替换它，比如外加了一级减速。
        std::optional<double> reduction_ratio = std::nullopt;
        bool reversed = false;
        bool multi_turn_angle_enabled = false;
        /// 连续多少拍没有反馈算掉线。电调和控制回路都跑在 1 kHz 左右，但时钟不同源，
        /// 所以某一拍没有新帧（或者来了两帧）是正常的；连着这么多拍都没有才不正常。
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
        const std::string& name_prefix, const Config& config)
        : angle_(0.0)
        , velocity_(0.0)
        , torque_(0.0) {
        status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_output_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);
        status_component.register_output(
            name_prefix + "/error_code", error_code_output_, std::uint8_t{0});

        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);

        configure(config);
    }

    DjiMotor(const DjiMotor&) = delete;
    DjiMotor& operator=(const DjiMotor&) = delete;
    DjiMotor(DjiMotor&&) = delete;
    DjiMotor& operator=(DjiMotor&&) = delete;

    ~DjiMotor() = default;

    [[nodiscard]] static constexpr std::uint32_t feedback_id(Type type, std::uint8_t index) {
        switch (type) {
        case Type::kGM6020:
        case Type::kGM6020Voltage: return 0x204 + index;
        case Type::kM3508:
        case Type::kM2006: return 0x200 + index;
        }
        return 0;
    }

    [[nodiscard]] static constexpr std::uint32_t command_id(Type type, std::uint8_t index) {
        switch (type) {
        case Type::kGM6020: return index <= 4 ? 0x1FE : 0x2FE;
        case Type::kGM6020Voltage: return index <= 4 ? 0x1FF : 0x2FF;
        case Type::kM3508:
        case Type::kM2006: return index <= 4 ? 0x200 : 0x1FF;
        }
        return 0;
    }

    auto id() const noexcept -> std::uint8_t { return id_; }
    [[nodiscard]] std::uint32_t feedback_id() const noexcept { return feedback_id(type_, id_); }
    [[nodiscard]] std::uint32_t command_id() const noexcept { return command_id(type_, id_); }

    /// 一帧新的反馈。首帧到来之前什么都不解，每个输出都保持注册时的初值，
    /// 所以多圈模式不会拿一个电机从没报过的位置去起算圈数。
    void on_frame(CanPacket8 frame) {
        decode(std::bit_cast<DjiMotorFeedback>(frame));

        *angle_output_ = angle();
        *velocity_output_ = velocity();
        *torque_output_ = torque();
        *error_code_output_ = static_cast<std::uint8_t>(error());
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
        const hcs_utility::be_int16_t control_current = static_cast<std::int16_t>(current);

        return std::bit_cast<CanPacket8::Quarter>(control_current);
    }

    /// 这台电机写的是共享帧 command_id() 里哪个 2 字节的槽位；id 为 0 时没有。
    /// 同一路总线上的两台电机不能既同指令 id 又同槽位（id 5 的 M3508 和 id 1 的电压模式
    /// GM6020 就撞了：都写 0x1FF 的槽 0）。
    std::optional<std::size_t> command_slot() const noexcept {
        if (id_ == 0)
            return std::nullopt;
        return static_cast<std::size_t>((id_ - 1) % 4);
    }

    /// 把这一拍的电流交给所在的总线。电调一帧管四台电机：同一路总线上指令 id 相同的电机，
    /// 各自写同一个共享帧里自己的 2 字节槽位 (id - 1) % 4。安全拍写零电流。
    /// id 为 0（没分配）时仍然会把这一帧开出来，但不碰任何槽位。
    template <class BusFrames>
    void append_command(BusFrames& bus, bool safe) const {
        auto& packet = bus.shared_frame(command_id());
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

    /// error() 是故障还是只是警告。kMotorOverheat（>= 125 ℃）是手册里唯一排在其余之下的码；
    /// 别的非零码，包括没列出的，一律当故障。条件消失后电调会自己把码清掉。
    bool faulted() const { return error_ != Error::kNone && error_ != Error::kMotorOverheat; }

private:
    /// 应用配置。只由构造函数调一次：驱动接到端口上之后不再重新配置。
    void configure(const Config& config) {
        type_ = config.motor_type;
        id_ = config.id;
        const double reduction_ratio =
            config.reduction_ratio.value_or(default_reduction_ratio(type_));
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

        angle_ = 0.0;
        velocity_ = 0.0;
        torque_ = 0.0;
        temperature_ = 0.0;

        *max_torque_output_ = max_torque();
    }

    void decode(const auto& feedback) {
        // 温度，单位摄氏度
        temperature_ = static_cast<double>(feedback.temperature);

        // 底层类型是定死的，所以任何一个字节都是合法的 Error 值，列没列出来都一样。
        error_ = static_cast<Error>(feedback.error);

        // 角度，单位 rad
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

        // 速度，单位 rad/s
        velocity_ = raw_velocity_to_velocity_coefficient_ * static_cast<double>(feedback.velocity);

        // 力矩，单位 N*m
        torque_ = raw_current_to_torque_coefficient_ * static_cast<double>(feedback.current);
    }

    struct alignas(std::uint64_t) DjiMotorFeedback {
        hcs_utility::be_int16_t angle;
        hcs_utility::be_int16_t velocity;
        hcs_utility::be_int16_t current;
        std::uint8_t temperature;
        std::uint8_t error; // 错误码；旧固件上为空（0）
    };
    static_assert(sizeof(DjiMotorFeedback) == sizeof(CanPacket8));

    Type type_ = Type::kM3508;
    std::uint8_t id_ = 0;

    static constexpr int kRawAngleMax = 8192;
    int encoder_zero_point_, last_raw_angle_;

    bool multi_turn_angle_enabled_;
    std::int64_t angle_multi_turn_;

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
    hcs_executor::Component::OutputInterface<std::uint8_t> error_code_output_;

    hcs_executor::Component::InputInterface<double> control_torque_;
};

} // namespace hcs_core::hardware::device
