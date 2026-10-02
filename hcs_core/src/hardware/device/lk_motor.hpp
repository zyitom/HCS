#pragma once

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
#include <string>
#include <utility>

#include <hcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/util/required.hpp"

// LK 反馈/命令帧为原生类型 packed，双向依赖小端主机。
static_assert(std::endian::native == std::endian::little, "wire structs assume a LE host");

namespace hcs_core::hardware::device {

/// LK（瓴控）MF / MG / MH 电机，CAN 协议 V2.36。只写协议：除 accepts() 之外全部成员都在
/// 控制线程上，整帧由端口包装层（board::Can<LkMotor>）交进来；掉线计数和健康输出也在包装层。
class LkMotor {
public:
    /// 这个驱动要的总线：经典 CAN 2.0，8 字节帧，电机出厂的 1 Mbps。
    /// 总线声明成别的，电机往上接的时候就会被拒绝。
    static constexpr std::uint32_t kCanBitrate = 1'000'000;
    static constexpr bool kCanFd = false;

    enum class Type : std::uint8_t {
        kMG5010Ei10,
        kMG4010Ei10,
        kMG6012Ei8,
        kMG4005Ei10,
        kMG5010Ei36,
        kMHF7015,
    };

    /// 用哪一种指令帧驱动这台电机。在配置里选定，绝不从接线推断：驱动只注册所选模式用到的输入。
    /// 以前只要接了 /control_velocity（比如上位机角度环的输出，本来是喂给上位机速度环的），
    /// 力矩模式的电机就会悄悄切到 0xA2，在电机里面把速度环闭上，/control_torque 则降格成电流限幅。
    enum class ControlMode : std::uint8_t {
        /// 0xA1，取 /control_torque。所有的环都由上位机闭。
        kTorque,
        /// 0xA2，取 /control_velocity，/control_torque 作电流限幅。
        kVelocity,
        /// 0xA3 / 0xA4，取 /control_angle，/control_velocity 作速度限幅。
        kAngle,
        /// 0xA7 / 0xA8，取 /control_angle_shift，/control_velocity 作速度限幅。
        kAngleShift,
    };

    /// 聚合类型：接线表里写了什么一目了然。motor_type、control_mode、can_id 必填
    /// （util::Required）：漏写一个就编不过，而不是悄悄替你挑一个模式或者 id 0。
    struct Config {
        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
        Config& set_offline_timeout(int value) { return offline_timeout = value, *this; }

        util::Required<Type> motor_type;
        /// 电机构造时就定死：它决定有哪些输入。
        util::Required<ControlMode> control_mode;
        util::Required<std::uint32_t> can_id; // 指令与反馈同 id（0x140+n）
        /// 在驱动已经减掉的零点之上再加的软件偏移：状态帧里的编码器字段是（原始值 - ROM 偏移），
        /// 见协议命令 0x90。
        int encoder_zero_point = 0;
        bool reversed = false;
        bool multi_turn_angle_enabled = false;
        /// 连续多少拍没有状态帧算掉线。
        int offline_timeout = 100;
    };

    LkMotor(
        hcs_executor::Component& status_component, hcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : control_mode_(config.control_mode) {
        status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
        status_component.register_output(name_prefix + "/raw_angle", raw_angle_output_, 0);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_output_, 0.0);
        status_component.register_output(name_prefix + "/temperature", temperature_output_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);

        // 只注册所选模式用到的。全部是可选输入：没接线的输入读出来是 NaN，绝不是 0。
        switch (control_mode_) {
        case ControlMode::kTorque:
            command_component.register_input( //
                name_prefix + "/control_torque", control_torque_, false);
            break;
        case ControlMode::kVelocity:
            command_component.register_input( //
                name_prefix + "/control_velocity", control_velocity_, false);
            command_component.register_input( //
                name_prefix + "/control_torque", control_torque_, false);
            break;
        case ControlMode::kAngle:
            command_component.register_input( //
                name_prefix + "/control_angle", control_angle_, false);
            command_component.register_input( //
                name_prefix + "/control_velocity", control_velocity_, false);
            break;
        case ControlMode::kAngleShift:
            command_component.register_input( //
                name_prefix + "/control_angle_shift", control_angle_shift_, false);
            command_component.register_input( //
                name_prefix + "/control_velocity", control_velocity_, false);
            break;
        }

        configure(config);
    }

    [[nodiscard]] std::uint32_t feedback_id() const noexcept { return can_id_; }
    [[nodiscard]] std::uint32_t command_id() const noexcept { return can_id_; }

    /// 我们 id 上的一帧回复是不是按"读取电机状态 2"（0x9C）的布局排的：温度、iq、速度、编码器。
    /// 控制命令 0xA1~0xA8 的回复都是这个布局，只是命令字节是各自的。别的回复（状态 1 / 3、
    /// 编码器、角度、参数、设零点……）id 相同但布局不同，把它们当状态解会让圈数计数跳变：
    /// 这些帧不解码，也不算反馈。
    ///
    /// @note 运行在传输线程上。除了这一帧本身，它什么都不读。
    [[nodiscard]] static bool accepts(CanPacket8 frame) noexcept {
        return is_status_reply(std::to_integer<std::uint8_t>(frame.as_bytes()[0]));
    }

    /// 一帧新的状态。首帧到来之前什么都不解，每个输出都保持注册时的初值，
    /// 所以多圈模式不会拿一个电机从没报过的位置去起算圈数。
    void on_frame(CanPacket8 frame) {
        const struct [[gnu::packed]] {
            std::uint8_t command;
            std::int8_t temperature;
            std::int16_t current;
            std::int16_t velocity;
            std::uint16_t encoder;
        } feedback alignas(CanPacket8) = std::bit_cast<decltype(feedback)>(frame);

        // 温度，单位摄氏度
        temperature_ = static_cast<double>(feedback.temperature);

        // 角度，单位 rad
        const auto raw_angle = feedback.encoder;
        auto calibrated_raw_angle = feedback.encoder - encoder_zero_point_;
        if (calibrated_raw_angle < 0)
            calibrated_raw_angle += raw_angle_modulus_;
        if (!multi_turn_angle_enabled_) {
            angle_ =
                status_angle_to_angle_coefficient_ * static_cast<double>(calibrated_raw_angle);
            if (angle_ < 0)
                angle_ += 2 * std::numbers::pi;
        } else {
            // 求两个角度之间的最小差值，归一化到 (-raw_angle_modulus_/2, raw_angle_modulus_/2]。
            // 这里用位运算来做，只有 raw_angle_modulus_ 是 2 的幂时才成立。
            auto diff =
                (calibrated_raw_angle - multi_turn_encoder_count_) & (raw_angle_modulus_ - 1);
            if (diff > (raw_angle_modulus_ >> 1))
                diff -= raw_angle_modulus_;

            multi_turn_encoder_count_ += diff;
            angle_ = status_angle_to_angle_coefficient_
                   * static_cast<double>(multi_turn_encoder_count_);
        }
        last_raw_angle_ = raw_angle;

        // 速度，单位 rad/s。线上的单位是 1 dps，和 0xA2 命令里的 0.01 dps 不一样。
        velocity_ =
            status_velocity_to_velocity_coefficient_ * static_cast<double>(feedback.velocity);

        // 力矩，单位 N*m
        torque_ =
            status_current_to_torque_coefficient_ * static_cast<double>(feedback.current);

        *angle_output_ = angle();
        *raw_angle_output_ = last_raw_angle();
        *velocity_output_ = velocity();
        *torque_output_ = torque();
        *temperature_output_ = temperature();
    }

    std::int64_t calibrate_zero_point() {
        multi_turn_encoder_count_ = 0;
        encoder_zero_point_ = last_raw_angle_;
        return encoder_zero_point_;
    }

    std::int64_t last_raw_angle() const { return last_raw_angle_; }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return max_torque_; }
    double temperature() const { return temperature_; }
    /// 状态 2 里没有错误字段；要知道故障得轮询 0x9A，这里没有实现。
    constexpr bool faulted() const { return false; }

    /// @brief 把电机从开启状态（上电后的默认状态）切到关闭状态，清除电机的圈数和之前收到的
    /// 控制指令。指示灯由常亮变为慢闪。这时电机仍然会回复控制指令，但不执行动作。
    constexpr static CanPacket8 generate_shutdown_command() {
        const struct [[gnu::packed]] {
            std::uint8_t id;
            std::uint8_t placeholder[7]{};
        } command alignas(CanPacket8){.id = 0x80};
        return std::bit_cast<CanPacket8>(command);
    }

    /// @brief 把电机从关闭状态切到开启状态。指示灯由慢闪变为常亮。
    /// 这之后发控制指令就能控制电机动作。
    constexpr static CanPacket8 generate_startup_command() {
        const struct [[gnu::packed]] {
            std::uint8_t id = 0x88;
            std::uint8_t placeholder[7]{};
        } command alignas(CanPacket8){};
        return std::bit_cast<CanPacket8>(command);
    }

    /// @brief 让电机失能，但不清除电机的运行状态。再发控制指令又能控制电机动作。
    constexpr static CanPacket8 generate_disable_command() {
        // 注意：这里发的不是真正的失能报文，而是力矩为 0 的力矩控制报文——
        // 因为失能报文不会让电机回传状态。
        const struct [[gnu::packed]] {
            std::uint8_t id = 0xA1;
            std::uint8_t placeholder0[3]{};
            std::int16_t current = 0;
            std::uint8_t placeholder1[2]{};
        } command alignas(CanPacket8){};
        return std::bit_cast<CanPacket8>(command);
    }

    /// @brief 读取电机当前的温度、转矩电流（MF、MG）/ 输出功率（MS）、速度和编码器位置。
    constexpr static CanPacket8 generate_status_request() {
        const struct [[gnu::packed]] {
            std::uint8_t id = 0x9C;
            std::uint8_t placeholder[7]{};
        } request alignas(CanPacket8){};
        return std::bit_cast<CanPacket8>(request);
    }

    /// @brief 上位机用这条命令控制电机的转矩电流输出。
    /// @note 电机收到命令后回复上位机。回复的数据和 generate_status_request 命令的相同
    /// （只有第 0 字节的命令字不同，这里是 0xA1）。
    CanPacket8 generate_torque_command(double control_torque) const {
        if (std::isnan(control_torque))
            return generate_disable_command();

        /// @param current 取值范围 -2048~2048，对应 MF 电机的实际转矩电流 -16.5A~16.5A、
        /// MG 电机的 -33A~33A。母线电流和电机的实际力矩因电机型号而异。
        const struct [[gnu::packed]] {
            std::uint8_t id = 0xA1;
            std::uint8_t placeholder0[3]{};
            std::int16_t current;
            std::uint8_t placeholder1[2]{};
        } command alignas(CanPacket8){.current = to_command_current(control_torque)};

        return std::bit_cast<CanPacket8>(command);
    }

    CanPacket8 generate_torque_command() const { return generate_torque_command(control_torque()); }

    /// @brief 上位机用这条命令控制电机的速度，可以带一个力矩限幅。
    /// @note 固件有三种：
    /// - A 版：只有 0xA2，力矩限幅字段被忽略。
    /// - B 版：0xA2 和 0xAD；0xAD 是专门带力矩限幅的那一条。
    /// - C 版：只有 0xA2，力矩限幅字段有效。
    /// 运行时分辨不出是哪一种，所以这里一律发 0xA2 以求通用（A 和 C），
    /// 代价是用不上 B 版专门的 0xAD。
    /// 回复的布局和 generate_status_request 的相同（命令字 = 0xA2）。
    CanPacket8
        generate_velocity_command(double control_velocity, double torque_limit = kNan) const {
        if (std::isnan(control_velocity))
            return generate_disable_command();

        /// @param torque_limit std::int16_t，取值范围 -2048~2048，对应 MF 电机的实际转矩电流
        /// -16.5A~16.5A、MG 电机的 -33.0A~33.0A。母线电流和电机的实际力矩因电机型号而异。
        /// @param velocity std::int32_t，对应实际速度 0.01 dps/LSB；
        struct [[gnu::packed]] {
            std::uint8_t id = 0xA2;
            std::uint8_t placeholder{};
            std::int16_t current_limit = 0;
            std::int32_t velocity;
        } command alignas(CanPacket8){.velocity = to_command_velocity(control_velocity)};

        if (!std::isnan(torque_limit)) {
            // 这里继续用 0xA2；见上面关于兼容性的说明。
            command.current_limit = to_command_current(torque_limit);
        }

        return std::bit_cast<CanPacket8>(command);
    }

    CanPacket8 generate_velocity_command() const {
        return generate_velocity_command(control_velocity());
    }

    /// @brief 上位机用这条命令控制电机的位置（多圈角度）。
    /// @note 电机收到命令后回复上位机。回复的数据和 generate_status_request 命令的相同
    /// （只有第 0 字节的命令字不同，这里是 0xA3 / 0xA4）。
    CanPacket8 generate_angle_command(double control_angle, double velocity_limit = kNan) const {
        if (std::isnan(control_angle))
            return generate_disable_command();

        /// @param angle 实际位置按 0.01 deg/LSB，即 36000 代表 360 度；
        /// 电机的转向由目标位置和当前位置之差决定。
        /// @param velocity 电机转动的最大速度限制，实际速度按 1 dps/LSB，即 360 代表 360 dps。
        struct [[gnu::packed]] {
            std::uint8_t id = 0xA3;
            std::uint8_t placeholder{};
            std::uint16_t velocity_limit = 0;
            std::int32_t angle;
        } command alignas(CanPacket8){.angle = to_absolute_command_angle(control_angle)};

        if (!std::isnan(velocity_limit)) {
            command.id = 0xA4;

            velocity_limit = std::abs(velocity_to_command_velocity_coefficient_) * (1.0 / 100.0)
                           * velocity_limit;
            velocity_limit = std::round(
                std::clamp<double>(
                    velocity_limit, std::numeric_limits<std::uint16_t>::min(),
                    std::numeric_limits<std::uint16_t>::max()));
            command.velocity_limit = static_cast<std::uint16_t>(velocity_limit);
        }

        return std::bit_cast<CanPacket8>(command);
    }

    CanPacket8 generate_angle_command() const { return generate_angle_command(control_angle()); }

    CanPacket8 generate_angle_shift_command(
        double control_shift_angle, double velocity_limit = kNan) const {
        if (std::isnan(control_shift_angle))
            return generate_disable_command();

        /// @param angle 实际位置按 0.01 deg/LSB，即 36000 代表 360 度；
        /// 电机的转向由这个参数的符号决定。
        /// @param velocity_limit 电机转动的最大速度限制，实际速度按 1 dps/LSB，即 360 代表 360 dps。
        struct [[gnu::packed]] {
            std::uint8_t id = 0xA7;
            std::uint8_t placeholder{};
            std::uint16_t velocity_limit = 0;
            std::int32_t angle;
        } command alignas(CanPacket8){.angle = to_command_angle(control_shift_angle)};

        if (!std::isnan(velocity_limit)) {
            command.id = 0xA8;

            velocity_limit = std::abs(velocity_to_command_velocity_coefficient_) * (1.0 / 100.0)
                           * velocity_limit;
            velocity_limit = std::round(
                std::clamp<double>(
                    velocity_limit, std::numeric_limits<std::uint16_t>::min(),
                    std::numeric_limits<std::uint16_t>::max()));
            command.velocity_limit = static_cast<std::uint16_t>(velocity_limit);
        }

        return std::bit_cast<CanPacket8>(command);
    }

    CanPacket8 generate_angle_shift_command() const {
        return generate_angle_shift_command(control_angle_shift());
    }

    /// @brief 所配置模式的指令帧。各个生成函数在自己的给定是 NaN 时发失能帧。
    CanPacket8 generate_command() const {
        switch (control_mode_) {
        case ControlMode::kTorque: return generate_torque_command(control_torque());
        case ControlMode::kVelocity:
            return generate_velocity_command(control_velocity(), control_torque());
        case ControlMode::kAngle:
            return generate_angle_command(control_angle(), control_velocity());
        case ControlMode::kAngleShift:
            return generate_angle_shift_command(control_angle_shift(), control_velocity());
        }
        return generate_disable_command();
    }

    /// @brief 这台电机这一拍发什么，安全规则已经套上：安全拍发失能帧（零电流——和真正的 0x81
    /// 不同，它仍然会让电机回传状态）。给定是 NaN 时经 generate_command() 也是同样的结果。
    CanPacket8 command_frame(bool safe) const {
        return safe ? generate_disable_command() : generate_command();
    }

    /// @brief 把这一拍的帧交给所在的总线。一台电机一帧。
    template <class BusFrames>
    void append_command(BusFrames& bus, bool safe) const {
        bus.push(command_id(), command_frame(safe));
    }

    ControlMode control_mode() const noexcept { return control_mode_; }

    // has_provider(), not ready(): HCS 在接线时就把没人提供的可选输入绑到默认值 0.0，
    // ready() 从此恒为真。若在这里读 ready()，一个没接控制器的电机会把 0.0 当成
    // "角度 0 指令"发给电机；has_provider() 才能把"未接线"翻成 NaN → 失能帧。
    // 与 DmMotor 的同名访问器同语义。
    double control_torque() const {
        if (control_torque_.has_provider()) [[likely]]
            return *control_torque_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    double control_velocity() const {
        if (control_velocity_.has_provider()) [[likely]]
            return *control_velocity_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    double control_angle() const {
        if (control_angle_.has_provider()) [[likely]]
            return *control_angle_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    double control_angle_shift() const {
        if (control_angle_shift_.has_provider()) [[likely]]
            return *control_angle_shift_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

private:
    /// 应用配置。只由构造函数调一次：驱动接到端口上之后不再重新配置。
    void configure(const Config& config) {
        can_id_ = config.can_id;
        multi_turn_encoder_count_ = 0;
        last_raw_angle_ = 0;

        angle_ = 0.0;
        velocity_ = 0.0;
        torque_ = 0.0;
        temperature_ = 0.0;

        double torque_constant;
        double reduction_ratio;

        // ±2048 这个字段的转矩电流满量程，指令里和状态帧的 iq 里都一样：MG 是 33 A，
        // MF 和 MH 是 16.5 A（协议 V2.36 的命令 0x9C 和 0xA1）。
        current_max_ = kMgCurrentMax;

        switch (config.motor_type.get()) {
        case Type::kMG5010Ei10:
            raw_angle_modulus_ = 1 << 16;
            torque_constant = 0.1;
            reduction_ratio = 10.0;

            // 注意：max_torque_ 应当是电机**实际**的最大力矩，必须直接取自厂家的资料。
            // 它不参与计算，只作参考。
            // 不要简单地拿最大电流乘以转矩常数去算它：那样得到的值既不准也不可靠。
            max_torque_ = 7.0;
            break;
        case Type::kMG4010Ei10:
            raw_angle_modulus_ = 1 << 16;
            torque_constant = 0.07;
            reduction_ratio = 10.0;
            max_torque_ = 4.5;
            break;
        case Type::kMG6012Ei8:
            raw_angle_modulus_ = 1 << 16;
            torque_constant = 1.09 / 8.0;
            reduction_ratio = 8.0;
            max_torque_ = 16.0;
            break;
        case Type::kMG4005Ei10:
            raw_angle_modulus_ = 1 << 16;
            torque_constant = 0.06;
            reduction_ratio = 10.0;
            max_torque_ = 2.5;
            break;
        case Type::kMG5010Ei36:
            raw_angle_modulus_ = 1 << 16;
            torque_constant = 0.3;
            reduction_ratio = 36.0;
            max_torque_ = 25.0;
            break;
        case Type::kMHF7015:
            raw_angle_modulus_ = 1 << 16;
            torque_constant = 0.51;
            reduction_ratio = 1.0;
            max_torque_ = 2.42;
            current_max_ = kMfCurrentMax;
            break;
        default: std::unreachable();
        }

        // 确保 raw_angle_modulus_ 是 2 的幂
        encoder_zero_point_ = config.encoder_zero_point & (raw_angle_modulus_ - 1);

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;

        const double sign = config.reversed ? -1.0 : 1.0;

        status_angle_to_angle_coefficient_ = sign / raw_angle_modulus_ * 2 * std::numbers::pi;
        angle_to_command_angle_coefficient_ = sign * reduction_ratio * kRadToDeg * 100.0;

        status_velocity_to_velocity_coefficient_ = sign / reduction_ratio * kDegToRad;
        velocity_to_command_velocity_coefficient_ = sign * reduction_ratio * kRadToDeg * 100.0;

        status_current_to_torque_coefficient_ =
            sign * (current_max_ / kRawCurrentMax) * torque_constant * reduction_ratio;
        torque_to_command_current_coefficient_ = 1 / status_current_to_torque_coefficient_;

        *max_torque_output_ = max_torque();
    }

    static constexpr bool is_status_reply(std::uint8_t command) {
        return command == 0x9C || (command >= 0xA1 && command <= 0xA8);
    }

    std::int16_t to_command_current(double torque) const {
        double current = torque_to_command_current_coefficient_ * torque;
        current = std::round(std::clamp<double>(current, -kRawCurrentMax, kRawCurrentMax));
        return static_cast<std::int16_t>(current);
    }

    std::int32_t to_command_velocity(double velocity) const {
        velocity = velocity_to_command_velocity_coefficient_ * velocity;
        velocity = std::round(
            std::clamp<double>(
                velocity, std::numeric_limits<std::int32_t>::min(),
                std::numeric_limits<std::int32_t>::max()));
        return static_cast<std::int32_t>(velocity);
    }

    std::int32_t to_command_angle(double angle) const {
        angle = angle_to_command_angle_coefficient_ * angle;
        angle = std::round(
            std::clamp<double>(
                angle, std::numeric_limits<std::int32_t>::min(),
                std::numeric_limits<std::int32_t>::max()));
        return static_cast<std::int32_t>(angle);
    }

    std::int32_t to_absolute_command_angle(double angle) const {
        angle = angle_to_command_angle_coefficient_ * angle;
        const auto one_turn = std::abs(angle_to_command_angle_coefficient_) * 2 * std::numbers::pi;
        const auto encoder_offset =
            one_turn * static_cast<double>(encoder_zero_point_) / raw_angle_modulus_;

        angle += encoder_offset;
        if (multi_turn_angle_enabled_) {
            // 映射到离电机当前位置最近的那个等效目标圈。
            const auto current_command_angle =
                angle_to_command_angle_coefficient_ * angle_ + encoder_offset;
            if (one_turn > 0.0)
                angle += one_turn * std::round((current_command_angle - angle) / one_turn);
        } else {
            // 保留历史上单圈指令的映射方式。
            angle -= one_turn;
        }
        // hero encoder 专用，不能改

        angle = std::round(
            std::clamp<double>(
                angle, std::numeric_limits<std::int32_t>::min(),
                std::numeric_limits<std::int32_t>::max()));

        return static_cast<std::int32_t>(angle);
    }

    // 限值
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    // LK 协议把电流字段的原始范围 [-2048, 2048] 映射到 MG 电机的 ±33 A、
    // MF / MH 电机的 ±16.5 A。
    static constexpr int kRawCurrentMax = 2048;
    static constexpr double kMgCurrentMax = 33.0;
    static constexpr double kMfCurrentMax = 16.5;
    double current_max_;
    int raw_angle_modulus_;
    ControlMode control_mode_;

    // 常量
    static constexpr double kDegToRad = std::numbers::pi / 180;
    static constexpr double kRadToDeg = 180 / std::numbers::pi;

    bool multi_turn_angle_enabled_;
    int encoder_zero_point_;

    // 系数
    double status_angle_to_angle_coefficient_;
    double angle_to_command_angle_coefficient_;

    double status_velocity_to_velocity_coefficient_;
    double velocity_to_command_velocity_coefficient_;

    double status_current_to_torque_coefficient_;
    double torque_to_command_current_coefficient_;

    // 状态
    std::int64_t multi_turn_encoder_count_ = 0;
    int last_raw_angle_ = 0;

    double angle_;
    double torque_;
    double velocity_;
    double max_torque_;
    double temperature_;

    std::uint32_t can_id_ = 0;

    hcs_executor::Component::OutputInterface<double> angle_output_;
    hcs_executor::Component::OutputInterface<std::int64_t> raw_angle_output_;
    hcs_executor::Component::OutputInterface<double> velocity_output_;
    hcs_executor::Component::OutputInterface<double> torque_output_;
    hcs_executor::Component::OutputInterface<double> temperature_output_;
    hcs_executor::Component::OutputInterface<double> max_torque_output_;

    hcs_executor::Component::InputInterface<double> control_torque_;
    hcs_executor::Component::InputInterface<double> control_velocity_;
    hcs_executor::Component::InputInterface<double> control_angle_;
    hcs_executor::Component::InputInterface<double> control_angle_shift_;

};

} // namespace hcs_core::hardware::device
