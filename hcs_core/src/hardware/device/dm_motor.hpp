#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include <hcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/util/required.hpp"

namespace hcs_core::hardware::device {

/// DM（达妙）电机，MIT 控制帧。只写协议：除 accepts() 之外全部成员都在控制线程上，
/// 整帧由端口包装层（board::Can<DmMotor>）交进来；掉线计数和健康输出也在包装层。
class DmMotor {
public:
    /// 这个驱动要的总线：经典 CAN 2.0，8 字节帧，电机出厂的 1 Mbps。
    /// 总线声明成别的，电机往上接的时候就会被拒绝。
    static constexpr std::uint32_t kCanBitrate = 1'000'000;
    static constexpr bool kCanFd = false;

    /// **这一台**电机的寄存器 PMAX (0x15) / VMAX (0x16) / TMAX (0x17) 里存的 MIT 映射范围。
    /// 指令帧和反馈帧里的位置、速度、力矩字段都是按这三个量线性映射到 [-max, +max] 上的整数，
    /// 所以它们必须和电机里实际存的一致：不一致的话所有量都会被按比例缩放，而且不报任何错。
    ///
    /// 它们是可写的寄存器（DMTool 能改），所以属于单台电机而不属于型号：同一台车上的两个 J4310
    /// 可以不一样。因此每台电机各写一份、必填，没有型号默认值可退。上车调试时把它们读回来
    /// （0x7FF，读命令 0x33，RID 0x15 / 0x16 / 0x17），证明这里抄的是对的。
    struct MitRange {
        double position_max; // rad
        double velocity_max; // rad/s
        double torque_max;   // N*m
    };

    /// 出厂状态的 J4310。只给寄存器从没被改过的电机用，而且要显式写出名字才用。
    static constexpr MitRange kJ4310Factory{12.5, 30.0, 10.0};

    /// 这台电机跑哪一种 MIT 形态。在配置里选定，绝不从接线推断：驱动只注册所选模式用到的输入，
    /// 所以上游某个恰好同名的信号（串级控制器为自己的速度环写的 /control_velocity）
    /// 不可能再把电机里面的内环打开。
    enum class ControlMode : std::uint8_t {
        /// 只用 /control_torque；kp = kd = 0。所有的环都由上位机闭。
        kTorque,
        /// /control_velocity，/control_torque 作前馈；kp = 0，kd 取自 /control_kd 或配置。
        kVelocity,
        /// /control_angle，/control_velocity 和 /control_torque 作前馈；kp 和 kd 取自
        /// /control_kp、/control_kd 或配置。
        kPosition,
    };

    /// 电机状态，在反馈帧 D[0] 的高 4 位里。
    enum class Error : std::uint8_t {
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

    /// 聚合类型：接线表里写了什么一目了然。到 mit_range 为止的四个字段必填（util::Required）：
    /// 漏写一个就编不过，而不是悄悄替你挑一个模式或映射范围。
    struct Config {
        Config& set_gain(double kp_value, double kd_value) {
            return kp = kp_value, kd = kd_value, *this;
        }
        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_zero_angle(double value) { return zero_angle = value, *this; }
        Config& set_reduction_ratio(double value) { return reduction_ratio = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
        Config& set_offline_timeout(int value) { return offline_timeout = value, *this; }
        Config& set_error_retry_interval(int value) { return error_retry_interval = value, *this; }

        /// 电机构造时就定死：它决定有哪些输入。
        util::Required<ControlMode> control_mode;

        /// 驱动的接收 id（寄存器 ESC_ID 0x08）。MIT 控制帧就发到这个 id，不加偏移。
        /// 另外三种模式要加 0x100 / 0x200 / 0x300，这里没有实现。
        util::Required<std::uint32_t> esc_id;
        /// 驱动的反馈 id（寄存器 MST_ID 0x07）。每台电机必须各不相同，也不能和同一路总线上
        /// DJI 的反馈段（0x201~0x20B）撞。撞了不会报错，只会没有声音。
        util::Required<std::uint32_t> master_id;

        /// 这台电机的 PMAX / VMAX / TMAX。见 MitRange。
        util::Required<MitRange> mit_range;

        /// MIT 增益的默认值，/control_kp 和 /control_kd 没有人提供时用它。
        /// 协议允许的范围是 kp ∈ [0, 500]、kd ∈ [0, 5]。
        double kp = 0.0;
        double kd = 0.0;

        /// 当作零角度的反馈原始值。默认值是相信驱动自己存的零点，
        /// 也就是"保存零点"那一帧的结果。
        int encoder_zero_point = kRawAngleZero;

        /// 用角度给零点，取驱动自己的位置坐标（rad，即反馈帧解出来的值：还没算 reversed 和
        /// reduction_ratio）。优先于 encoder_zero_point。位置的原始值是按 position_max 缩放的，
        /// 所以同一个物理零点在不同的 PMAX 下是不同的原始计数；用角度给，position_max 改成和重新
        /// 刷过的寄存器一致之后它仍然有效。
        std::optional<double> zero_angle = std::nullopt;

        /// 只指外加的减速箱。J4310 里面那一级 10:1 驱动已经算进去了，它的反馈是折算到输出轴的。
        /// 注意 kp / kd 是驱动侧的增益，**不**随这个量缩放。
        double reduction_ratio = 1.0;

        bool reversed = false;
        /// 关：角度就是驱动报的值，范围 ±position_max（大约 ±4 圈）。
        /// 开：在 ±position_max 处的回绕被累加起来，角度无界增长。
        bool multi_turn_angle_enabled = false;

        /// 连续多少拍没有新反馈算掉线。
        int offline_timeout = 100;
        /// 两次"清错"帧之间隔多少拍。各种保护（过温、过压、过流）都需要时间恢复，
        /// 按控制频率去重试只是在和保护逻辑对着干。
        int error_retry_interval = 500;
    };

    DmMotor(
        hcs_executor::Component& status_component, hcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : control_mode_(config.control_mode) {
        status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
        status_component.register_output(name_prefix + "/raw_angle", raw_angle_output_, 0);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_output_, 0.0);
        status_component.register_output(name_prefix + "/temperature", temperature_output_, 0.0);
        status_component.register_output(
            name_prefix + "/temperature_mos", temperature_mos_output_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);
        status_component.register_output(
            name_prefix + "/error_code", error_code_output_, std::uint8_t{0});

        // 只注册所选模式用到的。全部是可选输入：没接线的输入读出来是 NaN，绝不是 0。
        command_component.register_input( //
            name_prefix + "/control_torque", control_torque_, false);
        if (control_mode_ != ControlMode::kTorque) {
            command_component.register_input( //
                name_prefix + "/control_velocity", control_velocity_, false);
            command_component.register_input( //
                name_prefix + "/control_kd", control_kd_, false);
        }
        if (control_mode_ == ControlMode::kPosition) {
            command_component.register_input( //
                name_prefix + "/control_angle", control_angle_, false);
            command_component.register_input( //
                name_prefix + "/control_kp", control_kp_, false);
        }

        configure(config);
    }

    DmMotor(const DmMotor&) = delete;
    DmMotor& operator=(const DmMotor&) = delete;
    DmMotor(DmMotor&&) = delete;
    DmMotor& operator=(DmMotor&&) = delete;

    ~DmMotor() = default;

    auto id() const noexcept -> std::uint32_t { return esc_id_; }
    [[nodiscard]] std::uint32_t feedback_id() const noexcept { return master_id_; }
    [[nodiscard]] std::uint32_t command_id() const noexcept { return esc_id_; }

    /// 落在我们反馈 id 上的这一帧是不是真的属于这台电机。D[0] 是 ID | ERR << 4：手册说 ID 是
    /// "CAN_ID 的低 8 位"，但 ERR 占了同一个字节的高 4 位，所以 id 只剩下低 4 位；就比这么多，
    /// 不多比。电机在这里报的是 ESC_ID，不是 MST_ID。
    ///
    /// 对不上说明有两个驱动的 MST_ID 寄存器相同：它们的反馈在这个 id 上交错出现，不检查的话
    /// 角度会在两台电机之间来回跳，而且不报任何错。被拒的帧不解码，也不算反馈，见 take_problem()。
    ///
    /// @note 唯一运行在传输线程上的成员：它只许读 esc_id_（configure() 定死之后不再变），
    /// 别的都不许读。
    [[nodiscard]] bool accepts(CanPacket8 frame) const noexcept {
        return (std::to_integer<std::uint32_t>(frame.as_bytes()[0]) & 0x0F) == (esc_id_ & 0x0F);
    }

    /// 上线日志里的那一行：id，以及这个驱动假定的 MIT 范围，对照 J4310 的出厂值。
    /// 范围不同说明寄存器被改写过，这时接线表里抄的那份必须读回来核对
    /// （0x7FF，读命令 0x33，RID 0x15 / 0x16 / 0x17）：抄错了会悄悄地缩放每一条指令和反馈。
    std::string describe() const {
        constexpr auto factory = kJ4310Factory;
        const auto differs = [](double a, double b) { return std::abs(a - b) > 1e-9; };
        const bool rewritten = differs(mit_range_.position_max, factory.position_max)
                            || differs(mit_range_.velocity_max, factory.velocity_max)
                            || differs(mit_range_.torque_max, factory.torque_max);
        return std::format(
            "DM esc_id=0x{:x} master_id=0x{:x} PMAX={:.6g} VMAX={:.6g} TMAX={:.6g} "
            "(J4310 factory {:.6g}/{:.6g}/{:.6g}){}",
            esc_id_, master_id_, mit_range_.position_max, mit_range_.velocity_max,
            mit_range_.torque_max, factory.position_max, factory.velocity_max, factory.torque_max,
            rewritten ? " -- registers rewritten: read them back (0x7FF, 0x33, RID 0x15-0x17)"
                      : " (factory values)");
    }

    /// 尽力域（1 Hz，绝不在拍内）：自上次调用以来出了什么问题，没有则为空。
    /// accepts() 拒掉的帧数每涨一次报一次。
    /// @param count accepts() 至今拒掉的帧数，由端口包装层数
    std::optional<std::string> take_problem(std::uint32_t count) {
        if (count == reported_foreign_frames_)
            return std::nullopt;
        reported_foreign_frames_ = count;
        return std::format(
            "{} frames on feedback id 0x{:x} carried another motor's id; two drivers share one "
            "MST_ID, fix the register",
            count, master_id_);
    }

    /// 每拍一次，不管有没有新帧：清错的退避是按调用次数数的。
    void on_tick(bool /*online*/) {
        if (error_retry_countdown_ > 0)
            --error_retry_countdown_;
    }

    /// 一帧新的反馈。全零字节的 DM 帧会解成 (-PMAX, -VMAX, -TMAX)，也就是负向位置极限处的
    /// 满额负力矩，所以真正的帧到来之前什么都不解：在那之前每个输出都保持初值。
    void on_frame(CanPacket8 frame) {
        const struct [[gnu::packed]] {
            std::uint8_t id_and_error;
            std::uint8_t angle_high;
            std::uint8_t angle_low;
            std::uint8_t velocity_high;
            std::uint8_t velocity_low_and_torque_high;
            std::uint8_t torque_low;
            std::uint8_t temperature_mos;
            std::uint8_t temperature_rotor;
        } feedback alignas(CanPacket8) = std::bit_cast<decltype(feedback)>(frame);

        error_ = static_cast<Error>(feedback.id_and_error >> 4);

        // 角度，单位 rad。位置 16 位，速度和力矩各 12 位，都线性映射到 [-max, +max] 上，
        // 所以原始值 0 是负向极限而不是原点。
        const int raw_angle = (int{feedback.angle_high} << 8) | feedback.angle_low;
        auto calibrated_raw_angle = raw_angle - encoder_zero_point_;
        if (!multi_turn_angle_enabled_) {
            // 归一化到 (-模/2, 模/2]。DM 的位置在 ±PMAX 处回绕，那是大约四圈，
            // 所以没有"一圈"的范围可以折进去。
            calibrated_raw_angle =
                ((calibrated_raw_angle + kRawAngleModulus / 2) & (kRawAngleModulus - 1))
                - kRawAngleModulus / 2;
            multi_turn_encoder_count_ = calibrated_raw_angle;
        } else {
            // 和 LkMotor 一样的最小差值技巧，因为模是 2 的幂所以成立。在原始值域里做这件事，
            // 回绕才值 2*PMAX 而不是一圈。
            auto diff =
                (calibrated_raw_angle - multi_turn_encoder_count_) & (kRawAngleModulus - 1);
            if (diff > (kRawAngleModulus >> 1))
                diff -= kRawAngleModulus;
            multi_turn_encoder_count_ += diff;
        }
        last_raw_angle_ = raw_angle;
        angle_ =
            raw_angle_to_angle_coefficient_ * static_cast<double>(multi_turn_encoder_count_);

        // 速度，单位 rad/s
        const int raw_velocity = (int{feedback.velocity_high} << 4)
                               | (feedback.velocity_low_and_torque_high >> 4);
        velocity_ = raw_velocity_to_velocity_coefficient_
                  * (static_cast<double>(raw_velocity) - kRawVelocityZero);

        // 力矩，单位 N*m
        const int raw_torque = (int{feedback.velocity_low_and_torque_high & 0x0F} << 8)
                             | feedback.torque_low;
        torque_ = raw_torque_to_torque_coefficient_
                * (static_cast<double>(raw_torque) - kRawTorqueZero);

        // 温度，单位摄氏度。过温保护实际盯的是转子线圈这一个。
        temperature_     = static_cast<double>(feedback.temperature_rotor);
        temperature_mos_ = static_cast<double>(feedback.temperature_mos);

        *angle_output_           = angle();
        *raw_angle_output_       = last_raw_angle();
        *velocity_output_        = velocity();
        *torque_output_          = torque();
        *temperature_output_     = temperature();
        *temperature_mos_output_ = temperature_mos();
        *error_code_output_      = static_cast<std::uint8_t>(error());
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
    /// 电机报了故障，区别于两个普通的使能状态。驱动自己会一直去清它（见 generate_command()）；
    /// 清掉之后许不许把力矩重新加回关节上，是调用方的决定，所以才把它暴露出来。
    bool faulted() const { return error_ != Error::kDisabled && error_ != Error::kEnabled; }

    /// @brief 让电机离开上电默认状态。在这一帧被确认之前驱动不理任何控制帧，
    /// 所以它是每台 DM 电机要做的第一件事。
    constexpr static CanPacket8 generate_enable_command() { return generate_control_frame(0xFC); }

    /// @brief 让电机回到失能状态。没有指令可发的时候，往总线上放这一帧也是最安全的：
    /// 反馈是问一句答一句的，总线一旦安静下来，状态也就不再上报了。
    constexpr static CanPacket8 generate_disable_command() { return generate_control_frame(0xFD); }

    /// @brief 清除锁存的故障（过温之类）。
    constexpr static CanPacket8 generate_clear_error_command() {
        return generate_control_frame(0xFB);
    }

    /// @brief 把输出轴当前的位置设为驱动的零点，位置给定也随之归零。
    /// @note 这是一帧发给电机 id 的控制帧，不是 0x7FF 的寄存器写。
    constexpr static CanPacket8 generate_save_zero_command() {
        return generate_control_frame(0xFE);
    }

    /// @brief 上位机用这一帧按 MIT 控制律驱动电机：
    /// tau = kp * (angle - angle_measured) + kd * (velocity - velocity_measured) + torque。
    /// @note 每个参数在量化之前先限到映射范围里。不限的话，超范围的给定会在定点字段里绕回去，
    /// 出来的是相反方向的极值——对 16 位的位置字段来说就是一条满行程的指令。
    /// @note NaN 表示"没有给定"，对应的增益随之置零，所以没接线的位置输入不可能被当成
    /// "转到零点"的指令。
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

        // 五个参数按位挤在八个字节里，所以用字节数组比用位域结构体说得更清楚。
        const std::array<std::uint8_t, 8> command{
            static_cast<std::uint8_t>(raw_angle >> 8),
            static_cast<std::uint8_t>(raw_angle),
            static_cast<std::uint8_t>(raw_velocity >> 4),
            static_cast<std::uint8_t>(((raw_velocity & 0x0F) << 4) | (raw_kp >> 8)),
            static_cast<std::uint8_t>(raw_kp),
            static_cast<std::uint8_t>(raw_kd >> 4),
            static_cast<std::uint8_t>(((raw_kd & 0x0F) << 4) | (raw_torque >> 8)),
            static_cast<std::uint8_t>(raw_torque)};

        return std::bit_cast<CanPacket8>(command);
    }

    /// @brief 所配置模式的 MIT 帧，取自接了线的输入。
    CanPacket8 generate_mit_command() const {
        switch (control_mode_) {
        case ControlMode::kTorque:
            return generate_mit_command(kNan, kNan, control_torque(), kNan, kNan);
        case ControlMode::kVelocity:
            return generate_mit_command(
                kNan, control_velocity(), control_torque(), kNan, control_kd());
        case ControlMode::kPosition:
            return generate_mit_command(
                control_angle(), control_velocity(), control_torque(), control_kp(),
                control_kd());
        }
        return generate_disable_command();
    }

    /// @brief 这一拍要放到总线上的帧：使能、清错，或者所配置模式的 MIT 帧。
    /// @note 使能帧和清错帧是提前返回的，在 MIT 帧构造之前。先构造控制帧、再把它覆盖掉的状态机，
    /// 会让调用方看不出这一拍其实没有指令发出去。
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

    /// @brief 这台电机这一拍发什么，安全规则已经套上：安全拍，或者所选模式自己的给定是 NaN
    /// （控制器被禁用、倒地、被隔离），就让电机进失能状态（0xFD），而不是给它指令。
    CanPacket8 command_frame(bool safe) {
        if (safe || std::isnan(setpoint()))
            return generate_disable_command();
        return generate_command();
    }

    /// @brief 把这一拍的帧交给所在的总线。一台电机一帧。
    template <class BusFrames>
    void append_command(BusFrames& bus, bool safe) {
        bus.push(command_id(), command_frame(safe));
    }

    ControlMode control_mode() const noexcept { return control_mode_; }

    double control_angle() const {
        // 用 has_provider() 而不是 ready()：上游没人的可选输入绑的是一个默认构造的 0.0，
        // 读起来是 ready 的，那样"没接线"就变成了"命令转到零点"。
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
    /// 应用配置。只由构造函数调一次：驱动接到端口上之后不再重新配置。
    void configure(const Config& config) {
        esc_id_    = config.esc_id;
        master_id_ = config.master_id;
        mit_range_ = config.mit_range;

        // DM 解码 angle = raw * 2PMAX / 65535 - PMAX 的逆运算，
        // 所以驱动自己坐标里的 zero_angle 在这里正好落在原始零点上。
        const double position_max = config.mit_range->position_max;
        const int encoder_zero_point =
            config.zero_angle ? static_cast<int>(std::lround(
                                    (*config.zero_angle + position_max) * kRawAngleMax
                                    / (2 * position_max)))
                              : config.encoder_zero_point;
        encoder_zero_point_ = encoder_zero_point & (kRawAngleModulus - 1);

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        multi_turn_encoder_count_ = 0;
        last_raw_angle_           = encoder_zero_point_;

        const double sign            = config.reversed ? -1 : 1;
        const double reduction_ratio = config.reduction_ratio;

        raw_angle_to_angle_coefficient_ =
            sign / reduction_ratio * (2 * config.mit_range->position_max) / kRawAngleMax;
        angle_to_raw_angle_coefficient_ = 1 / raw_angle_to_angle_coefficient_;

        raw_velocity_to_velocity_coefficient_ =
            sign / reduction_ratio * (2 * config.mit_range->velocity_max) / kRawVelocityMax;
        velocity_to_raw_velocity_coefficient_ = 1 / raw_velocity_to_velocity_coefficient_;

        raw_torque_to_torque_coefficient_ =
            sign * reduction_ratio * (2 * config.mit_range->torque_max) / kRawTorqueMax;
        torque_to_raw_torque_coefficient_ = 1 / raw_torque_to_torque_coefficient_;

        // 注意：和 LkMotor 不同，这里的 max_torque_ **不是**手册上的峰值力矩（J4310 在 0.8 过流系数下
        // 11 N*m，0.98 下 12.5 N*m，额定 3.5 N*m）。它是 MIT 帧实际能表达的最大力矩，也就是 TMAX。
        // 控制器按手册上的数字去限幅，只会被协议再截一次。
        max_torque_ = config.mit_range->torque_max * reduction_ratio;

        kp_ = config.kp;
        kd_ = config.kd;

        error_retry_interval_ = config.error_retry_interval;

        error_                 = Error::kDisabled;
        error_retry_countdown_ = 0;

        angle_           = 0.0;
        velocity_        = 0.0;
        torque_          = 0.0;
        temperature_     = 0.0;
        temperature_mos_ = 0.0;

        *max_torque_output_ = max_torque();
    }

    /// 所选模式围绕的那个输入：它是 NaN 就表示"不要给这台电机指令"。
    double setpoint() const {
        switch (control_mode_) {
        case ControlMode::kTorque: return control_torque();
        case ControlMode::kVelocity: return control_velocity();
        case ControlMode::kPosition: return control_angle();
        }
        return kNan;
    }

    /// 使能、失能、清错、保存零点都是这个形状。
    constexpr static CanPacket8 generate_control_frame(std::uint8_t id) {
        const struct [[gnu::packed]] {
            std::uint8_t placeholder[7];
            std::uint8_t id;
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

    /// 在转换之前先挡住 NaN：std::clamp 会把 NaN 原样放过去，而把它转成 int 是未定义行为。
    /// 调用方滤 NaN 是为了语义，这里滤是为了安全。
    static int clamp_raw(double raw, int raw_max) {
        if (std::isnan(raw)) [[unlikely]]
            return 0;
        return static_cast<int>(std::round(std::clamp(raw, 0.0, static_cast<double>(raw_max))));
    }

    // 限值
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    // DM 把位置映射到 16 位上，速度、力矩、kp、kd 各映射到 12 位上。
    // "模"是计数器回绕的地方，"最大值"是线性映射跨过的范围。
    static constexpr int kRawAngleModulus = 1 << 16;
    static constexpr int kRawAngleMax     = kRawAngleModulus - 1;
    static constexpr int kRawAngleZero    = kRawAngleModulus / 2;
    static constexpr int kRawVelocityMax  = (1 << 12) - 1;
    static constexpr int kRawTorqueMax    = (1 << 12) - 1;
    static constexpr int kRawGainMax      = (1 << 12) - 1;

    // 常量
    // kp 和 kd 的范围是协议定死的，不由电机决定，所以不可配置。
    static constexpr double kKpMax = 500.0;
    static constexpr double kKdMax = 5.0;

    static constexpr double kRawVelocityZero = kRawVelocityMax / 2.0;
    static constexpr double kRawTorqueZero   = kRawTorqueMax / 2.0;

    ControlMode control_mode_;
    std::uint32_t esc_id_ = 0, master_id_ = 0;
    MitRange mit_range_{};
    std::uint32_t reported_foreign_frames_ = 0; /// 只在尽力域碰，见 take_problem()

    bool multi_turn_angle_enabled_;
    int encoder_zero_point_;

    double kp_ = 0.0, kd_ = 0.0;

    // 系数
    double raw_angle_to_angle_coefficient_, angle_to_raw_angle_coefficient_;
    double raw_velocity_to_velocity_coefficient_, velocity_to_raw_velocity_coefficient_;
    double raw_torque_to_torque_coefficient_, torque_to_raw_torque_coefficient_;

    // 状态
    Error error_ = Error::kDisabled;
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
    hcs_executor::Component::OutputInterface<std::int64_t> raw_angle_output_;
    hcs_executor::Component::OutputInterface<double> velocity_output_;
    hcs_executor::Component::OutputInterface<double> torque_output_;
    hcs_executor::Component::OutputInterface<double> temperature_output_;
    hcs_executor::Component::OutputInterface<double> temperature_mos_output_;
    hcs_executor::Component::OutputInterface<double> max_torque_output_;
    hcs_executor::Component::OutputInterface<std::uint8_t> error_code_output_;

    hcs_executor::Component::InputInterface<double> control_angle_;
    hcs_executor::Component::InputInterface<double> control_velocity_;
    hcs_executor::Component::InputInterface<double> control_torque_;
    hcs_executor::Component::InputInterface<double> control_kp_;
    hcs_executor::Component::InputInterface<double> control_kd_;
};

} // namespace hcs_core::hardware::device
