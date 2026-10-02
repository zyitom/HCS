#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <algorithm>
#include <array>
#include <bit>
#include <numbers>
#include <span>
#include <string>

#include <eigen3/Eigen/Dense>

#include <hcs_executor/component.hpp>
#include <hcs_base/protocol/endian_promise.hpp>
#include <hcs_base/protocol/package_receive.hpp>

#include "hardware/device/imu_outputs.hpp"
#include "hardware/device/serial_line.hpp"
#include "hardware/util/required.hpp"

namespace hcs_core::hardware::device {

/// HiPNUC（超核）CH0x0 系列 IMU，HI91 二进制帧。
///
/// 帧格式，多字节字段一律小端：
///
///     5A A5 | 长度:u16 | crc:u16 | 载荷[长度]
///
/// HI91 是出厂默认输出，也是唯一定长的帧：长度恒为 76，一帧恒为 82 字节。正因为定长，
/// receive_package 才能把它读进一个定长缓冲、对不上时逐字节重新对齐。校验是 CRC16/CCITT
/// （多项式 0x1021，初值 0），先算帧头的 4 个字节、再算载荷——crc 字段本身是跳过的，不是填零。
///
/// 依据：HiPNUC IMU 指令与编程手册 v1.7.2，2.8 / 7.31 / 7.35 节。
///
/// 输出的坐标约定：向量（角速度、加速度、磁场）和四元数的机体侧一律是 FLU（x 前 / y 左 / z 上，
/// REP-103），与模块 flash 里怎么配的无关。出厂配置 CONFIG IMU COORD 0（ENU）的模块报的机体系是
/// RFU（x 右 / y 前 / z 上），这里把它换成 FLU，只换这一次，消费者不需要知道模块怎么配的。
/// 模块用的是哪种约定由 Config::module_frame 告诉驱动——驱动自己看不出来：HI91 里没有坐标系
/// 字段，填错了只会悄悄地把轴换掉。
///
/// 只写协议：全部成员都在控制线程上，串口字节由端口包装层（board::Serial<Hipnuc>）按到达顺序
/// 喂进来；掉线计数和健康输出也在包装层。
class Hipnuc {
public:
    /// MAIN_STATUS 字里的位。没列出来的位是保留位：固件自己在用，不保证是 0，所以只能按掩码
    /// 测试，不要拿整个字去比较。
    enum class Status : std::uint16_t {
        kGyroBiasNotConverged = 1 << 3,   // WB_CONV，让设备静止 3~5 秒
        kMagneticDisturbance = 1 << 4,    // MAG_DIST，仅九轴；航向可能在靠惯性保持
        kAccelerometerSaturated = 1 << 5, // ACC_SAT，现在或最近 2 秒内
        kGyroscopeSaturated = 1 << 6,     // GYR_SAT，现在或最近 2 秒内
        kAttitudeNotConverged = 1 << 7,   // ATT_CONV
        kStatic = 1 << 9,                 // STATIC，模块认为自己静止
        kMagneticAiding = 1 << 10,        // MAG_AIDING，没置位就是六轴模式
        kUtcNotSynchronized = 1 << 11,    // UTC_UNSYNC，置位表示**没有**同步
        kSyncOutPulse = 1 << 12,          // SOUT_PULSE，这一帧对应一个 SYNC_OUT 脉冲
    };

    struct Config {
        /// 模块 flash 里存的是哪种约定，也就是 CONFIG IMU COORD 上次保存成了什么。
        /// 它只决定输入怎么换轴：输出两种情况下都是 FLU。
        /// - kEnu（默认，即出厂状态）：机体系 RFU，这里换成 FLU。
        /// - kNwu：机体系本来就是 FLU，原样通过。
        enum class ModuleFrame : std::uint8_t { kEnu, kNwu };

        Config& set_offline_timeout(int value) { return offline_timeout = value, *this; }
        Config& set_module_frame(ModuleFrame value) { return module_frame = value, *this; }

        /// 模块 flash 里设的串口速率（CONFIG SERIAL BAUD）。必填：它是这一个模块的属性，
        /// 不是这个型号的属性。
        util::Required<std::uint32_t> baudrate;

        /// 连续多少拍没有新帧算掉线。1 kHz 的控制回路配默认 100 Hz 的 HI91 输出，
        /// 每 10 拍应该来一帧。
        int offline_timeout = 100;

        ModuleFrame module_frame = ModuleFrame::kEnu;
    };

    Hipnuc(
        hcs_executor::Component& status_component, const std::string& name_prefix,
        const Config& config)
        : outputs_(status_component, name_prefix) {
        configure(config);
    }

    Hipnuc(const Hipnuc&) = delete;
    Hipnuc& operator=(const Hipnuc&) = delete;
    Hipnuc(Hipnuc&&) = delete;
    Hipnuc& operator=(Hipnuc&&) = delete;

    ~Hipnuc() = default;

    /// 这个模块要的串口设置：flash 里的速率，8N1（HI91 的帧格式）。
    [[nodiscard]] SerialLine serial_line() const noexcept { return {.baudrate = baudrate_}; }

    /// 喂一段收到的串口字节，按到达顺序。帧边界不要求和段的边界对齐：半帧留在缓存里，
    /// 等剩下的到了再解。
    /// @return 这一段里有没有解出至少一帧 HI91；一段里有好几帧时，输出是最新那帧
    ///
    /// 不是 HI91 的东西（HI83 帧、NMEA 行、波特率不对时的噪声）在重新对齐的过程中被丢掉。
    /// 这是故意不出声的——周期域里不许打日志——所以这种故障表现为 crc_error_count() 在涨、
    /// 以及设备掉线。
    bool on_bytes(std::span<const std::byte> uart_data) {
        auto stream = ByteSpanStream{uart_data};

        bool decoded = false;
        while (!stream.data.empty()) {
            const auto result =
                hcs_utility::receive_package<sizeof(Package::header), std::byte>(
                    stream, package_, cache_size_,
                    [](const Package& package) {
                        return package.header[0] == 0x5A && package.header[1] == 0xA5;
                    },
                    [this](const Package& package) { return verify(package); });
            if (result != hcs_utility::ReceiveResult::SUCCESS)
                continue;

            cache_size_ = 0;
            frame_count_ = frame_count_ + 1 == 0 ? 1 : frame_count_ + 1;
            decode(package_.payload);
            decoded = true;
        }

        if (decoded)
            publish();
        return decoded;
    }

    // FLU 机体系相对模块世界系的姿态。世界系的原点取决于模块配置（ENU 航向零点朝东，NWU 朝北），
    // 只对要绝对航向的消费者有意义，而这台车上没有这样的消费者。首帧之前是单位四元数。
    const Eigen::Quaterniond& quaternion() const { return quaternion_; }

    // 欧拉角，单位 rad，依次是 roll pitch yaw，物理意义在两种模块配置下相同。不同的是：
    // 航向零点（东 / 北，同 quaternion()），以及由这三个角重建旋转时用的转序（ENU 下 312，
    // NWU 下 321，手册 2.2 节）——不知道配置就不要拿它们去重建旋转。
    // yaw 是原始航向，±pi，逆时针为正；九轴模式下参考的是磁北，不是真北。
    const Eigen::Vector3d& euler_angles() const { return euler_angles_; }

    // 加速度，单位 m/s^2，FLU 机体系。线上的单位是 G。
    const Eigen::Vector3d& acceleration() const { return acceleration_; }

    // 角速度，单位 rad/s，FLU 机体系。线上的单位是 deg/s——和 HI83 帧里的同名字段不一样，
    // 那个已经是 rad/s。
    const Eigen::Vector3d& angular_velocity() const { return angular_velocity_; }

    // 磁场，单位 uT，FLU 机体系。可信到什么程度看 kMagneticDisturbance。
    const Eigen::Vector3d& magnetic_field() const { return magnetic_field_; }

    // 气压，单位 Pa
    double air_pressure() const { return air_pressure_; }

    // 温度，单位摄氏度；是模块的平均温度，不是哪一个传感器芯片的温度
    double temperature() const { return temperature_; }

    // 设备时间戳，单位 ms。没有同步 UTC 时（kUtcNotSynchronized 置位）是一个本地单调计数，
    // 每 24 小时回绕；同步了则是当天 UTC 00:00:00 起的毫秒数。两种情况下都不带日期，
    // 也都不是板卡的时钟，不要把两者混在一起。
    std::uint32_t system_time() const { return system_time_; }

    std::uint16_t main_status() const { return main_status_; }

    bool status(Status bit) const { return (main_status_ & static_cast<std::uint16_t>(bit)) != 0; }

    /// 至今解出的帧数。它为 0 的时候所有量都读零、四元数是单位四元数。只作诊断用；
    /// 超过 2^32 回绕时跳过 0。
    std::uint32_t frame_count() const { return frame_count_; }

    /// 帧头、长度、标签都对上了但 CRC 没过的帧数。一直在涨说明是接线或波特率的问题，
    /// 不是协议的问题。
    std::uint32_t crc_error_count() const { return crc_error_count_; }

private:
    /// 应用配置。只由构造函数调一次：驱动接到端口上之后不再重新配置。
    void configure(const Config& config) {
        baudrate_ = config.baudrate;
        module_frame_ = config.module_frame;
        // 这之前到的字节不算数。
        cache_size_ = 0;
    }

    static constexpr std::uint8_t kHi91Tag = 0x91;
    static constexpr double kGravity = 9.80665;                   // m/s^2
    static constexpr double kHalfSqrt2 = std::numbers::sqrt2 / 2; // Rz(+90°) 四元数的 W / Z

    /// 手册 7.31 节的字段表。偏移依次是 0 tag、1 main_status、3 temperature、4 air_pressure、
    /// 8 system_time、12 acc、24 gyr、36 mag、48 euler、60 quat。每个成员都是字节序容器或单个
    /// 字节，对齐是 1，所以不需要 packed 属性；下面那句 sizeof 断言才是"布局里没有填充"的证明。
    struct Hi91Payload {
        std::uint8_t tag;
        hcs_utility::le_uint16_t main_status;
        std::int8_t temperature;                       // 摄氏度
        hcs_utility::le_float32_t air_pressure;        // Pa
        hcs_utility::le_uint32_t system_time;          // ms
        hcs_utility::le_float32_t acceleration[3];     // G，XYZ
        hcs_utility::le_float32_t angular_velocity[3]; // deg/s，XYZ
        hcs_utility::le_float32_t magnetic_field[3];   // uT，XYZ
        hcs_utility::le_float32_t euler_angles[3];     // deg，roll pitch yaw
        hcs_utility::le_float32_t quaternion[4];       // WXYZ
    };
    static_assert(sizeof(Hi91Payload) == 76);

    struct Package {
        std::uint8_t header[2];          // 5A A5
        hcs_utility::le_uint16_t length; // HI91 恒为 sizeof(Hi91Payload)
        hcs_utility::le_uint16_t crc;    // 除 crc 自己之外所有字段的校验
        Hi91Payload payload;
    };
    static_assert(sizeof(Package) == 82);

    /// 载荷从哪里开始，以及 crc 字段之前有多少字节。crc 覆盖的是前面那一段加上载荷，
    /// 跳过它自己。
    static constexpr std::size_t kPayloadOffset = sizeof(Package) - sizeof(Hi91Payload);
    static constexpr std::size_t kCrcCoveredHeaderBytes = kPayloadOffset - sizeof(std::uint16_t);

    /// 把一段字节适配成 receive_package 要的 read() 接口。
    struct ByteSpanStream {
        std::span<const std::byte> data;

        std::size_t read(std::byte* buffer, std::size_t size) {
            size = std::min(size, data.size());
            if (size == 0)
                return 0;

            std::memcpy(buffer, data.data(), size);
            data = data.subspan(size);
            return size;
        }
    };

    /// CRC16/CCITT，多项式 0x1021，初值 0，不反转、不做最终异或。累计值带进带出，这样一帧可以
    /// 分两段算——和手册里的 crc16_update() 一个做法。
    static constexpr std::uint16_t crc16(std::span<const std::byte> bytes, std::uint16_t crc = 0) {
        for (const std::byte byte : bytes) {
            crc ^= static_cast<std::uint16_t>(std::to_integer<unsigned>(byte) << 8);
            for (int bit = 0; bit < 8; ++bit)
                crc = static_cast<std::uint16_t>((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
        }
        return crc;
    }

    /// 手册 7.34 节的示例帧，编译期校验。crc16() 是这个解码器里唯一纯粹的字节运算，所以也是
    /// 唯一不需要设备、总线、测试框架就能对着手册钉死的一块。
    static consteval bool crc16_matches_manual_example() {
        constexpr auto frame = std::array<std::uint8_t, 82>{
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

        // 帧里带的 crc（按读小端字段的方式读出来），以及手册上印的那个值。
        const auto expected = static_cast<std::uint16_t>(
            std::to_integer<unsigned>(bytes[kCrcCoveredHeaderBytes])
            | std::to_integer<unsigned>(bytes[kCrcCoveredHeaderBytes + 1]) << 8);

        return crc == expected && expected == 0xBB14
            && bytes[kPayloadOffset] == std::byte{kHi91Tag} // 载荷确实是 HI91
            && bytes[2] == std::byte{sizeof(Hi91Payload)};  // 长度字段确实是 76
    }

    /// 每个前两个字节对上了帧头的候选帧过一次。在这里拒掉，receive_package 就滑一个字节再试——
    /// 不是 HI91 的帧就是这样被跳过去的。
    bool verify(const Package& package) {
        // 这个函数依赖的校验算法，在构建时就对着手册钉死了。
        static_assert(crc16_matches_manual_example());

        // static_cast：EndianContainer 自带的模板 operator!= 和经隐式转换的内置
        // != 在 clang 下二义（gcc 能选出来），显式转一侧，语义不变。
        if (static_cast<std::uint16_t>(package.length) != sizeof(Hi91Payload)
            || package.payload.tag != kHi91Tag)
            return false;

        // 用 bit_cast 而不是 reinterpret_cast：字节视图是对象的一份拷贝，校验因此避开了转换指针
        // 会带来的对齐和别名问题，也正因为这样 crc16() 才能在上面的编译期检查里运行。
        const auto bytes = std::bit_cast<std::array<std::byte, sizeof(Package)>>(package);
        const auto crc = crc16(
            std::span{bytes}.subspan(kPayloadOffset),
            crc16(std::span{bytes}.first(kCrcCoveredHeaderBytes)));

        if (crc != static_cast<std::uint16_t>(package.crc)) {
            ++crc_error_count_;
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

        // 欧拉角不换轴：roll / pitch / yaw 在两种模块配置下物理意义相同，变的只是它们背后的
        // 分解转序（见访问函数上的说明）。

        // 线上的顺序是 WXYZ。模块发的本来就是单位四元数，所以重新归一化只是清掉浮点误差；
        // 而全零的四元数——模块还在启动时确实会发——不许被归一化成 NaN。
        auto quaternion = Eigen::Quaterniond{
            static_cast<float>(payload.quaternion[0]), static_cast<float>(payload.quaternion[1]),
            static_cast<float>(payload.quaternion[2]),
            static_cast<float>(payload.quaternion[3])};
        if (quaternion.coeffs().squaredNorm() > 1e-6) {
            if (module_frame_ == Config::ModuleFrame::kEnu) {
                // ENU 配置的模块报的机体系是 RFU。FLU = RFU 绕 Z 转 90°（前 = 原来的 y，
                // 左 = 原来的 -x），所以 R_world<-flu = R_world<-rfu * Rz(+90°)，是右乘。
                // 向量的换轴是同一个旋转：x_flu = y_rfu，y_flu = -x_rfu，z_flu = z_rfu。
                quaternion = quaternion * Eigen::Quaterniond{kHalfSqrt2, 0.0, 0.0, kHalfSqrt2};
                acceleration_    = rfu_to_flu(acceleration_);
                angular_velocity_ = rfu_to_flu(angular_velocity_);
                magnetic_field_  = rfu_to_flu(magnetic_field_);
            }
            quaternion_ = quaternion.normalized();
        }
    }

    /// 把最后解出的那一帧写到输出上。
    ///
    /// 欧拉角的 roll / pitch / yaw 只是名字：航向零点随 Config::module_frame 变（ENU 朝东 /
    /// NWU 朝北），换 module_frame 等于换 yaw 的零偏，消费侧要重新标定。
    void publish() {
        outputs_.publish(
            {.quaternion = quaternion(),
             .angular_velocity = angular_velocity(),
             .acceleration = acceleration(),
             .euler_angles = euler_angles()});
    }

    static Eigen::Vector3d rfu_to_flu(const Eigen::Vector3d& v) {
        return {v.y(), -v.x(), v.z()};
    }

    static Eigen::Vector3d to_vector(const hcs_utility::le_float32_t (&values)[3]) {
        // 这几个转换不是摆设：EndianContainer 有一个模板转换运算符，花括号初始化会让它推导成
        // std::initializer_list 而不是 float。
        return Eigen::Vector3d{
            static_cast<float>(values[0]), static_cast<float>(values[1]),
            static_cast<float>(values[2])};
    }

    // 组帧：正在拼的那一帧，以及它已经到了多少。
    Package package_;
    std::size_t cache_size_ = 0;
    std::uint32_t frame_count_ = 0;
    std::uint32_t crc_error_count_ = 0;

    Config::ModuleFrame module_frame_ = Config::ModuleFrame::kEnu;

    Eigen::Quaterniond quaternion_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d euler_angles_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d magnetic_field_ = Eigen::Vector3d::Zero();
    double air_pressure_ = 0.0;
    double temperature_ = 0.0;
    std::uint32_t system_time_ = 0;
    std::uint16_t main_status_ = 0;

    /// <名字>/quaternion、/angular_velocity、/acceleration、/euler/*、/angular_velocity/*：
    /// 所有 IMU 共用的那一组输出，见 ImuOutputs。
    ImuOutputs outputs_;

    std::uint32_t baudrate_ = 0;
};

} // namespace hcs_core::hardware::device
