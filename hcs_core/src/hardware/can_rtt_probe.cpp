#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hcs_can_port.hpp>
#include <libhcs/board/hpm5321.hpp>
#include <libhcs/data/datas.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/timer.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_sync/snapshot.hpp>
#include <hcs_sync/tick.hpp>
#include <hcs_utility/doorbell.hpp>
#include <hcs_utility/doorbell_worker.hpp>
#include <hcs_utility/rt_attributes.hpp>
#include <hcs_utility/thread_config.hpp>

namespace hcs_demo::hardware {

// ============================================================================
// 底层测试探针：CAN2.0 电机指令一发一收 + 串口 IMU（HI91 协议），会不会被拖过
// 1 kHz 拍边界、落到 update() 的下一拍才被看见——这正是它要量的事。
//
// 硬件链路：一块 HPM5321 板卡（USB 接主机），CAN1（外壳丝印）挂电机（0x145，
// 指令固定 A1 00 00 00 00 00 00 00，标准帧），UART0 接外部串口 IMU
// （921600bps，超核 HI91 定长二进制帧，见桌面 imu_cum_cn_1.7.2.pdf 第 30 页）。
// 两路数据都经同一条 USB 链路、同一个 libhcs IO 线程异步到达——
// 用户担心的"会不会一起被移到下一拍"，问的就是这条共享链路的到达时序。
//
// 组件本身不摸这条事件线程：CAN1/UART0 的接收回调只管落地时间戳、
// 发布进 Snapshot；发指令走独立的 DoorbellWorker 线程，拍尾门铃叫醒，
// 全程不进控制回路（board_->start_transmit() 内部有锁、可能阻塞，
// 这正是 DoorbellWorker 存在的理由，见 doorbell_worker.hpp 的注释）。
//
// 判据（都在 update() 里对着 Snapshot::Reading 算）：
//   - fresh == false  ——  本拍读到的还是上一拍那份，说明这一整个周期
//     （1/update_rate）里压根没有新样本落地，是最硬的"被移到下一拍"证据。
//   - age > tick.dt   ——  样本到手时已经比一拍还旧，即便 fresh，也说明
//     它不是"这一拍该有的那份"。
// ============================================================================
class CanRttProbe
    : public hcs_executor::Component
    , public rclcpp::Node
    , public libhcs::board::Hpm5321::Callback {
public:
    CanRttProbe()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , sender_(
              hcs_utility::ThreadConfig{tx_thread_config_spec(), "can-rtt-probe-tx"},
              [this] { send_command(); }) {

        can_id_ = static_cast<std::uint32_t>(int_parameter("can_id", can_id_));
        // 未单独配置时默认跟 can_id 一致（一发一收同 ID 回帧），所以种子取更新后的 can_id_。
        response_can_id_ = static_cast<std::uint32_t>(int_parameter("response_can_id", can_id_));
        report_stride_ = static_cast<std::uint64_t>(
            std::max<std::int64_t>(1, int_parameter("report_stride", static_cast<std::int64_t>(report_stride_))));
        serial_filter_ = string_parameter("serial_filter", serial_filter_);
        skip_version_checks_ =
            bool_parameter("dangerously_skip_version_checks", skip_version_checks_);

        if (!sender_.thread_config_error().empty())
            RCLCPP_WARN(
                get_logger(), "[%s] tx thread config: %s", get_component_name().c_str(),
                sender_.thread_config_error().c_str());

        // 板卡在构造期就可能开始收发，早到的回调不许依赖"构造函数体后面还没跑完"的状态。
        // 所以放最后，且成员声明顺序上
        // board_ 在它会碰的那些数据（motor_snapshot_ 等）之后、在 sender_ 之前——
        // 析构逆序：sender_ 先停线程，然后 board_ 停传输，最后数据才消失。
        auto options = libhcs::board::AdvancedOptions{};
        options.dangerously_skip_version_checks = skip_version_checks_;
        // 波特率走构造期 Configuration：会话前 apply + 读回校验，重连自动重发（EP0）
        libhcs::board::hcs::Configuration config;
        config.uart_baudrate[0] = kImuBaudrate;
        board_ = std::make_unique<libhcs::board::Hpm5321>(*this, serial_filter_, options, config);

        report_timer_ = create_wall_timer(std::chrono::seconds{1}, [this] { report(); });

        RCLCPP_INFO(
            get_logger(),
            "[%s] CAN2.0+UART RTT probe on libhcs Hpm5321 (serial_filter='%s'): "
            "tx_id=0x%03X rx_id=0x%03X imu_baud=%u",
            get_component_name().c_str(), serial_filter_.c_str(), can_id_, response_can_id_,
            kImuBaudrate);
    }

    ~CanRttProbe() override = default;

    hcs_utility::Doorbell* tick_end_doorbell() override { return &sender_.doorbell(); }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        const auto motor = motor_snapshot_.read(tick.scheduled);
        const auto imu = imu_snapshot_.read(tick.scheduled);

        accumulate<MotorSample>(stats_.motor, motor, tick.dt);
        accumulate<ImuSample>(stats_.imu, imu, tick.dt);
        accumulate_skew(stats_, motor.fresh, imu.fresh, motor.age, imu.age);
        // 只在这一拍确实拿到新回帧时才计入一发一收耗时——fresh 保证这是这一拍的样本，
        // 不是上一拍就算过的那份；rtt_us < 0 是"这是收到的第一帧、配不到发送时刻"的哨兵。
        if (motor.fresh && motor.value.rtt_us >= 0.0) {
            stats_.rtt_sum_us += motor.value.rtt_us;
            stats_.rtt_max_us = std::max(stats_.rtt_max_us, motor.value.rtt_us);
            stats_.rtt_min_us =
                stats_.rtt_min_us < 0.0 ? motor.value.rtt_us : std::min(stats_.rtt_min_us, motor.value.rtt_us);
            ++stats_.rtt_samples;
        }
        ++stats_.ticks;

        if (tick.sequence % report_stride_ == 0) {
            stats_snapshot_.publish(stats_, tick.scheduled);
            // 按窗口统计，不是从开机累到现在：发完这一窗口就清零，下一窗口从零开始。
            // 这样日志里每一行反映的是"最近这一秒"，趋势是不是在恶化一眼就能看出来，
            // 不用再拿相邻两行的累计值手动做差。
            stats_ = Stats{};
        }

        // IMU 内部时钟 vs 主机时钟对表：这个不能按窗口清零，基线越长 ppm 估得越准
        // （system_time_ms 只有整毫秒分辨率，1 秒基线下几十 ppm 的漂移根本看不出来）。
        // age 反推 sampled_at：reading.age == reference − sampled_at，reference 传的
        // 是 tick.scheduled，所以 sampled_at = tick.scheduled − imu.age。
        if (imu.fresh) {
            const auto host_time = tick.scheduled - imu.age;
            if (!imu_clock_check_.started) {
                imu_clock_check_.started = true;
                imu_clock_check_.first_system_ms = imu.value.system_time_ms;
                imu_clock_check_.first_host_ns = to_ns(host_time);
            }
            imu_clock_check_.last_system_ms = imu.value.system_time_ms;
            imu_clock_check_.last_host_ns = to_ns(host_time);
            imu_clock_check_snapshot_.publish(imu_clock_check_, tick.scheduled);
        }
    }

private:
    struct MotorSample {
        std::array<std::byte, 8> payload{};
        // send_command() 实际把帧交出去、到这一帧回调落地之间的耗时——就是"一发一收"
        // 本身的往返时间，跟 age（这份数据相对本拍拍首有多旧）是两件事。< 0 表示
        // 收到这帧时还没有可配对的发送时刻（一般只发生在收到的第一帧）。
        double rtt_us = -1.0;
    };

    struct ImuSample {
        std::uint32_t system_time_ms = 0; ///< HI91 帧里的模块时间戳（墙钟 ms）
        float gyro[3] = {0, 0, 0};        ///< deg/s，机体系 XYZ
    };

    struct ChannelStats {
        std::uint64_t misses = 0; ///< fresh == false 的拍数：上一拍没有新样本落地
        std::uint64_t late = 0;   ///< age > dt 的拍数：到手时已经比一拍还旧
        hcs_sync::Duration max_age{};
        hcs_sync::Duration sum_age{};
        std::uint64_t samples = 0;
    };

    struct Stats {
        std::uint64_t ticks = 0;
        ChannelStats motor;
        ChannelStats imu;

        // ── 错相判据：motor/imu 两路独立 miss 会不会往同一拍上撞 ────────────
        // both_miss 高、motor_only/imu_only 低 ——两路共享的东西（USB 链路、
        // libhcs IO 线程）整体卡了一下，两路一起被拖过拍边界，符合"一起被
        // 移到下一拍"；only 类高、both 低——是各自独立的问题（丢帧/校验错/
        // 单个电机没答），不是链路层面的同步问题。
        std::uint64_t both_miss = 0;
        std::uint64_t motor_only_miss = 0;
        std::uint64_t imu_only_miss = 0;

        // ── 错相判据：两路都新鲜时，彼此的年龄差（motor.age − imu.age）────
        // 均值不为零 ——两路有固定的相对提前/滞后（比如板上处理顺序、UART
        // 波特率导致 82 字节传输耗时不同）；stddev 大——这个相对偏移本身在
        // 抖动，不是恒定相位差，融合时"同一拍两路数据其实来自不同时刻"的
        // 严重程度逐拍变化，更难补偿。
        double skew_sum_us = 0.0;
        double skew_sum_sq_us = 0.0;
        double skew_abs_max_us = 0.0;
        std::uint64_t skew_samples = 0;

        // ── 一发一收本身的往返时间：send_command() 交出帧到 can_receive
        // 落地之间的耗时。跟上面的 age/skew 不是一回事——那两个问的是"数据相对拍
        // 边界新不新鲜"，这个问的是"这一次请求-应答电气上到底花了多久"。
        double rtt_sum_us = 0.0;
        double rtt_max_us = 0.0;
        double rtt_min_us = -1.0; // < 0 表示这一窗口还没有样本
        std::uint64_t rtt_samples = 0;
    };

    // IMU 内部时钟 vs 主机时钟对表用：第一份、最新一份新鲜样本各自的
    // "IMU 自报时间戳" + "主机收到它的时刻"，从不清零——基线越长，
    // ppm 级的时钟频差才越量得准。
    struct ImuClockCheck {
        bool started = false;
        std::uint32_t first_system_ms = 0;
        std::int64_t first_host_ns = 0;
        std::uint32_t last_system_ms = 0;
        std::int64_t last_host_ns = 0;
    };

    static constexpr std::uint32_t kImuBaudrate = 921600;
    static constexpr std::uint8_t kHi91Tag = 0x91;
    static constexpr std::size_t kHi91PayloadLen = 76;
    static constexpr std::size_t kHi91FrameLen = 6 + kHi91PayloadLen;

    template <typename T>
    static void accumulate(
        ChannelStats& stats, const typename hcs_sync::Snapshot<T>::Reading& reading,
        hcs_sync::Duration dt) noexcept {
        if (!reading.fresh)
            ++stats.misses;
        if (reading.valid && reading.age > dt)
            ++stats.late;
        stats.max_age = std::max(stats.max_age, reading.age);
        stats.sum_age += reading.age;
        ++stats.samples;
    }

    static void accumulate_skew(
        Stats& stats, bool motor_fresh, bool imu_fresh, hcs_sync::Duration motor_age,
        hcs_sync::Duration imu_age) noexcept {
        if (!motor_fresh && !imu_fresh)
            ++stats.both_miss;
        else if (!motor_fresh)
            ++stats.motor_only_miss;
        else if (!imu_fresh)
            ++stats.imu_only_miss;

        if (motor_fresh && imu_fresh) {
            const double skew_us =
                std::chrono::duration<double, std::micro>(motor_age - imu_age).count();
            stats.skew_sum_us += skew_us;
            stats.skew_sum_sq_us += skew_us * skew_us;
            stats.skew_abs_max_us = std::max(stats.skew_abs_max_us, std::abs(skew_us));
            ++stats.skew_samples;
        }
    }

    std::string tx_thread_config_spec() { return string_parameter("tx_thread_config", ""); }

    std::string string_parameter(const std::string& name, const std::string& fallback) {
        std::string value = fallback;
        get_parameter_or(name, value, fallback);
        return value;
    }

    std::int64_t int_parameter(const std::string& name, std::int64_t fallback) {
        std::int64_t value = fallback;
        get_parameter_or(name, value, fallback);
        return value;
    }

    bool bool_parameter(const std::string& name, bool fallback) {
        bool value = fallback;
        get_parameter_or(name, value, fallback);
        return value;
    }

    // 拍尾门铃叫醒后，在这条独立线程上把固定指令发出去。board_->start_transmit()
    // 内部有锁、可能阻塞，所以绝不能从 update() 里直接调——这条线程就是为了
    // 替控制回路挨这一下。异常由 DoorbellWorker 自己吞掉、计数，见 sender_.exceptions()。
    void send_command() {
        static constexpr std::array<std::byte, 8> kPayload = {
            std::byte{0xA1}, std::byte{0}, std::byte{0}, std::byte{0},
            std::byte{0},    std::byte{0}, std::byte{0}, std::byte{0}};

        board_->start_transmit().can_transmit(
            libhcs::board::hcs::CanPort::kCan1,
            libhcs::data::CanDataView{.can_id = can_id_, .can_data = kPayload});
        // 记在 transmit() 返回之后，而不是之前——这样它标的是"帧已经交出去"的时刻，
        // 不含 transmit() 自己排队/拿锁的耗时，can_receive 里配对出来的
        // rtt_us 才对得上"一发一收"字面意思：发出去到答回来。
        last_tx_ns_.store(to_ns(hcs_sync::Clock::now()), std::memory_order_relaxed);
    }

    // 事件域：libhcs IO 线程回调，电机回帧到手就发布，不做任何等待。
    // libhcs 的接收回调带端口参数，端口按外壳丝印编号（旧 SDK 的 can0 就是这里的 CAN1）。
    void can_receive(
        libhcs::board::hcs::CanPort port, const libhcs::data::CanDataView& data) override {
        if (port != libhcs::board::hcs::CanPort::kCan1)
            return;
        if (data.is_extended_can_id || data.is_remote_transmission)
            return;
        if (data.can_id != response_can_id_ || data.can_data.size() != 8)
            return;

        const auto now = hcs_sync::Clock::now();

        MotorSample sample{};
        std::memcpy(sample.payload.data(), data.can_data.data(), sample.payload.size());

        // 用最近一次 send_command() 记的发出时刻配对。协议本身没有序号，配对只能靠
        // "最近一次"，一发一收模式下这就是对的；真出现丢帧/乱序会把 rtt 算宽，但不会
        // 算错方向——足够"大致测量"用，不是精确的逐帧往返时间表。
        const auto tx_ns = last_tx_ns_.load(std::memory_order_relaxed);
        sample.rtt_us = tx_ns != 0
                          ? std::chrono::duration<double, std::micro>(now - from_ns(tx_ns)).count()
                          : -1.0;

        motor_snapshot_.publish(sample, now);
    }

    // 事件域：同一条传输线程回调，喂 HI91 定长二进制帧的字节流解析器。
    void uart0_receive_callback(const libhcs::data::UartDataView& data) override {
        uart_rx_buffer_.insert(uart_rx_buffer_.end(), data.uart_data.begin(), data.uart_data.end());

        while (try_consume_one_hi91_frame()) {
        }

        // 防止同步一直找不到帧头时缓冲区无限增长（比如波特率配错、纯垃圾字节）。
        constexpr std::size_t kMaxBacklog = 4096;
        if (uart_rx_buffer_.size() > kMaxBacklog)
            uart_rx_buffer_.erase(uart_rx_buffer_.begin(), uart_rx_buffer_.end() - kHi91FrameLen);
    }

    // 返回 true 表示消费了一个字节（丢弃重新同步）或者一整帧，调用方应该接着再试一次；
    // 返回 false 表示缓冲区里剩下的字节不够判断，等下一批数据。
    bool try_consume_one_hi91_frame() {
        if (uart_rx_buffer_.size() < 6)
            return false;

        if (uart_rx_buffer_[0] != std::byte{0x5A} || uart_rx_buffer_[1] != std::byte{0xA5}) {
            uart_rx_buffer_.erase(uart_rx_buffer_.begin());
            return true;
        }

        const auto payload_len = read_u16le(uart_rx_buffer_.data() + 2);
        if (payload_len != kHi91PayloadLen) {
            // 不是 HI91（或者同步丢了），丢一个字节重新找帧头。
            uart_rx_buffer_.erase(uart_rx_buffer_.begin());
            return true;
        }
        if (uart_rx_buffer_.size() < kHi91FrameLen)
            return false; // 还没收全，等下一批字节

        std::uint16_t crc = 0;
        crc16_update(crc, uart_rx_buffer_.data(), 4);
        crc16_update(crc, uart_rx_buffer_.data() + 6, payload_len);
        if (crc != read_u16le(uart_rx_buffer_.data() + 4)) {
            imu_crc_errors_.fetch_add(1, std::memory_order_relaxed);
            uart_rx_buffer_.erase(uart_rx_buffer_.begin());
            return true;
        }

        const std::byte* payload = uart_rx_buffer_.data() + 6;
        if (static_cast<std::uint8_t>(payload[0]) == kHi91Tag) {
            ImuSample sample{};
            sample.system_time_ms = read_u32le(payload + 8);
            sample.gyro[0] = read_f32le(payload + 24);
            sample.gyro[1] = read_f32le(payload + 28);
            sample.gyro[2] = read_f32le(payload + 32);
            imu_snapshot_.publish(sample, hcs_sync::Clock::now());

            if (!imu_logged_once_) {
                imu_logged_once_ = true;
                RCLCPP_INFO(
                    get_logger(), "[%s] first HI91 frame parsed: t=%ums gyro=(% .2f % .2f % .2f)deg/s",
                    get_component_name().c_str(), sample.system_time_ms, sample.gyro[0],
                    sample.gyro[1], sample.gyro[2]);
            }
        }

        uart_rx_buffer_.erase(
            uart_rx_buffer_.begin(), uart_rx_buffer_.begin() + static_cast<std::ptrdiff_t>(kHi91FrameLen));
        return true;
    }

    static std::uint16_t read_u16le(const std::byte* p) noexcept {
        return static_cast<std::uint16_t>(
            static_cast<unsigned>(p[0]) | (static_cast<unsigned>(p[1]) << 8));
    }

    static std::uint32_t read_u32le(const std::byte* p) noexcept {
        return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8)
             | (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
    }

    static float read_f32le(const std::byte* p) noexcept {
        const std::uint32_t raw = read_u32le(p);
        float value;
        std::memcpy(&value, &raw, sizeof(value));
        return value;
    }

    // last_tx_ns_ 得在发送线程和 IO 线程之间传，std::atomic<Timestamp> 没必要
    // 折腾（time_point 不保证 lock-free），干脆存自纪元以来的纳秒数，两个方向各转一次。
    static std::int64_t to_ns(hcs_sync::Timestamp t) noexcept {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
    }

    static hcs_sync::Timestamp from_ns(std::int64_t ns) noexcept {
        return hcs_sync::Timestamp{std::chrono::nanoseconds{ns}};
    }

    // CRC-16/XMODEM：多项式 0x1021，初值 0x0000，无输入/输出反转，无最终异或。
    // 算法照抄手册「7.34 帧解析示例」附的参考实现，逐字节逐位，不查表。
    static void crc16_update(std::uint16_t& crc, const std::byte* src, std::size_t len) noexcept {
        std::uint32_t c = crc;
        for (std::size_t j = 0; j < len; ++j) {
            c ^= static_cast<std::uint32_t>(src[j]) << 8;
            for (int i = 0; i < 8; ++i) {
                std::uint32_t temp = c << 1;
                if ((c & 0x8000u) != 0)
                    temp ^= 0x1021u;
                c = temp;
            }
        }
        crc = static_cast<std::uint16_t>(c);
    }

    // 尽力域：ROS wall timer 回调，每秒打一次。stats_ 是按窗口清零的，所以下面
    // 全部数字都是"最近这一窗口"的即时值，不是从开机累到现在的滚动平均——
    // 趋势是不是在恶化，看相邻两行就行，不用再手动做差。
    void report() {
        const auto reading = stats_snapshot_.read(hcs_sync::Clock::now());
        if (!reading.valid)
            return;
        const auto& s = reading.value;
        ++report_count_;

        // sender_.exceptions() / imu_crc_errors_ 是别处维护的累计计数器，改不了，
        // 这里自己记上一次的值做差，换算成"这一窗口新增了多少"，跟其余字段口径一致。
        const auto tx_exceptions_total = sender_.exceptions();
        const auto tx_exceptions_delta = tx_exceptions_total - last_tx_exceptions_;
        last_tx_exceptions_ = tx_exceptions_total;

        const auto imu_crc_total = imu_crc_errors_.load(std::memory_order_relaxed);
        const auto imu_crc_delta = imu_crc_total - last_imu_crc_errors_;
        last_imu_crc_errors_ = imu_crc_total;

        const auto pct = [](std::uint64_t n, std::uint64_t d) {
            return d != 0 ? 100.0 * static_cast<double>(n) / static_cast<double>(d) : 0.0;
        };
        const auto mean_us = [](const ChannelStats& c) {
            return c.samples != 0
                     ? std::chrono::duration<double, std::micro>(c.sum_age).count()
                           / static_cast<double>(c.samples)
                     : 0.0;
        };
        const auto max_us = [](const ChannelStats& c) {
            return std::chrono::duration<double, std::micro>(c.max_age).count();
        };

        RCLCPP_INFO(
            get_logger(),
            "[%s] t=+%llus ticks=%llu | motor(0x%03X<-0x%03X) miss=%.2f%% late=%.2f%% "
            "age(mean/max)=%.0f/%.0fus tx_err=%llu | "
            "imu(HI91) miss=%.2f%% late=%.2f%% age(mean/max)=%.0f/%.0fus crc_err=%llu",
            get_component_name().c_str(), static_cast<unsigned long long>(report_count_),
            static_cast<unsigned long long>(s.ticks), can_id_, response_can_id_,
            pct(s.motor.misses, s.ticks), pct(s.motor.late, s.ticks), mean_us(s.motor),
            max_us(s.motor), static_cast<unsigned long long>(tx_exceptions_delta),
            pct(s.imu.misses, s.ticks), pct(s.imu.late, s.ticks), mean_us(s.imu), max_us(s.imu),
            static_cast<unsigned long long>(imu_crc_delta));

        // 错相：两路是不是共享的东西一起卡（both_miss），还是各自独立丢
        // （*_only_miss）；都新鲜时彼此的年龄差有没有固定偏移、抖不抖。
        const double skew_mean_us = s.skew_samples != 0 ? s.skew_sum_us / static_cast<double>(s.skew_samples) : 0.0;
        const double skew_var_us2 = s.skew_samples != 0
                                       ? s.skew_sum_sq_us / static_cast<double>(s.skew_samples)
                                             - skew_mean_us * skew_mean_us
                                       : 0.0;
        const double skew_stddev_us = skew_var_us2 > 0.0 ? std::sqrt(skew_var_us2) : 0.0;

        RCLCPP_INFO(
            get_logger(),
            "[%s] 错相: both_miss=%.2f%% motor_only=%.2f%% imu_only=%.2f%% | "
            "skew(motor.age-imu.age) mean=%.0fus stddev=%.0fus max|.|=%.0fus (n=%llu)",
            get_component_name().c_str(), pct(s.both_miss, s.ticks), pct(s.motor_only_miss, s.ticks),
            pct(s.imu_only_miss, s.ticks), skew_mean_us, skew_stddev_us, s.skew_abs_max_us,
            static_cast<unsigned long long>(s.skew_samples));

        // 一发一收本身的往返时间：send_command() 交出帧到 can_receive 落地。
        const double rtt_mean_us =
            s.rtt_samples != 0 ? s.rtt_sum_us / static_cast<double>(s.rtt_samples) : 0.0;
        RCLCPP_INFO(
            get_logger(), "[%s] CAN一发一收 rtt(mean/min/max)=%.0f/%.0f/%.0fus (n=%llu)",
            get_component_name().c_str(), rtt_mean_us, std::max(s.rtt_min_us, 0.0), s.rtt_max_us,
            static_cast<unsigned long long>(s.rtt_samples));

        // IMU 内部时钟跟主机时钟对表：IMU 自报的时间流逝速度，跟主机实测的流逝速度
        // 是不是一致。基线不够长时 ms 分辨率下 ppm 级差异测不出来，跳过没意义的打印。
        const auto clock_check = imu_clock_check_snapshot_.read(hcs_sync::Clock::now());
        if (clock_check.valid && clock_check.value.started) {
            const auto& c = clock_check.value;
            const double host_elapsed_s =
                static_cast<double>(c.last_host_ns - c.first_host_ns) / 1e9;
            const double imu_elapsed_s =
                (static_cast<double>(c.last_system_ms) - static_cast<double>(c.first_system_ms))
                / 1000.0;
            if (host_elapsed_s >= 5.0) {
                const double drift_ms = (imu_elapsed_s - host_elapsed_s) * 1000.0;
                const double ppm = (imu_elapsed_s - host_elapsed_s) / host_elapsed_s * 1e6;
                RCLCPP_INFO(
                    get_logger(),
                    "[%s] IMU时钟核对: 基线=%.1fs IMU侧流逝-主机侧流逝=%.1fms (%.1fppm，"
                    "正值=IMU自认为的时间比主机实测的走得快)",
                    get_component_name().c_str(), host_elapsed_s, drift_ms, ppm);
            }
        }
    }

    // 配置：构造函数体里用 get_parameter_or 填，这里的初值只是没配置时的兜底。
    std::uint32_t can_id_{0x145};
    std::uint32_t response_can_id_{0x145};
    std::uint64_t report_stride_{1000};
    std::string serial_filter_{};
    bool skip_version_checks_ = false;

    // 只被 report() 触碰（ROS wall timer 回调，单线程），不需要原子。
    std::uint64_t report_count_ = 0;
    std::uint64_t last_tx_exceptions_ = 0;
    std::uint64_t last_imu_crc_errors_ = 0;

    // 三份数据、缓冲区、计数器必须先于 board_/sender_ 声明——析构逆序：
    // sender_ 先停发送线程，然后 board_ 停传输事件线程，最后这些数据才安全消失。
    hcs_sync::Snapshot<MotorSample> motor_snapshot_;
    hcs_sync::Snapshot<ImuSample> imu_snapshot_;
    hcs_sync::Snapshot<Stats> stats_snapshot_;
    hcs_sync::Snapshot<ImuClockCheck> imu_clock_check_snapshot_;

    Stats stats_{}; ///< 周期域私有累计值，只被 update() 触碰，不需要原子。
    ImuClockCheck imu_clock_check_{}; ///< 同上，从不清零，见类型定义处注释。

    std::vector<std::byte> uart_rx_buffer_; ///< 只被 uart0_receive_callback 触碰。
    std::atomic<std::uint64_t> imu_crc_errors_{0};
    bool imu_logged_once_ = false; ///< 只被 uart0_receive_callback 触碰。

    // send_command()（sender_ 线程）写、can_receive()（board_ 的 IO 线程）读，
    // 两条线程各自独立，靠这一个原子量配对"发出去"和"答回来"，撑起 rtt_us 的计算。
    std::atomic<std::int64_t> last_tx_ns_{0};

    std::unique_ptr<libhcs::board::Hpm5321> board_;
    hcs_utility::DoorbellWorker<std::function<void()>> sender_;

    rclcpp::TimerBase::SharedPtr report_timer_;
};

} // namespace hcs_demo::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_demo::hardware::CanRttProbe, hcs_executor::Component)
