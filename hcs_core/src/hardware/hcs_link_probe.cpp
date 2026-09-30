#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <libhcs/board/common.hpp>
#include <libhcs/board/hcs_can_port.hpp>
#include <libhcs/board/hpm5321.hpp>
#include <libhcs/board/mc02.hpp>
#include <libhcs/data/datas.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/timer.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_sync/event_queue.hpp>
#include <hcs_sync/snapshot.hpp>
#include <hcs_sync/tick.hpp>
#include <hcs_utility/doorbell.hpp>
#include <hcs_utility/doorbell_worker.hpp>
#include <hcs_utility/machine_guard.hpp>
#include <hcs_utility/rt_attributes.hpp>
#include <hcs_utility/thread_config.hpp>

namespace hcs_demo::hardware {

// ============================================================================
// 链路探针：一个实例 = 一块 libhcs 板卡，在真实执行器里量主机侧每一段花了多久。
//
// 形态和真车一致：控制线程在 update() 里只登记"这一拍要发"，拍尾门铃叫醒
// DoorbellWorker 发送线程去调 start_transmit()；回包在 libhcs 的 IO 线程回调里落时间戳。
// 板上 CAN1 接 CAN2 回环。每帧 8 字节 = 序号 + 序号的散列，逐帧配对，不靠"最近一次"——
// 这样晚到、丢帧都能单独数出来，不会把 rtt 算宽。
//
// 每块板每 report_period_ms 报一次（窗口值 + 从启动累计），单位 us：
//   - cmd   本拍 scheduled -> 发送线程里 start_transmit() 返回。拍尾、门铃唤醒、SDK 提交
//           全算上，回答"算完的命令多久才交到 USB 手里"。
//   - tx    发送线程里 start_transmit() 本身（拿发送缓冲 + libusb 提交）。
//   - rtt   发送线程开始调 SDK -> IO 线程回调收到回包。
//   - miss  上一拍发过、这一拍 update() 读 Snapshot 却没有新回包的拍数。
//
// rtt 超过 slow_rtt_threshold_us（默认 500 us）的回包逐条记进无锁环，报告线程打印：
// 到达时刻（CLOCK_MONOTONIC ns）、rtt、序号。到达时刻和
// `trace-cmd record -C mono` 的 trace 时间轴同基准，可直接按时刻对齐去找
// 那 1 ms 里是谁没被调度。回调路径不加锁、不分配、不打日志，见 on_echo。
//
// 板型只做了 hpm5321 和 mc02 两种：日常硬件就是两块 HPM5321 DualCan + 一块低速 mc02。
// ============================================================================

namespace link_probe {

/// 1 us 一格的延迟直方图：0..4095 us，另加一个溢出格。
///
/// 允许多个写者线程（fetch_add 是原子自增）：inline_submit 时 RT 线程和发送线程
/// 会并发记录。报告线程随时读，读到的是一份不严格一致的快照——两次读之间
/// 写者可能又记了几个样本，对"几秒报一次分位数"来说可以忽略，换来的是
/// 写侧不加锁、不分配、不进内核。
class LatencyHistogram {
public:
    static constexpr std::size_t kBuckets = 4096;

    void record(std::int64_t duration_ns) noexcept {
        const std::int64_t us = duration_ns > 0 ? duration_ns / 1000 : 0;
        const std::size_t index =
            us < static_cast<std::int64_t>(kBuckets) ? static_cast<std::size_t>(us) : kBuckets;
        counts_[index].fetch_add(1, std::memory_order_relaxed);
        if (duration_ns > max_ns_.load(std::memory_order_relaxed))
            max_ns_.store(duration_ns, std::memory_order_relaxed);
    }

    void copy_to(std::vector<std::uint64_t>& counts) const {
        counts.resize(kBuckets + 1);
        for (std::size_t index = 0; index <= kBuckets; ++index)
            counts[index] = counts_[index].load(std::memory_order_relaxed);
    }

    [[nodiscard]] std::int64_t max_ns() const noexcept {
        return max_ns_.load(std::memory_order_relaxed);
    }

private:
    std::array<std::atomic<std::uint64_t>, kBuckets + 1> counts_{};
    std::atomic<std::int64_t> max_ns_{0};
};

struct Summary {
    std::uint64_t count = 0;
    std::uint64_t over_1ms = 0; ///< >= 1000 us 的样本数
    std::size_t p50 = 0;
    std::size_t p99 = 0;
    std::size_t p999 = 0;
    std::size_t max = 0;   ///< 最高的非空格（us）
    bool overflow = false; ///< 有样本落进溢出格（>= 4096 us），此时 max 不可信
};

inline Summary summarize(const std::vector<std::uint64_t>& counts) {
    Summary summary;
    for (const auto count : counts)
        summary.count += count;
    if (summary.count == 0)
        return summary;

    for (std::size_t index = 1000; index < counts.size(); ++index)
        summary.over_1ms += counts[index];

    const auto percentile = [&counts, total = summary.count](double quantile) {
        const auto rank = std::max<std::uint64_t>(
            1, static_cast<std::uint64_t>(std::ceil(quantile * static_cast<double>(total))));
        std::uint64_t seen = 0;
        for (std::size_t index = 0; index < counts.size(); ++index) {
            seen += counts[index];
            if (seen >= rank)
                return index;
        }
        return counts.size() - 1;
    };
    summary.p50 = percentile(0.5);
    summary.p99 = percentile(0.99);
    summary.p999 = percentile(0.999);

    for (std::size_t index = counts.size(); index-- > 0;) {
        if (counts[index] != 0) {
            summary.max = index;
            break;
        }
    }
    summary.overflow = counts.back() != 0;
    return summary;
}

/// 一条慢回包的原始记录。arrival 是 IO 线程回调拿到回包的 CLOCK_MONOTONIC 时刻
/// （hcs_sync::Clock = steady_clock = CLOCK_MONOTONIC），与
/// `trace-cmd record -C mono` 的 trace 时间戳同基准，直接按纳秒对齐。
struct SlowRtt {
    std::int64_t arrival_ns = 0;
    std::int64_t rtt_ns = 0;
    std::uint32_t sequence = 0;
};

} // namespace link_probe

class HcsLinkProbe
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    HcsLinkProbe()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , hpm5321_callback_{*this}
        , mc02_callback_{*this}
        , sender_(
              hcs_utility::ThreadConfig{
                  string_parameter("tx_thread_config", ""), get_component_name() + "-tx"},
              [this] { send(); }) {

        board_type_ = string_parameter("board", "hpm5321");
        const bool is_mc02 = board_type_ == "mc02";
        serial_filter_ = string_parameter("serial_filter", "");
        can_id_ = static_cast<std::uint32_t>(int_parameter("can_id", is_mc02 ? 0x555 : 0x556));
        fdcan_ = bool_parameter("fdcan", !is_mc02);
        send_every_ =
            static_cast<std::uint64_t>(std::max<std::int64_t>(1, int_parameter("send_every", 1)));
        // 过载测试用：过载阶段每拍发 burst 帧（打进同一个 USB 包），把 CAN1 压过线速，
        // 观察板子过载时会话与其他板是否受影响、过载结束后是否恢复。burst_on_ms /
        // burst_off_ms 让过载与正常交替（on=0 表示全程过载）。默认 burst=1 即关闭。
        burst_ = static_cast<std::uint32_t>(
            std::clamp<std::int64_t>(int_parameter("burst", 1), 1, kMaxBurst));
        burst_on_ns_ = int_parameter("burst_on_ms", 0) * 1'000'000;
        burst_off_ns_ = int_parameter("burst_off_ms", 0) * 1'000'000;
        const auto io_thread_cpu = int_parameter("io_thread_cpu", -1);
        const auto io_thread_rt_priority = int_parameter("io_thread_rt_priority", 0);
        inline_submit_ = bool_parameter("inline_submit", false);
        const auto report_period_ms =
            std::max<std::int64_t>(200, int_parameter("report_period_ms", 5000));
        slow_rtt_threshold_ns_ = int_parameter("slow_rtt_threshold_us", 500) * 1000;

        if (!sender_.thread_config_error().empty())
            RCLCPP_WARN(
                get_logger(), "[%s] tx thread config: %s", get_component_name().c_str(),
                sender_.thread_config_error().c_str());

        // 配置漂移防护：IO 线程留在普通核上就是 SCHED_OTHER，桌面负载下尾部实测到过
        // 毫秒级（绑隔离核 + FIFO 后 giveback -> 回调 < 15 us）。这板子是辅助低速件
        // 也要先知道代价再决定，而不是从日志里猜。
        if (io_thread_cpu < 0)
            RCLCPP_WARN(
                get_logger(),
                "[%s] io_thread_cpu=-1: the USB event thread runs unpinned under SCHED_OTHER; "
                "its rtt tail is then at the mercy of CFS load (measured up to tens of ms). "
                "Pin it to an isolated core with an RT priority for deterministic tails.",
                get_component_name().c_str());

        // 板卡在构造期就可能开始回调，所以放在所有它会碰的数据都就位之后。
        // io_thread_cpu 设了之后，libhcs 的保活线程会跟到同一组核、低一级实时优先级。
        auto options = libhcs::board::AdvancedOptions{};
        options.dangerously_skip_version_checks =
            bool_parameter("dangerously_skip_version_checks", false);
        if (io_thread_cpu >= 0)
            options.set_io_thread_affinity(
                static_cast<int>(io_thread_cpu), static_cast<int>(io_thread_rt_priority));

        if (board_type_ == "hpm5321")
            hpm5321_ = std::make_unique<libhcs::board::Hpm5321>(
                hpm5321_callback_, serial_filter_, options);
        else if (is_mc02)
            mc02_ = std::make_unique<libhcs::board::Mc02>(mc02_callback_, serial_filter_, options);
        else
            throw std::invalid_argument{
                std::format("board must be hpm5321 or mc02, got '{}'", board_type_)};

        // 线上没有逐帧 fd 标志了（CanDataView 不再有 is_fdcan，bit 4 保留）：帧型是
        // 总线的属性，由固件在初始化时定、经 EP0 上报。yaml 里的 fdcan 参数退化成
        // 交叉校验：和板子自报的不一致只警告不改行为——发什么帧型由板子说了算。
        const bool board_can1_is_fd =
            hpm5321_ ? hpm5321_->can1_is_fd() : (mc02_ ? mc02_->can1_is_fd() : fdcan_);
        if (board_can1_is_fd != fdcan_)
            RCLCPP_WARN(
                get_logger(),
                "[%s] yaml fdcan=%s but the board reports CAN1 %s (EP0); the board wins",
                get_component_name().c_str(), fdcan_ ? "true" : "false",
                board_can1_is_fd ? "FD" : "classic");

        last_report_ = hcs_sync::Clock::now();
        report_timer_ = create_wall_timer(
            std::chrono::milliseconds{report_period_ms}, [this] { report(); });

        RCLCPP_INFO(
            get_logger(),
            "[%s] link probe: board=%s serial_filter='%s' can_id=0x%03X %s send_every=%llu "
            "burst=%u burst_on_ms=%lld burst_off_ms=%lld "
            "io_thread_cpu=%lld io_thread_rt_priority=%lld slow_rtt_threshold_us=%lld "
            "inline_submit=%d",
            get_component_name().c_str(), board_type_.c_str(), serial_filter_.c_str(), can_id_,
            fdcan_ ? "fd" : "classic", static_cast<unsigned long long>(send_every_), burst_,
            static_cast<long long>(burst_on_ns_ / 1'000'000),
            static_cast<long long>(burst_off_ns_ / 1'000'000),
            static_cast<long long>(io_thread_cpu), static_cast<long long>(io_thread_rt_priority),
            static_cast<long long>(slow_rtt_threshold_ns_ / 1000), inline_submit_ ? 1 : 0);

        // 机器护栏：只读检查 USB 实时路径依赖的内核状态（IRQ 线程优先级/落核、中断核
        // C1、板卡枚举速度、bootloader 滞留……），判据来自 HOST_TUNING.md 的实测。
        // 需要修改的项归 hcs_rt_tune.sh（开机服务）管，这里只负责发现"它没生效"。
        // 每进程只跑一次：构造函数按板各跑一遍，护栏结果与板无关。
        static std::once_flag machine_guard_once;
        std::call_once(machine_guard_once, [this]() {
            for (const auto& finding : hcs_utility::MachineGuard::run()) {
                const char* text = finding.text.c_str();
                switch (finding.level) {
                case hcs_utility::MachineGuard::Level::kCritical:
                    RCLCPP_ERROR(get_logger(), "[machine] %s", text);
                    break;
                case hcs_utility::MachineGuard::Level::kWarn:
                    RCLCPP_WARN(get_logger(), "[machine] %s", text);
                    break;
                default: RCLCPP_INFO(get_logger(), "[machine] %s", text); break;
                }
            }
        });
    }

    ~HcsLinkProbe() override = default;

    hcs_utility::Doorbell* tick_end_doorbell() override { return &sender_.doorbell(); }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        const auto echo = echo_snapshot_.read(tick.scheduled);
        if (sent_last_tick_ && !echo.fresh)
            increment(misses_);
        increment(ticks_);

        // 只登记，不发：发送在拍尾门铃叫醒的发送线程里做。
        const bool send_this_tick = tick.sequence % send_every_ == 0;
        if (send_this_tick)
            pending_scheduled_ns_.store(to_ns(tick.scheduled), std::memory_order_release);
        sent_last_tick_ = send_this_tick;

        // inline_submit A/B：在 RT 线程内直接发送，省掉门铃唤醒一跳。submit 的
        // syscall 进了周期域（打破"RT 线程零 syscall"纪律）——这正是实验要量的
        // 东西：换来的 cmd 段缩短是否大于纪律损失。异常不上报隔离：交给 executor
        // 现成的组件隔离路径（HCS_NONBLOCKING 函数禁止 catch）。
        if (inline_submit_ && send_this_tick)
            send();
    }

private:
    struct Echo {
        std::uint32_t sequence = 0;
    };

    class Hpm5321Callback final : public libhcs::board::Hpm5321::Callback {
    public:
        explicit Hpm5321Callback(HcsLinkProbe& probe)
            : probe_(probe) {}

        void can_receive(
            libhcs::board::hcs::CanPort port, const libhcs::data::CanDataView& data) override {
            if (port == libhcs::board::hcs::CanPort::kCan2)
                probe_.on_echo(data);
        }

    private:
        HcsLinkProbe& probe_;
    };

    class Mc02Callback final : public libhcs::board::Mc02::Callback {
    public:
        explicit Mc02Callback(HcsLinkProbe& probe)
            : probe_(probe) {}

        void can2_receive_callback(const libhcs::data::CanDataView& data) override {
            probe_.on_echo(data);
        }

    private:
        HcsLinkProbe& probe_;
    };

    struct Counters {
        std::uint64_t ticks = 0;
        std::uint64_t sent = 0;
        std::uint64_t received = 0;
        std::uint64_t stray = 0;
        std::uint64_t misses = 0;
        std::uint64_t tx_exceptions = 0;
    };

    // 序号槽：发送线程写，IO 线程读。4096 个槽 = 1 kHz 下 4 秒才回绕，晚到 4 秒的回包
    // 按丢帧算。先写时间再写序号：IO 线程只要读到序号对得上，时间一定已经是新的。
    static constexpr std::size_t kSentSlots = 4096;
    static constexpr std::uint32_t kSentMask = kSentSlots - 1;

    // 慢回包环的容量。满就丢新（计数），不阻塞回调：4096 条 = 1 kHz 全慢时也够
    // 4 秒，远长于一个报告窗口，真溢出说明问题已经不是偶发而是常态。
    static constexpr std::size_t kSlowRttSlots = 4096;

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

    static std::int64_t to_ns(hcs_sync::Timestamp timestamp) noexcept {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch())
            .count();
    }

    /// 计数器自增。fetch_add 而非 load+store：inline_submit 时 RT 线程和发送线程
    /// 会并发碰 sent_。
    static void increment(std::atomic<std::uint64_t>& counter) noexcept {
        counter.fetch_add(1, std::memory_order_relaxed);
    }

    static std::uint32_t mix(std::uint32_t x) noexcept {
        x ^= x >> 16;
        x *= 0x7feb352dU;
        x ^= x >> 15;
        x *= 0x846ca68bU;
        x ^= x >> 16;
        return x;
    }

    static void put_u32(std::byte* destination, std::uint32_t value) noexcept {
        for (int i = 0; i < 4; ++i)
            destination[i] = static_cast<std::byte>(value >> (8 * i));
    }

    static std::uint32_t get_u32(const std::byte* source) noexcept {
        std::uint32_t value = 0;
        for (int i = 0; i < 4; ++i)
            value |= static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(source[i])) << (8 * i);
        return value;
    }

    // 发送线程（拍尾门铃叫醒）。门铃每拍都响，send_every > 1 的拍、以及门铃合并时
    // 已经发过的那一拍，靠 pending_scheduled_ns_ 有没有变来跳过。inline_submit_ 时
    // 本函数改由 ctrl 线程在 update() 里直接调用（A/B 实验：省掉门铃唤醒，代价是
    // submit syscall 进入 RT 线程）；tx 线程仍会被门铃叫醒，但水位已推进，立即返回。
    void send() {
        const auto scheduled_ns = pending_scheduled_ns_.load(std::memory_order_acquire);
        // inline_submit 时本函数会被 RT 线程和发送线程并发进入。水位用 CAS 推进：
        // 谁把它从旧值推到本拍的 scheduled_ns，谁负责发送；失败者要么这一拍已被
        // 对方发过（水位已到本拍值），要么水位已领先到更新的拍，两种情况都直接
        // 返回。不能写成 load-check-store——那是个 TOCTOU：两个线程可能同时通过
        // 检查，然后把下面的序号分配、sent_ns_ 槽位和直方图全部写花。
        auto observed = handled_scheduled_ns_.load(std::memory_order_acquire);
        if (observed >= scheduled_ns)
            return;
        if (!handled_scheduled_ns_.compare_exchange_strong(
                observed, scheduled_ns, std::memory_order_acq_rel, std::memory_order_acquire))
            return;

        const std::uint32_t frames = frames_this_tick(scheduled_ns);
        std::array<std::array<std::byte, 8>, kMaxBurst> payloads{};
        const auto start_ns = to_ns(hcs_sync::Clock::now());
        if (hpm5321_) {
            auto builder = hpm5321_->start_transmit();
            for (std::uint32_t i = 0; i < frames; ++i)
                builder.can_transmit(
                    libhcs::board::hcs::CanPort::kCan1, next_frame(payloads[i], start_ns));
        } else if (mc02_) {
            auto builder = mc02_->start_transmit();
            for (std::uint32_t i = 0; i < frames; ++i)
                builder.can_transmit(
                    libhcs::board::hcs::CanPort::kCan1, next_frame(payloads[i], start_ns));
        }

        const auto done_ns = to_ns(hcs_sync::Clock::now());
        tx_.record(done_ns - start_ns);
        cmd_.record(done_ns - scheduled_ns);
    }

    // 发送线程：给下一帧编号、登记发送时刻，返回指向 payload 的帧视图。
    libhcs::data::CanDataView next_frame(std::array<std::byte, 8>& payload, std::int64_t sent_ns) {
        const std::uint32_t sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
        put_u32(payload.data(), sequence);
        put_u32(payload.data() + 4, mix(sequence));
        const std::size_t slot = sequence & kSentMask;
        sent_ns_[slot].store(sent_ns, std::memory_order_relaxed);
        sent_sequence_[slot].store(sequence, std::memory_order_relaxed);
        increment(sent_);
        return libhcs::data::CanDataView{.can_id = can_id_, .can_data = payload};
    }

    [[nodiscard]] std::uint32_t frames_this_tick(std::int64_t scheduled_ns) const noexcept {
        if (burst_ == 1)
            return 1;
        const std::int64_t period = burst_on_ns_ + burst_off_ns_;
        if (burst_on_ns_ <= 0 || burst_off_ns_ <= 0)
            return burst_;
        return scheduled_ns % period < burst_on_ns_ ? burst_ : 1;
    }

    // 事件域：libhcs IO 线程。只校验、记直方图、发布 Snapshot——不分配、不加锁、不打日志，
    // 因为同一个核上可能还有别的板子的 IO 线程在排队。
    void on_echo(const libhcs::data::CanDataView& data) noexcept {
        const auto arrival = hcs_sync::Clock::now();
        if (data.can_id != can_id_ || data.is_extended_can_id || data.can_data.size() != 8)
            return;

        const std::uint32_t sequence = get_u32(data.can_data.data());
        const std::size_t slot = sequence & kSentMask;
        if (get_u32(data.can_data.data() + 4) != mix(sequence)
            || sent_sequence_[slot].load(std::memory_order_relaxed) != sequence) {
            increment(stray_);
            return;
        }

        const auto rtt_ns = to_ns(arrival) - sent_ns_[slot].load(std::memory_order_relaxed);
        rtt_.record(rtt_ns);
        if (rtt_ns > slow_rtt_threshold_ns_)
            slow_rtts_.try_push(link_probe::SlowRtt{
                .arrival_ns = to_ns(arrival), .rtt_ns = rtt_ns, .sequence = sequence});
        increment(received_);
        echo_snapshot_.publish(Echo{.sequence = sequence}, arrival);
    }

    [[nodiscard]] Counters read_counters() const {
        return Counters{
            .ticks = ticks_.load(std::memory_order_relaxed),
            .sent = sent_.load(std::memory_order_relaxed),
            .received = received_.load(std::memory_order_relaxed),
            .stray = stray_.load(std::memory_order_relaxed),
            .misses = misses_.load(std::memory_order_relaxed),
            .tx_exceptions = sender_.exceptions(),
        };
    }

    /// "p50/p99/p99.9/max"。窗口里有样本落进溢出格时 max 写成 ">=4096"；
    /// 累计值则换成写者记的精确最大值。
    static std::string format_latency(
        const link_probe::Summary& summary, std::int64_t exact_max_ns, bool cumulative) {
        if (summary.count == 0)
            return "-";
        std::string max_text = std::format("{}", summary.max);
        if (summary.overflow)
            max_text = cumulative ? std::format("{:.0f}", static_cast<double>(exact_max_ns) / 1e3)
                                  : std::string{">=4096"};
        return std::format("{}/{}/{}/{}", summary.p50, summary.p99, summary.p999, max_text);
    }

    // 尽力域：ROS wall timer 回调。
    void report() {
        const auto now = hcs_sync::Clock::now();
        const double window_seconds = std::chrono::duration<double>(now - last_report_).count();
        last_report_ = now;

        rtt_.copy_to(rtt_now_);
        tx_.copy_to(tx_now_);
        cmd_.copy_to(cmd_now_);
        const auto counters = read_counters();

        // 慢回包逐条打印。arrival 是 CLOCK_MONOTONIC ns，trace 用 -C mono 录的就是
        // 同一条时间轴。满了丢新并计数（EventQueue 的显式策略），这里只报累计丢弃数。
        link_probe::SlowRtt slow;
        while (slow_rtts_.try_pop(slow))
            RCLCPP_INFO(
                get_logger(), "[%s] slow rtt: arrival=%lld ns rtt=%.1f us seq=%u",
                get_component_name().c_str(), static_cast<long long>(slow.arrival_ns),
                static_cast<double>(slow.rtt_ns) / 1e3, slow.sequence);
        if (const auto dropped = slow_rtts_.dropped(); dropped != slow_dropped_reported_) {
            RCLCPP_WARN(
                get_logger(), "[%s] slow rtt ring overflow: dropped=%llu total",
                get_component_name().c_str(), static_cast<unsigned long long>(dropped));
            slow_dropped_reported_ = dropped;
        }

        const auto window_of = [](const std::vector<std::uint64_t>& current,
                                  const std::vector<std::uint64_t>& previous,
                                  std::vector<std::uint64_t>& window) {
            window.assign(current.size(), 0);
            for (std::size_t index = 0; index < current.size(); ++index)
                window[index] = current[index] - (index < previous.size() ? previous[index] : 0);
        };
        window_of(rtt_now_, rtt_previous_, rtt_window_);
        window_of(tx_now_, tx_previous_, tx_window_);
        window_of(cmd_now_, cmd_previous_, cmd_window_);

        const auto print = [this](
                               const char* label, const Counters& c,
                               const std::vector<std::uint64_t>& rtt,
                               const std::vector<std::uint64_t>& tx,
                               const std::vector<std::uint64_t>& cmd, bool cumulative) {
            const auto rtt_summary = link_probe::summarize(rtt);
            const auto line = std::format(
                "[{}] {} ticks={} sent={} recv={} lost={} miss={} stray={} tx_exc={} | "
                "rtt p50/p99/p99.9/max={} >=1ms={} | tx={} | cmd={}",
                get_component_name(), label, c.ticks, c.sent, c.received,
                static_cast<std::int64_t>(c.sent) - static_cast<std::int64_t>(c.received),
                c.misses, c.stray, c.tx_exceptions,
                format_latency(rtt_summary, rtt_.max_ns(), cumulative), rtt_summary.over_1ms,
                format_latency(link_probe::summarize(tx), tx_.max_ns(), cumulative),
                format_latency(link_probe::summarize(cmd), cmd_.max_ns(), cumulative));
            RCLCPP_INFO(get_logger(), "%s", line.c_str());
        };

        const Counters window_counters{
            .ticks = counters.ticks - previous_counters_.ticks,
            .sent = counters.sent - previous_counters_.sent,
            .received = counters.received - previous_counters_.received,
            .stray = counters.stray - previous_counters_.stray,
            .misses = counters.misses - previous_counters_.misses,
            .tx_exceptions = counters.tx_exceptions - previous_counters_.tx_exceptions,
        };
        const auto window_label = std::format("window {:.1f}s", window_seconds);
        print(window_label.c_str(), window_counters, rtt_window_, tx_window_, cmd_window_, false);
        print("total", counters, rtt_now_, tx_now_, cmd_now_, true);

        std::swap(rtt_previous_, rtt_now_);
        std::swap(tx_previous_, tx_now_);
        std::swap(cmd_previous_, cmd_now_);
        previous_counters_ = counters;
    }

    // 配置：构造函数体里填，之后只读。
    std::string board_type_;
    std::string serial_filter_;
    std::uint32_t can_id_ = 0x556;
    bool fdcan_ = true;
    std::uint64_t send_every_ = 1;
    static constexpr std::int64_t kMaxBurst = 40; // 40 条 11 字节记录仍装得进一个 512 字节包
    std::uint32_t burst_ = 1;
    std::int64_t burst_on_ns_ = 0;
    std::int64_t burst_off_ns_ = 0;
    std::int64_t slow_rtt_threshold_ns_ = 500 * 1000; ///< IO 线程读，超阈值才进慢回包环

    // 周期域私有。
    bool sent_last_tick_ = false;
    bool inline_submit_ = false; ///< A/B：拍内直接 submit（submit syscall 进周期域）

    // 周期域写、发送线程读。
    std::atomic<std::int64_t> pending_scheduled_ns_{0};

    // inline_submit 时 RT 线程和发送线程都可能分配序号，atomic 保证不重号。
    // 序号从 1 开始：槽的初值 0 永远配不上。
    std::atomic<std::uint32_t> next_sequence_{1};
    // send() 的两路并发调用者（发送线程 / RT 线程 inline）用 CAS 推进的去重水位，
    // 只前进；CAS 赢家负责发送本拍。
    std::atomic<std::int64_t> handled_scheduled_ns_{0};

    // 跨线程数据：每个都只有一个写者线程，见各自注释。
    std::array<std::atomic<std::int64_t>, kSentSlots> sent_ns_{};        ///< send() 写，槽位按序号唯一
    std::array<std::atomic<std::uint32_t>, kSentSlots> sent_sequence_{}; ///< send() 写，槽位按序号唯一
    std::atomic<std::uint64_t> sent_{0};                                 ///< send() 写（可能两线程并发）
    std::atomic<std::uint64_t> received_{0};                             ///< IO 线程写
    std::atomic<std::uint64_t> stray_{0};                                ///< IO 线程写
    std::atomic<std::uint64_t> ticks_{0};                                ///< 周期域写
    std::atomic<std::uint64_t> misses_{0};                               ///< 周期域写
    link_probe::LatencyHistogram rtt_;                                   ///< IO 线程写
    link_probe::LatencyHistogram tx_;                                    ///< send() 写（可能两线程并发）
    link_probe::LatencyHistogram cmd_;                                   ///< send() 写（可能两线程并发）
    hcs_sync::Snapshot<Echo> echo_snapshot_;                            ///< IO 线程写
    hcs_sync::EventQueue<link_probe::SlowRtt, kSlowRttSlots> slow_rtts_; ///< IO 线程写，报告线程读

    // 报告线程私有。
    hcs_sync::Timestamp last_report_{};
    Counters previous_counters_{};
    std::uint64_t slow_dropped_reported_ = 0;
    std::vector<std::uint64_t> rtt_now_, tx_now_, cmd_now_;
    std::vector<std::uint64_t> rtt_previous_, tx_previous_, cmd_previous_;
    std::vector<std::uint64_t> rtt_window_, tx_window_, cmd_window_;

    // 析构逆序：report_timer_ -> sender_（先停发送线程）-> 板卡（停 IO 线程）-> 回调对象
    // -> 上面的数据。发送线程和 IO 线程都会碰数据，所以数据必须最后消失。
    Hpm5321Callback hpm5321_callback_;
    Mc02Callback mc02_callback_;
    std::unique_ptr<libhcs::board::Hpm5321> hpm5321_;
    std::unique_ptr<libhcs::board::Mc02> mc02_;
    hcs_utility::DoorbellWorker<std::function<void()>> sender_;

    rclcpp::TimerBase::SharedPtr report_timer_;
};

} // namespace hcs_demo::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_demo::hardware::HcsLinkProbe, hcs_executor::Component)
