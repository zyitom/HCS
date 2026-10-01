#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>

#include <hcs_base/channel/tick.hpp>
#include <hcs_base/channel/time_base.hpp>

#include "predefined_msg_provider.hpp"
#include "hcs_executor/component.hpp"
#include "hcs_executor/graph.hpp"
#include "hcs_executor/wiring.hpp"
#include "rt_reporter.hpp"
#include "hcs_base/thread/doorbell.hpp"
#include "hcs_base/check/machine_guard.hpp"
#include "hcs_base/thread/precise_sleep.hpp"
#include "hcs_base/thread/realtime_arm.hpp"
#include "hcs_base/thread/realtime_scope.hpp"
#include "rt_sampler.hpp"
#include "hcs_base/thread/thread_config.hpp"

// 逐组件计时默认关：开着的话每个组件每拍多两次 clock_gettime，
// 在 1 kHz × 几十个组件的量级上这本身就是被测量的东西的一大部分。
#ifndef HCS_EXECUTOR_COMPONENT_TIMING
#    define HCS_EXECUTOR_COMPONENT_TIMING 0
#endif

namespace hcs_executor {

class Executor final : public rclcpp::Node {
public:
    explicit Executor(
        const std::string& node_name, rclcpp::executors::SingleThreadedExecutor& rcl_executor)
        : Node{
              node_name,
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)}
        , rcl_executor_(rcl_executor) {
        predefined_msg_provider_ = std::make_shared<PredefinedMsgProvider>();
        add_component(predefined_msg_provider_);
    }

    ~Executor() override {
        // 先停控制线程：它是唯一还在碰组件输出的线程。
        running_.store(false, std::memory_order_relaxed);
        if (thread_.joinable())
            thread_.join();

        if (failed_report_timer_)
            failed_report_timer_->cancel();

        if (reporter_) {
            reporter_->stop();
            reporter_.reset();
        }
    }

    void add_component(const std::shared_ptr<Component>& component) {
        component_list_.emplace_back(component);
        if (auto node = std::dynamic_pointer_cast<rclcpp::Node>(component))
            rcl_executor_.add_node(node);
        for (const auto& partner_component : component->partner_components())
            add_component(partner_component);
    }

    /// 启动控制线程与报告线程。
    ///
    /// 控制线程的实时武装失败会被原样 rethrow 到这里的调用方 —— 改造前那条路径只打一条
    /// WARN 就继续跑，结果是"以为自己是 RT，其实不是"，比直接起不来危险得多。
    void start() {
        run_machine_guard();
        init();

        for (auto& component : component_list_)
            component->before_updating();

        double update_rate;
        if (!get_parameter("update_rate", update_rate))
            throw std::runtime_error{"Unable to get parameter update_rate<double>"};
        if (!std::isfinite(update_rate) || update_rate <= 0.0)
            throw std::runtime_error{"Parameter update_rate must be finite and positive"};
        const auto period =
            std::chrono::nanoseconds(static_cast<long>(std::round(1'000'000'000.0 / update_rate)));
        if (period.count() <= 0)
            throw std::runtime_error{
                "Parameter update_rate is too large to produce a valid period"};
        predefined_msg_provider_->set_update_rate(update_rate);

        std::string thread_config_spec;
        get_parameter_or<std::string>("thread_config", thread_config_spec, "");
        auto thread_config = hcs_utility::ThreadConfig{thread_config_spec, "update"};

        std::string realtime_config_spec;
        get_parameter_or<std::string>("realtime_config", realtime_config_spec, "");
        const auto arm_options = hcs_utility::RealtimeArmOptions::parse(realtime_config_spec);

        std::string reporter_thread_config_spec;
        get_parameter_or<std::string>(
            "reporter_thread_config", reporter_thread_config_spec,
            "name=hcs-report;policy=other;nice=5");
        auto reporter_thread_config =
            hcs_utility::ThreadConfig{reporter_thread_config_spec, "hcs-report"};

        double report_period_seconds;
        get_parameter_or<double>("report_period_seconds", report_period_seconds, 5.0);
        // RtReporter 收的是整秒；亚秒级的报告周期只会把统计切碎，钳到 1 秒。
        const auto report_period = std::chrono::seconds{
            std::isfinite(report_period_seconds)
                ? std::max<long long>(1, std::llround(report_period_seconds))
                : 5LL};

#if HCS_EXECUTOR_COMPONENT_TIMING
        int component_timing_stride;
        get_parameter_or<int>("component_timing_stride", component_timing_stride, 16);
        component_timing_stride_ =
            component_timing_stride >= 1 ? static_cast<std::uint64_t>(component_timing_stride) : 1;
#endif

        // 失效隔离要在 RT 线程里 push_back，容量必须在这里一次性备好。
        newly_failed_.reserve(updating_order_.size());

        auto component_names = std::vector<std::string>{};
        component_names.reserve(updating_order_.size());
        for (auto* component : updating_order_)
            component_names.emplace_back(component->get_component_name());

        reporter_ = std::make_unique<RtReporter>(
            get_logger(), sampler_, std::move(component_names), report_period,
            std::move(reporter_thread_config));

        // 武装结果经 promise/future 回到主线程：控制线程在 set_value 前写 arm_summary_，
        // future.get() 之后主线程读它是有 happens-before 的。
        std::promise<void> armed;
        auto armed_future = armed.get_future();

        running_.store(true, std::memory_order_relaxed);
        thread_ = std::thread{
            [this, armed = std::move(armed), thread_config = std::move(thread_config), arm_options,
             period]() mutable {
                thread_main(std::move(armed), thread_config, arm_options, period);
            }};

        try {
            armed_future.get();
        } catch (...) {
            running_.store(false, std::memory_order_relaxed);
            if (thread_.joinable())
                thread_.join();
            throw;
        }

        // 武装摘要只在主线程打：RT 线程从进入主循环起就不许碰日志。
        RCLCPP_INFO(get_logger(), "Realtime arm: %s", arm_summary_.c_str());

        // 失效组件的名字由尽力域打：RT 线程只留下下标。
        failed_report_timer_ = create_wall_timer(
            std::chrono::milliseconds{500}, [this]() { report_newly_failed_components(); });

        reporter_->start();
    }

private:
    /// 机器护栏：只读检查 USB 实时路径依赖的内核状态（RT 限流、xHCI 中断线程优先级与落核、
    /// 中断核 C1、板卡枚举速度、bootloader 滞留……），判据来自 libhcs HOST_TUNING.md 的实测。
    /// 放在调度器里而不是各个硬件组件里：每进程只该跑一次，而且新写的硬件组件不该有机会忘掉它。
    ///
    /// 参数 `machine_guard`：
    ///   enforce（默认）有 kCritical 就拒绝启动 —— 与武装失败同一个道理：带着已知会掐死
    ///                   RT 线程的内核状态跑起来，比直接起不来危险；
    ///   warn            只打日志，照常启动（开发机、没调优的台架）；
    ///   off             不检查。
    void run_machine_guard() {
        std::string mode;
        get_parameter_or<std::string>("machine_guard", mode, "enforce");
        if (mode == "off")
            return;
        if (mode != "enforce" && mode != "warn")
            throw std::runtime_error{
                "Parameter machine_guard must be one of enforce / warn / off, got '" + mode + "'"};

        using Level = hcs_utility::MachineGuard::Level;
        std::size_t critical_count = 0;
        for (const auto& finding : hcs_utility::MachineGuard::run()) {
            const char* text = finding.text.c_str();
            switch (finding.level) {
            case Level::kCritical:
                ++critical_count;
                RCLCPP_ERROR(get_logger(), "[machine] %s", text);
                break;
            case Level::kWarn: RCLCPP_WARN(get_logger(), "[machine] %s", text); break;
            default: RCLCPP_INFO(get_logger(), "[machine] %s", text); break;
            }
        }

        if (critical_count != 0 && mode == "enforce")
            throw std::runtime_error{
                std::to_string(critical_count)
                + " critical machine finding(s) above; fix them (hcs_rt_tune.sh) or set "
                  "machine_guard: warn to start anyway"};
    }

    void thread_main(
        std::promise<void> armed, hcs_utility::ThreadConfig thread_config,
        hcs_utility::RealtimeArmOptions arm_options, std::chrono::nanoseconds period) {
        std::optional<hcs_utility::RealtimeArm> arm;
        try {
            arm.emplace(thread_config, arm_options);
            arm_summary_ = arm->summary();
            armed.set_value();
        } catch (...) {
            armed.set_exception(std::current_exception());
            return;
        }

        // ══════════════════════════════════════════════════════════════════════════
        //  封 盘 点 —— RealtimeArm 构造成功即内存布局定型（mlock / mallopt 已生效）。
        //  这条线以下的主循环里，一次 malloc / new / std::string / 日志 / 锁 都是 bug：
        //  它们会在最坏的那一拍上以缺页或 arena 争用的形式变成一个抖动尖峰。
        // ══════════════════════════════════════════════════════════════════════════

        const auto spin_guard = arm->options().spin_guard;

        auto time_base = hcs_sync::FreeRunTimeBase{period};
        auto tick = time_base.first(hcs_sync::Clock::now());
        std::uint64_t previous_sequence = tick.sequence;

        while (running_.load(std::memory_order_relaxed)) {
            const auto actual_end = execute_update_iteration(tick, previous_sequence);

            // ── 拍尾门铃 ──────────────────────────────────────────────────────
            // 周期域里**唯一**被允许的额外系统调用位置，而且只在真有等待者时才进内核
            // （见 Doorbell::ring）。放在这里有两个理由：
            //   - 越早越好：HOST_TUNING 实测提前 flush 的收益是**连续的、没有台阶**，
            //     所以命令算完就该立刻把发送线程叫起来，而不是等下一拍；
            //   - 它紧贴着下面那个 sleep —— 本来就要进内核 —— 所以这一下叠在已经要付的
            //     开销上，不引入新的抖动源。
            // 它不在 execute_update_iteration 里面，也就不在那个 RealtimeScope 里：
            // 这是刻意的，不是漏网。
            for (auto* doorbell : tick_end_doorbells_)
                doorbell->ring();

            const auto next = time_base.advance(tick, actual_end);
            previous_sequence = tick.sequence;
            tick = next;
            hcs_utility::sleep_until_precise(tick.scheduled, spin_guard);
        }
    }

    hcs_sync::Timestamp execute_update_iteration(
        const hcs_sync::Tick& tick, std::uint64_t previous_sequence) noexcept {
        const auto actual_start = hcs_sync::Clock::now();

        // 这一拍的全部工作都在实时上下文里。基类 update() 上的 HCS_NONBLOCKING 是静态的，
        // 但它管不到本函数 —— 效果分析禁止 nonblocking 函数 catch 异常，而下面的失效隔离
        // 必须 catch。没有这一行，闩存 / record_tick / 时基推进这三段就是 RTSan 的盲区。
        // A/B 实测（探针 = 一次逃逸到全局的 malloc，放在这一行下面）：
        // 有这行抓到 9733 次（一拍一次），摘掉这行抓到 0 次。
        [[maybe_unused]] const hcs_utility::RealtimeScope realtime_scope;

        // ── 1/z 闩存，必须在任何 update() 之前 ────────────────────────────────
        // 所有 DelayedInput 在这里一次性从上游存储各拷一份。放在拍首而不是"用的时候取"，
        // 是为了让"读到的是上一拍的值"这件事**与拓扑序无关**：上游本拍晚些时候写的新值
        // 一定看不见。否则同一个 1/z 会因为上下游谁先跑而给出不同的语义，
        // 而它存在的全部意义就是提供一个可预测的单位延迟。
        // 每条都是一次平凡拷贝，条数是 delayed input 的个数（通常个位数）。
        for (const auto& entry : latch_list_)
            entry.latch(entry.interface);

#if HCS_EXECUTOR_COMPONENT_TIMING
        const bool sample_components = (tick.sequence % component_timing_stride_) == 0;
#endif

        const auto component_count = static_cast<std::uint32_t>(updating_order_.size());
        for (std::uint32_t index = 0; index < component_count; ++index) {
            auto* component = updating_order_[index];
            if (component->failed_) [[unlikely]]
                continue;

#if HCS_EXECUTOR_COMPONENT_TIMING
            const auto component_start =
                sample_components ? hcs_sync::Clock::now() : hcs_sync::Timestamp{};
#endif

            try {
                component->update(tick);
            } catch (...) {
                mark_component_failed(component);
            }

#if HCS_EXECUTOR_COMPONENT_TIMING
            if (sample_components) {
                const auto elapsed = hcs_sync::Clock::now() - component_start;
                sampler_.record_component(
                    hcs_utility::ComponentSample{
                        .sequence = tick.sequence,
                        .component_index = index,
                        .duration_ns = saturate_u32(
                            std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed)
                                .count())});
            }
#endif
        }

        const auto actual_end = hcs_sync::Clock::now();

        // 首拍的 sequence 与 previous_sequence 相等，钳到 0；其余情况差值减一就是跳拍数。
        const std::uint64_t skipped =
            tick.sequence > previous_sequence ? tick.sequence - previous_sequence - 1 : 0;

        sampler_.record_tick(
            hcs_utility::TickSample{
                .sequence = tick.sequence,
                .scheduled_ns = to_nanoseconds(tick.scheduled),
                .start_ns = to_nanoseconds(actual_start),
                .end_ns = to_nanoseconds(actual_end),
                .skipped = saturate_u32(static_cast<std::int64_t>(skipped)),
                .failed_count = failed_count_});

        return actual_end;
    }

    /// 隔离逻辑本体在 Linker::isolate 里（noexcept、不分配、不打日志）。
    /// 这里只补两件 Executor 才知道的事：把新失效的下标长度发布给尽力域，整机进 safe mode。
    void mark_component_failed(Component* component) noexcept {
        Linker::isolate(component, newly_failed_, failed_count_);
        newly_failed_size_.store(newly_failed_.size(), std::memory_order_release);

        predefined_msg_provider_->set_safe_mode(true);
    }

    /// 尽力域（spin 线程）。只读 RT 线程已发布的那一段前缀，容量固定所以不会被搬走。
    void report_newly_failed_components() {
        const auto published = newly_failed_size_.load(std::memory_order_acquire);
        while (reported_failed_count_ < published) {
            const auto index = newly_failed_[reported_failed_count_++];
            RCLCPP_ERROR(
                get_logger(),
                "Component [%s] threw from update() and has been isolated: its outputs were reset "
                "to the registered defaults and it will no longer be scheduled. System is in safe "
                "mode.",
                updating_order_[index]->get_component_name().c_str());
        }
    }

    static std::int64_t to_nanoseconds(hcs_sync::Timestamp timestamp) noexcept {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch())
            .count();
    }

    static std::uint32_t saturate_u32(std::int64_t value) noexcept {
        if (value <= 0)
            return 0;
        constexpr auto limit = static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max());
        return static_cast<std::uint32_t>(value < limit ? value : limit);
    }

    /// 建图、接线、排序全在 Linker 里（wiring.hpp），这里只剩"失败就打一次日志再抛"
    /// 和把缩进树打出来。Linker 不依赖 rclcpp，所以那部分能被单独拿去测。
    void init() {
        auto wiring = Linker::link(component_list_);
        if (!wiring) {
            RCLCPP_FATAL(get_logger(), "%s", wiring.error().message.c_str());
            throw std::runtime_error{wiring.error().message};
        }

        RCLCPP_INFO(get_logger(), "Calculating component dependencies");
        for (std::size_t position = 0; position < wiring->updating_order.size(); ++position) {
            std::string indent = "- ";
            for (std::uint32_t level = wiring->depth[position]; level-- > 0;)
                indent.append("    ");
            RCLCPP_INFO(
                get_logger(), "%s%s", indent.c_str(),
                wiring->updating_order[position]->get_component_name().c_str());
        }

        updating_order_ = std::move(wiring->updating_order);
        latch_list_ = std::move(wiring->latches);
        // 门铃列表由 Linker 收齐并定型：RT 线程只按这个固定列表遍历，
        // 运行期不许再增删（否则就是在周期域里改一个 vector）。
        tick_end_doorbells_ = std::move(wiring->tick_end_doorbells);
    }

    // 1 kHz 下 reporter 每 100 ms drain 一次，4096 条留了 4 秒余量；写满只丢样本，不影响控制。
    static constexpr std::size_t tick_sample_capacity_ = 4096;
    static constexpr std::size_t component_sample_capacity_ = 8192;

    rclcpp::executors::SingleThreadedExecutor& rcl_executor_;

    std::thread thread_;
    std::atomic<bool> running_{false};

    /// 控制线程在 promise.set_value() 之前写，主线程在 future.get() 之后读。
    std::string arm_summary_;

    hcs_utility::RtSampler sampler_{tick_sample_capacity_, component_sample_capacity_};
    std::unique_ptr<RtReporter> reporter_;
    rclcpp::TimerBase::SharedPtr failed_report_timer_;

#if HCS_EXECUTOR_COMPONENT_TIMING
    std::uint64_t component_timing_stride_ = 16;
#endif

    // 只被 RT 线程读写。
    std::uint32_t failed_count_ = 0;
    std::vector<std::uint32_t> newly_failed_;
    // RT 线程发布长度，尽力域按这个长度读前缀。
    std::atomic<std::size_t> newly_failed_size_{0};
    std::size_t reported_failed_count_ = 0;

    std::shared_ptr<PredefinedMsgProvider> predefined_msg_provider_;
    std::vector<std::shared_ptr<Component>> component_list_;

    std::vector<Component*> updating_order_;
    std::vector<LatchEntry> latch_list_;
    std::vector<hcs_utility::Doorbell*> tick_end_doorbells_;
};

} // namespace hcs_executor
