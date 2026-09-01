#pragma once

#include <fcntl.h>
#include <malloc.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <format>
#include <stdexcept>
#include <string>
#include <string_view>

#include "rmcs_utility/thread_config.hpp"

namespace rmcs_utility {

/**
 * 实时武装选项。语法与 ThreadConfig 一致：`key=value;key=value`
 *
 * | key               | 值     | 默认 | 说明                                              |
 * |-------------------|--------|------|---------------------------------------------------|
 * | `mlock`           | on/off | off  | mlockall(MCL_CURRENT|MCL_FUTURE)                  |
 * | `malloc`          | on/off | on   | mallopt: M_TRIM_THRESHOLD=-1, M_MMAP_MAX=0,       |
 * |                   |        |      | M_ARENA_MAX=1                                     |
 * | `timer_slack`     | 时长   | 1us  | prctl(PR_SET_TIMERSLACK)。0 表示不设置            |
 * | `prefault_stack`  | 字节数 | 512K | 提前把栈页踩出来。0 表示不做                      |
 * | `prefault_heap`   | 字节数 | 4M   | malloc + 踩页 + free，让 arena 长到位。0 表示不做 |
 * | `cpu_dma_latency` | on/off | off  | 持有 /dev/cpu_dma_latency 的 fd 并写 0，禁深      |
 * |                   |        |      | C-state。需要 root                                |
 * | `spin_guard`      | 时长   | 0    | 交给 sleep_until_precise 的尾部忙等长度           |
 *
 * 时长后缀 `ns` `us` `ms` `s`，无后缀按微秒；字节后缀 `K` `M`（1024 进制），无后缀按字节。
 * 后缀大小写不敏感。未知 key、语法错、值越界一律抛 std::invalid_argument。
 *
 * 默认值为什么是这套（以验收 §7 为准，不是设计表里那套）：
 * 不填 thread_config / realtime_config 的普通用户——没有 rtprio 权限、RLIMIT_MEMLOCK 只有
 * 几十 MB——必须能正常启动。需要特权的只有两项，所以这两项默认关：
 *   - `mlock`：RLIMIT_MEMLOCK 不够时 mlockall 直接 EPERM/ENOMEM，而本类是失败即抛，
 *     默认开它等于让普通用户开箱即挂。开之前先把 limits.conf 的 memlock 调够。
 *   - `cpu_dma_latency`：/dev/cpu_dma_latency 只有 root / CAP_SYS_ADMIN 能写。
 */
struct RealtimeArmOptions {
    bool lock_memory = false;
    bool tune_malloc = true;
    std::chrono::nanoseconds timer_slack = std::chrono::microseconds{1};
    std::size_t prefault_stack = 512u * 1024u;
    std::size_t prefault_heap = 4u * 1024u * 1024u;
    bool hold_cpu_dma_latency = false;
    std::chrono::nanoseconds spin_guard = std::chrono::nanoseconds{0};

    /// 踩栈上限。默认线程栈就是 8 MB，再往上只可能是笔误——而笔误的代价是踩爆栈。
    static constexpr std::size_t kMaxPrefaultStack = 8u * 1024u * 1024u;
    /// 堆预热上限，纯粹的笔误保护（`4G` 写成 `4M` 的反向）。
    static constexpr std::size_t kMaxPrefaultHeap = 1024u * 1024u * 1024u;
    /// timer slack 超过 1s 说明单位写错了。
    static constexpr std::chrono::nanoseconds kMaxTimerSlack = std::chrono::seconds{1};
    /// 忙等超过 10ms 不是调参是烧核——一个周期都放不下。
    static constexpr std::chrono::nanoseconds kMaxSpinGuard = std::chrono::milliseconds{10};

    /**
     * 解析武装选项。
     *
     * @param spec `key=value;key=value` 形式的选项串，空串表示全用默认值。
     * @throws std::invalid_argument 语法错误 / 未知 key / 重复 key / 值越界。
     */
    static RealtimeArmOptions parse(std::string_view spec) {
        const auto original_spec = spec;
        RealtimeArmOptions options;
        unsigned seen = 0;

        auto remaining_spec = trim(spec);
        while (true) {
            const auto separator = remaining_spec.find(';');
            const auto field = trim(remaining_spec.substr(0, separator));

            if (!field.empty()) {
                const auto equals = field.find('=');
                if (equals == std::string_view::npos)
                    throw_invalid_spec("missing '='", original_spec);

                const auto key = trim(field.substr(0, equals));
                const auto value = trim(field.substr(equals + 1));
                if (key.empty() || value.empty())
                    throw_invalid_spec("empty key or value", original_spec);

                assign_field(options, seen, key, value, original_spec);
            }

            if (separator == std::string_view::npos)
                break;
            remaining_spec.remove_prefix(separator + 1);
        }

        return options;
    }

private:
    enum : unsigned {
        kSeenLockMemory = 1u << 0,
        kSeenTuneMalloc = 1u << 1,
        kSeenTimerSlack = 1u << 2,
        kSeenPrefaultStack = 1u << 3,
        kSeenPrefaultHeap = 1u << 4,
        kSeenCpuDmaLatency = 1u << 5,
        kSeenSpinGuard = 1u << 6,
    };

    static std::string_view trim(std::string_view text) {
        while (!text.empty() && (text.front() == ' ' || text.front() == '\t'))
            text.remove_prefix(1);
        while (!text.empty() && (text.back() == ' ' || text.back() == '\t'))
            text.remove_suffix(1);
        return text;
    }

    [[noreturn]] static void throw_invalid_spec(std::string_view reason, std::string_view spec) {
        throw std::invalid_argument(
            std::format("Invalid realtime config spec ({}): \"{}\"", reason, spec));
    }

    static void assign_field(
        RealtimeArmOptions& options, unsigned& seen, std::string_view key, std::string_view value,
        std::string_view spec) {
        const auto mark_seen = [&seen, key, spec](unsigned bit) {
            if (seen & bit)
                throw_invalid_spec(std::format("duplicate key '{}'", key), spec);
            seen |= bit;
        };

        if (key == "mlock") {
            mark_seen(kSeenLockMemory);
            options.lock_memory = parse_bool(value, key, spec);
        } else if (key == "malloc") {
            mark_seen(kSeenTuneMalloc);
            options.tune_malloc = parse_bool(value, key, spec);
        } else if (key == "timer_slack") {
            mark_seen(kSeenTimerSlack);
            options.timer_slack = parse_duration(value, key, spec, kMaxTimerSlack);
        } else if (key == "prefault_stack") {
            mark_seen(kSeenPrefaultStack);
            options.prefault_stack = parse_bytes(value, key, spec, kMaxPrefaultStack);
        } else if (key == "prefault_heap") {
            mark_seen(kSeenPrefaultHeap);
            options.prefault_heap = parse_bytes(value, key, spec, kMaxPrefaultHeap);
        } else if (key == "cpu_dma_latency") {
            mark_seen(kSeenCpuDmaLatency);
            options.hold_cpu_dma_latency = parse_bool(value, key, spec);
        } else if (key == "spin_guard") {
            mark_seen(kSeenSpinGuard);
            options.spin_guard = parse_duration(value, key, spec, kMaxSpinGuard);
        } else {
            throw_invalid_spec(std::format("unknown key '{}'", key), spec);
        }
    }

    static bool parse_bool(std::string_view value, std::string_view key, std::string_view spec) {
        if (value == "on")
            return true;
        if (value == "off")
            return false;
        throw_invalid_spec(std::format("key '{}' expects on/off", key), spec);
    }

    /// 后缀匹配要求前面还剩至少一个字符，这样 `s` 单独出现会走"数字非法"而不是"零秒"。
    static bool has_suffix(std::string_view text, std::string_view suffix) {
        if (text.size() <= suffix.size())
            return false;
        const auto tail = text.substr(text.size() - suffix.size());
        for (std::size_t i = 0; i < suffix.size(); ++i) {
            const auto lowered = static_cast<char>(
                tail[i] >= 'A' && tail[i] <= 'Z' ? tail[i] - 'A' + 'a' : tail[i]);
            if (lowered != suffix[i])
                return false;
        }
        return true;
    }

    static std::int64_t
        parse_count(std::string_view text, std::string_view key, std::string_view spec) {
        text = trim(text);
        std::int64_t result = 0;
        const auto* begin = text.data();
        const auto* end = text.data() + text.size();
        const auto [ptr, error_code] = std::from_chars(begin, end, result);
        if (error_code != std::errc{} || ptr != end)
            throw_invalid_spec(std::format("invalid number for key '{}'", key), spec);
        if (result < 0)
            throw_invalid_spec(std::format("key '{}' must not be negative", key), spec);
        return result;
    }

    static std::chrono::nanoseconds parse_duration(
        std::string_view value, std::string_view key, std::string_view spec,
        std::chrono::nanoseconds max_value) {
        std::string_view number = value;
        std::int64_t scale = 1'000; // 无后缀按微秒
        if (has_suffix(value, "ns")) {
            number = value.substr(0, value.size() - 2);
            scale = 1;
        } else if (has_suffix(value, "us")) {
            number = value.substr(0, value.size() - 2);
            scale = 1'000;
        } else if (has_suffix(value, "ms")) {
            number = value.substr(0, value.size() - 2);
            scale = 1'000'000;
        } else if (has_suffix(value, "s")) {
            number = value.substr(0, value.size() - 1);
            scale = 1'000'000'000;
        }

        const auto count = parse_count(number, key, spec);
        if (count > max_value.count() / scale)
            throw_invalid_spec(std::format("key '{}' out of range", key), spec);
        return std::chrono::nanoseconds{count * scale};
    }

    static std::size_t parse_bytes(
        std::string_view value, std::string_view key, std::string_view spec,
        std::size_t max_value) {
        std::string_view number = value;
        std::int64_t scale = 1; // 无后缀按字节
        if (has_suffix(value, "k")) {
            number = value.substr(0, value.size() - 1);
            scale = 1024;
        } else if (has_suffix(value, "m")) {
            number = value.substr(0, value.size() - 1);
            scale = 1024 * 1024;
        }

        const auto count = parse_count(number, key, spec);
        if (count > static_cast<std::int64_t>(max_value) / scale)
            throw_invalid_spec(std::format("key '{}' out of range", key), spec);
        return static_cast<std::size_t>(count * scale);
    }
};

/**
 * RAII 武装当前线程。构造 = 武装，析构 = 释放 /dev/cpu_dma_latency 的 fd
 * （其余不回滚：这条线程随后就结束了，回滚没有意义）。
 *
 * 封盘点：构造函数返回的那一刻就是那道线。它之前随便分配，它之后一次 malloc 都是 bug。
 * 之所以要有这么一道线，是因为 mallopt / mlockall / 踩页三件事只对"已经存在的内存"
 * 和"已经定型的 arena"成立；线程进入周期循环后再去 malloc，就是在 RT 路径上开
 * brk/mmap 加缺页中断的盲盒。
 *
 * 为什么每一步失败即抛，而不是像改造前那样 WARN 一下照跑：
 * "失败只 WARN 然后照跑"的真实语义是"以为自己是 RT，其实不是"。它把一个配置错误
 * 变成一个看起来正常的日志，然后你会拿着这份日志去解释一条根本就不存在的实时线程的抖动，
 * 查调度、查中断、查节能，查几天，而真相是 setschedparam 在第一秒就返回了 EPERM。
 * 宁可起不来：起不来的原因写在异常消息里，一眼就能看见。
 */
class RealtimeArm {
public:
    /**
     * 武装顺序：绑核/命名 → timer slack → malloc → mlock → 踩栈 → 踩堆 → **升优先级** → C-state。
     *
     * 升优先级放在内存武装之后，不是风格问题，是实测出来的：
     * `mlockall(MCL_CURRENT)` 要把整个 ROS 进程的页走一遍并锁住，本机实测 **35 ms**。
     * 如果先升到 SCHED_FIFO 90 再做（改造前的顺序），这 35 ms 就是在隔离核上以最高优先级
     * 独占——而 `sched_rt_runtime_us` 已经被调优服务设成 -1，没有任何东西能打断它。
     * 对照实测：一条 FIFO 80 的线程先在同一个核上跑着，executor 启动时它被饿住
     *   mlock=on            → 36.4 ms
     *   mlock=off（只预热）  → 1.5 ms
     *   武装完之后才起       → 11 us
     * 接上集成层之后传输线程是先起的，这 35 ms 会实实在在落在它头上。
     *
     * 绑核仍然排在最前：踩栈踩堆要踩在这条线程真正会跑的那个核上。
     *
     * @throws std::runtime_error 任何一步失败（消息里带 strerror(errno)）。
     */
    RealtimeArm(const ThreadConfig& thread_config, const RealtimeArmOptions& options)
        : options_(options) {
        // 放第一条：非 PREEMPT_RT 内核上，**尾部延迟没有任何上界保证**。
        // 后面那一串旋钮全开也只能改善 p50/p99，改善不了 max。
        // 把它写进摘要，是为了让"为什么 max 有 1 ms"这个问题一眼有答案，
        // 而不是每次都重新去查一遍机器。
        append_summary(has_realtime_kernel() ? "kernel=preempt_rt" : "kernel=NOT_rt");
        arm_thread_identity(thread_config);
        arm_timer_slack();
        arm_malloc();
        arm_memory_lock();
        arm_prefault_stack();
        arm_prefault_heap();
        // 封盘之后才升优先级：上面每一步都是重活，不该以 RT 优先级做。
        arm_thread_scheduling(thread_config);
        // 放最后：只有这一步会留下需要析构的资源，而构造抛异常时析构不会跑，
        // 所以它自己的失败路径负责关掉半开的 fd。
        arm_cpu_dma_latency();
        // spin_guard 不是这里执行的动作（由 sleep_until_precise 用），但它决定这条线程烧不烧核，
        // 属于"这条线程到底被配成了什么样"的一部分，所以照样进清单。
        append_summary(std::format("spin_guard={}ns", options_.spin_guard.count()));
    }

    ~RealtimeArm() {
        // fd 一关，写进去的 0us 延迟要求立刻失效，所以整个线程生命周期内必须持有。
        if (cpu_dma_latency_fd_ >= 0)
            ::close(cpu_dma_latency_fd_);
    }

    RealtimeArm(const RealtimeArm&) = delete;
    RealtimeArm& operator=(const RealtimeArm&) = delete;
    RealtimeArm(RealtimeArm&&) = delete;
    RealtimeArm& operator=(RealtimeArm&&) = delete;

    [[nodiscard]] const RealtimeArmOptions& options() const noexcept { return options_; }

    /**
     * 武装结果清单，供启动日志打印（构造期产生，允许分配）。
     *
     * 没开的项也会以 `=off` 出现——这行日志存在的意义就是回答"到底哪些真的生效了"，
     * 省略掉关着的项只会让人重新猜。
     */
    [[nodiscard]] const std::string& summary() const noexcept { return summary_; }

private:
    /// 内核是不是 PREEMPT_RT。realtime_tools 有同名函数，但为读一个字节引一个依赖不值。
    [[nodiscard]] static bool has_realtime_kernel() noexcept {
        const int fd = ::open("/sys/kernel/realtime", O_RDONLY | O_CLOEXEC);
        if (fd < 0)
            return false; // 文件不存在就是普通内核，不是错误
        char value = 0;
        const auto bytes_read = ::read(fd, &value, 1);
        ::close(fd);
        return bytes_read == 1 && value == '1';
    }

    [[noreturn]] static void throw_failure(std::string_view what) {
        throw std::runtime_error(std::format("RealtimeArm: {}", what));
    }

    void append_summary(std::string_view item) {
        if (!summary_.empty())
            summary_ += "; ";
        summary_ += item;
    }

    static std::string_view policy_name(int policy) {
        switch (policy) {
        case SCHED_OTHER: return "other";
        case SCHED_BATCH: return "batch";
        case SCHED_IDLE: return "idle";
        case SCHED_FIFO: return "fifo";
        case SCHED_RR: return "rr";
        default: return "unknown";
        }
    }

    static std::string format_cpu_set(const cpu_set_t& cpus) {
        std::string result;
        int range_first = -1;
        for (int cpu = 0; cpu <= CPU_SETSIZE; ++cpu) {
            const bool is_set = cpu < CPU_SETSIZE && CPU_ISSET(cpu, &cpus);
            if (is_set && range_first < 0)
                range_first = cpu;
            if (is_set || range_first < 0)
                continue;

            if (!result.empty())
                result += ',';
            if (range_first == cpu - 1)
                result += std::format("{}", range_first);
            else
                result += std::format("{}-{}", range_first, cpu - 1);
            range_first = -1;
        }
        return result.empty() ? std::string{"none"} : result;
    }

    void arm_thread_identity(const ThreadConfig& thread_config) {
        if (const auto applied = thread_config.apply_identity_to_current_thread(); !applied)
            throw_failure(applied.error());

        std::string fields;
        const auto add = [&fields](std::string_view item) {
            if (!fields.empty())
                fields += ',';
            fields += item;
        };
        if (thread_config.name())
            add(std::format("name={}", *thread_config.name()));
        if (thread_config.policy())
            add(std::format("policy={}", policy_name(*thread_config.policy())));
        if (thread_config.priority())
            add(std::format("priority={}", *thread_config.priority()));
        if (thread_config.nice())
            add(std::format("nice={}", *thread_config.nice()));
        if (thread_config.cpus())
            add(std::format("cpus={}", format_cpu_set(*thread_config.cpus())));

        const std::string_view shown =
            fields.empty() ? std::string_view{"unchanged"} : std::string_view{fields};
        append_summary(std::format("thread[{}]", shown));
    }

    /// 只做调度策略那一半。摘要已经在 arm_thread_identity 里打全了。
    void arm_thread_scheduling(const ThreadConfig& thread_config) {
        if (const auto applied = thread_config.apply_scheduling_to_current_thread(); !applied)
            throw_failure(applied.error());
    }

    void arm_timer_slack() {
        // 0 在 PR_SET_TIMERSLACK 里的含义是"恢复继承来的默认值"，不是"零 slack"，
        // 所以 0 只能当"不设置"讲。
        if (options_.timer_slack <= std::chrono::nanoseconds{0}) {
            append_summary("timer_slack=off");
            return;
        }

        const auto slack_ns = static_cast<unsigned long>(options_.timer_slack.count());
        if (::prctl(PR_SET_TIMERSLACK, slack_ns) != 0)
            throw_failure(
                std::format("prctl(PR_SET_TIMERSLACK, {}) failed: {}", slack_ns,
                    std::strerror(errno)));
        append_summary(std::format("timer_slack={}ns", slack_ns));
    }

    void arm_malloc() {
        if (!options_.tune_malloc) {
            append_summary("malloc=off");
            return;
        }

        // mallopt 不设置 errno，所以这里没有 strerror 可带；返回 0 就是唯一的信息。
        const auto apply = [](int parameter, int value, std::string_view name) {
            if (::mallopt(parameter, value) == 0)
                throw_failure(std::format("mallopt({}, {}) failed", name, value));
        };
        apply(M_TRIM_THRESHOLD, -1, "M_TRIM_THRESHOLD"); // 不把空闲堆还给内核
        apply(M_MMAP_MAX, 0, "M_MMAP_MAX");              // 大块也走 arena，别去 mmap
        apply(M_ARENA_MAX, 1, "M_ARENA_MAX");            // 单 arena，杜绝按线程新建
        append_summary("malloc[M_TRIM_THRESHOLD=-1,M_MMAP_MAX=0,M_ARENA_MAX=1]");
    }

    void arm_memory_lock() {
        if (!options_.lock_memory) {
            append_summary("mlock=off");
            return;
        }

        if (::mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
            throw_failure(
                std::format(
                    "mlockall(MCL_CURRENT|MCL_FUTURE) failed: {}"
                    "（RLIMIT_MEMLOCK 不够时就是这个错，调 limits.conf 的 memlock，"
                    "或者把 mlock 关掉）",
                    std::strerror(errno)));
        append_summary("mlock=on[MCL_CURRENT|MCL_FUTURE]");
    }

    void arm_prefault_stack() {
        if (options_.prefault_stack == 0) {
            append_summary("prefault_stack=off");
            return;
        }
        if (options_.prefault_stack > RealtimeArmOptions::kMaxPrefaultStack)
            throw_failure(
                std::format(
                    "prefault_stack {} exceeds the {} byte cap", options_.prefault_stack,
                    RealtimeArmOptions::kMaxPrefaultStack));

        // 硬上限挡不住"这条线程的栈本来就小"这种情况，所以能问到真实栈大小就再兜一层。
        const auto stack_size = query_stack_size();
        if (stack_size != 0 && options_.prefault_stack > stack_size - stack_size / 4)
            throw_failure(
                std::format(
                    "prefault_stack {} exceeds 3/4 of this thread's {} byte stack",
                    options_.prefault_stack, stack_size));

        prefault_stack_pages(options_.prefault_stack);
        append_summary(std::format("prefault_stack={}B", options_.prefault_stack));
    }

    void arm_prefault_heap() {
        if (options_.prefault_heap == 0) {
            append_summary("prefault_heap=off");
            return;
        }

        auto* block = static_cast<volatile unsigned char*>(std::malloc(options_.prefault_heap));
        if (block == nullptr)
            throw_failure(
                std::format(
                    "malloc({}) for heap prefault failed: {}", options_.prefault_heap,
                    std::strerror(errno)));

        touch_pages(block, options_.prefault_heap);
        // free 之后 arena 仍然是长好的（M_TRIM_THRESHOLD=-1 保证不还给内核），
        // 这正是预热的意义：后面万一真有人 malloc，也是纯用户态的取块。
        std::free(const_cast<unsigned char*>(block));
        append_summary(std::format("prefault_heap={}B", options_.prefault_heap));
    }

    void arm_cpu_dma_latency() {
        if (!options_.hold_cpu_dma_latency) {
            append_summary("cpu_dma_latency=off");
            return;
        }

        const int fd = ::open("/dev/cpu_dma_latency", O_WRONLY | O_CLOEXEC);
        if (fd < 0)
            throw_failure(
                std::format(
                    "open(/dev/cpu_dma_latency) failed: {}"
                    "（需要 root 或 CAP_SYS_ADMIN，或把 cpu_dma_latency 关掉）",
                    std::strerror(errno)));

        const std::int32_t target_latency_us = 0;
        const auto written = ::write(fd, &target_latency_us, sizeof(target_latency_us));
        if (written != static_cast<ssize_t>(sizeof(target_latency_us))) {
            const auto write_error = errno;
            ::close(fd);
            throw_failure(
                std::format(
                    "write(/dev/cpu_dma_latency, 0) failed: {}"
                    "（需要 root 或 CAP_SYS_ADMIN，或把 cpu_dma_latency 关掉）",
                    std::strerror(write_error)));
        }

        cpu_dma_latency_fd_ = fd;
        append_summary("cpu_dma_latency=on[0us]");
    }

    static std::size_t query_stack_size() noexcept {
        pthread_attr_t attributes;
        if (::pthread_getattr_np(::pthread_self(), &attributes) != 0)
            return 0;

        void* stack_address = nullptr;
        std::size_t stack_size = 0;
        const int error_code = ::pthread_attr_getstack(&attributes, &stack_address, &stack_size);
        ::pthread_attr_destroy(&attributes);
        return error_code == 0 ? stack_size : 0;
    }

    /**
     * 逐块 alloca 往下探，把栈页真正 fault 进来。
     *
     * alloca 的分配到函数返回才一起收回，所以循环里的每一块都是新的栈空间，栈指针一路下降；
     * 探完返回后栈指针弹回，但页已经映射且（开了 mlock 的话）锁住，之后再用不会缺页。
     */
    static void prefault_stack_pages(std::size_t bytes) noexcept {
        constexpr std::size_t kChunk = 64u * 1024u;

        std::size_t remaining = bytes;
        while (remaining != 0) {
            const std::size_t chunk = remaining < kChunk ? remaining : kChunk;
            touch_pages(static_cast<volatile unsigned char*>(__builtin_alloca(chunk)), chunk);
            remaining -= chunk;
        }
    }

    /// 必须是 volatile 写：memset 一个之后就没人读、马上又要 free / 出作用域的块，
    /// 会被当成死存储直接删掉，页一个都碰不到。
    static void touch_pages(volatile unsigned char* block, std::size_t bytes) noexcept {
        const long queried_page_size = ::sysconf(_SC_PAGESIZE);
        const auto page_size =
            queried_page_size > 0 ? static_cast<std::size_t>(queried_page_size) : 4096u;

        for (std::size_t offset = 0; offset < bytes; offset += page_size)
            block[offset] = 0;
        block[bytes - 1] = 0;
    }

    RealtimeArmOptions options_;
    std::string summary_;
    int cpu_dma_latency_fd_ = -1;
};

} // namespace rmcs_utility
