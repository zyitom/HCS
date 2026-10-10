#pragma once

#include <expected>
#include <format>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "hcs_base/thread/detail/spec_parse.hpp"

namespace hcs_utility {

/**
 * Parsed thread configuration.
 *
 * Supported fields in `spec` are `name`, `cpus`, `policy`, `priority`, and `nice`.
 * Format: `key=value;key=value;...`
 * Empty specs are allowed and represent a no-op configuration.
 * Examples:
 * - `name=hcs-ctrl;cpus=3;policy=fifo;priority=80`
 * - `cpus=4-7;policy=other;nice=-5`
 */
class ThreadConfig {
public:
    /**
     * Parse a thread configuration spec.
     *
     * @param spec Thread configuration spec string.
     * @throws std::invalid_argument If the spec is malformed or contains invalid values.
     */
    explicit ThreadConfig(std::string_view spec) {
        spec = detail::trim(spec);
        detail::for_each_kv_field(
            kSpecKind, spec, [this, spec](std::string_view key, std::string_view value) {
                assign_field(key, value, spec);
            });
        validate(spec);
    }

    /**
     * Parse a thread configuration spec and fill in a default thread name when `name` is unset.
     *
     * @param spec Thread configuration spec string.
     * @param default_name Fallback name used only when `spec` does not configure `name`.
     * @throws std::invalid_argument If the spec is malformed, contains invalid values, or
     * `default_name` is invalid when used.
     */
    ThreadConfig(std::string_view spec, std::string_view default_name)
        : ThreadConfig(spec) {
        if (name_)
            return;
        if (const auto invalid_reason = check_name(default_name))
            throw std::invalid_argument(std::string(*invalid_reason));
        name_ = std::string{default_name};
    }

    [[nodiscard]] auto name() const -> const std::optional<std::string>& { return name_; }
    [[nodiscard]] auto cpus() const -> const std::optional<cpu_set_t>& { return cpus_; }
    [[nodiscard]] auto policy() const -> const std::optional<int>& { return policy_; }
    [[nodiscard]] auto priority() const -> const std::optional<int>& { return priority_; }
    [[nodiscard]] auto nice() const -> const std::optional<int>& { return nice_; }

    /**
     * Apply the stored configuration to the calling thread.
     *
     * Identity (name, affinity) is applied before scheduling (policy, priority, nice), so the
     * thread is already on its target CPU by the time it becomes realtime. The reverse order
     * lets an already-realtime thread run briefly on whatever CPU it happened to start on.
     *
     * This operation is not atomic. If a later step fails, earlier changes may already have been
     * applied to the current thread.
     *
     * @return `std::expected<void, std::string>{}` on success, or an error string describing the
     * failing runtime step.
     */
    auto apply_to_current_thread() const -> std::expected<void, std::string> {
        if (auto applied = apply_identity_to_current_thread(); !applied)
            return applied;
        return apply_scheduling_to_current_thread();
    }

    /**
     * Apply only the cheap, non-privileged half: thread name and CPU affinity.
     *
     * Split out for RealtimeArm, which must do its memory arming (mlockall, prefault) *before*
     * raising priority: mlockall on a large process takes tens of milliseconds, and at
     * SCHED_FIFO 90 on an isolated core that starves every other realtime thread there for
     * exactly that long. Pinning first is still worth doing, so the prefaulted stack pages and
     * the arena are warmed on the CPU the thread will actually run on.
     */
    auto apply_identity_to_current_thread() const -> std::expected<void, std::string> {
        const auto current_thread = pthread_self();

        if (cpus_) {
            const int error_code =
                pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &*cpus_);
            if (error_code != 0)
                return std::unexpected(
                    std::format("Failed to set thread affinity: {}", std::strerror(error_code)));
        }

        if (name_) {
            const int error_code = pthread_setname_np(current_thread, name_->c_str());
            if (error_code != 0)
                return std::unexpected(
                    std::format("Failed to set thread name: {}", std::strerror(error_code)));
        }

        return {};
    }

    /// 另一半：调度策略、实时优先级、nice。为什么它排在最后，见上面。
    auto apply_scheduling_to_current_thread() const -> std::expected<void, std::string> {
        if (policy_) {
            sched_param param{};
            param.sched_priority = priority_.value_or(0);
            const int error_code = pthread_setschedparam(pthread_self(), *policy_, &param);
            if (error_code != 0) {
                return std::unexpected(
                    std::format(
                        "Failed to set thread scheduling policy: {}", std::strerror(error_code)));
            }
        }

        if (nice_) {
            errno = 0;
            const auto tid = static_cast<id_t>(syscall(SYS_gettid));
            if (setpriority(PRIO_PROCESS, tid, *nice_) != 0)
                return std::unexpected(
                    std::format("Failed to set thread nice value: {}", std::strerror(errno)));
        }

        return {};
    }

private:
    /// 报错里的规格类别名,顺便把 this 捕不进 static 的问题绕掉。
    static constexpr std::string_view kSpecKind = "thread config";

    [[noreturn]] static void throw_invalid_spec(std::string_view reason, std::string_view spec) {
        detail::throw_invalid_spec(kSpecKind, reason, spec);
    }

    static int parse_int(std::string_view value, std::string_view key, std::string_view spec) {
        const auto parsed = detail::parse_integer<int>(value);
        if (!parsed)
            throw_invalid_spec(std::format("invalid integer for key '{}'", key), spec);
        return *parsed;
    }

    static auto check_name(std::string_view name) -> std::optional<std::string> {
        if (name.empty())
            return "Thread name must not be empty";
        if (name.size() > 15)
            return std::format("Thread name exceeds 15 characters: \"{}\"", name);
        return std::nullopt;
    }

    /// 字段既是值也是"见过没有"的记录(optional)—— 重复键检测不用另设标记位。
    void assign_field(std::string_view key, std::string_view value, std::string_view spec) {
        if (key == "name") {
            if (name_)
                throw_invalid_spec("duplicate key 'name'", spec);
            if (const auto invalid_reason = check_name(value))
                throw_invalid_spec(*invalid_reason, spec);
            name_ = std::string{value};
            return;
        }
        if (key == "cpus") {
            if (cpus_)
                throw_invalid_spec("duplicate key 'cpus'", spec);
            cpus_ = parse_cpu_list(value, spec);
            return;
        }
        if (key == "policy") {
            if (policy_)
                throw_invalid_spec("duplicate key 'policy'", spec);
            policy_ = parse_policy(value, spec);
            return;
        }
        if (key == "priority") {
            if (priority_)
                throw_invalid_spec("duplicate key 'priority'", spec);
            priority_ = parse_int(value, key, spec);
            return;
        }
        if (key == "nice") {
            if (nice_)
                throw_invalid_spec("duplicate key 'nice'", spec);
            nice_ = parse_int(value, key, spec);
            return;
        }
        throw_invalid_spec(std::format("unknown key '{}'", key), spec);
    }

    static auto parse_policy(std::string_view value, std::string_view spec) -> int {
        if (value == "other")
            return SCHED_OTHER;
        if (value == "batch")
            return SCHED_BATCH;
        if (value == "idle")
            return SCHED_IDLE;
        if (value == "fifo")
            return SCHED_FIFO;
        if (value == "rr")
            return SCHED_RR;
        throw_invalid_spec("unknown policy", spec);
    }

    static auto parse_cpu_list(std::string_view value, std::string_view spec) -> cpu_set_t {
        cpu_set_t cpus;
        CPU_ZERO(&cpus);

        bool has_any_cpu = false;
        while (true) {
            const auto separator = value.find(',');
            const auto token = detail::trim(value.substr(0, separator));
            if (token.empty())
                throw_invalid_spec("empty cpu list token", spec);

            const auto dash = token.find('-');
            if (dash == std::string_view::npos) {
                const int cpu = parse_int(token, "cpus", spec);
                set_cpu_range(cpus, cpu, cpu, spec);
            } else {
                if (token.find('-', dash + 1) != std::string_view::npos)
                    throw_invalid_spec("invalid cpu range token", spec);
                const int first_cpu = parse_int(detail::trim(token.substr(0, dash)), "cpus", spec);
                const int last_cpu = parse_int(detail::trim(token.substr(dash + 1)), "cpus", spec);
                set_cpu_range(cpus, first_cpu, last_cpu, spec);
            }

            has_any_cpu = true;
            if (separator == std::string_view::npos)
                break;
            value.remove_prefix(separator + 1);
        }

        if (!has_any_cpu)
            throw_invalid_spec("empty cpu list", spec);
        return cpus;
    }

    /// 这个内核配置了多少个 CPU。故意**不**用 sched_getaffinity：在带 isolcpus 的机器上，
    /// 被隔离的 CPU 不在默认掩码里，而绑到其中一个上恰恰是我们要允许的事。
    static int configured_cpu_count() {
        const long count = ::sysconf(_SC_NPROCESSORS_CONF);
        return count > 0 ? static_cast<int>(count) : 0;
    }

    static void set_cpu_range(cpu_set_t& cpus, int first_cpu, int last_cpu, std::string_view spec) {
        if (first_cpu < 0 || last_cpu < 0 || first_cpu > last_cpu)
            throw_invalid_spec("invalid cpu range", spec);
        if (last_cpu >= CPU_SETSIZE)
            throw_invalid_spec("cpu index exceeds CPU_SETSIZE", spec);

        // 掩码里写了这台机器没有的 CPU，以前是不出声地通过的：只要列出来的 CPU 里有一个存在，
        // pthread_setaffinity_np 就成功，内核把其余的丢掉。给 20 核机器写的配置就是这样在 8 核
        // 机器上"照常工作"的，而实际意思已经悄悄变了。
        if (const int configured = configured_cpu_count(); configured > 0 && last_cpu >= configured)
            throw_invalid_spec(
                std::format(
                    "cpu index {} is beyond this machine's {} CPUs", last_cpu, configured),
                spec);

        for (int cpu = first_cpu; cpu <= last_cpu; ++cpu)
            CPU_SET(cpu, &cpus);
    }

    void validate(std::string_view spec) const {
        if (priority_ && !policy_)
            throw_invalid_spec("priority requires policy", spec);

        if (policy_) {
            const bool is_realtime = *policy_ == SCHED_FIFO || *policy_ == SCHED_RR;
            if (is_realtime) {
                if (!priority_)
                    throw_invalid_spec("realtime policy requires priority", spec);
                if (nice_)
                    throw_invalid_spec("realtime policy cannot use nice", spec);

                const int min_priority = sched_get_priority_min(*policy_);
                const int max_priority = sched_get_priority_max(*policy_);
                if (*priority_ < min_priority || *priority_ > max_priority)
                    throw_invalid_spec("priority out of range for policy", spec);
            } else if (priority_) {
                throw_invalid_spec("non-realtime policy cannot use priority", spec);
            }
        }

        if (nice_ && (*nice_ < -20 || *nice_ > 19))
            throw_invalid_spec("nice out of range", spec);
    }

    std::optional<std::string> name_;
    std::optional<cpu_set_t> cpus_;
    std::optional<int> policy_;
    std::optional<int> priority_;
    std::optional<int> nice_;
};

} // namespace hcs_utility
