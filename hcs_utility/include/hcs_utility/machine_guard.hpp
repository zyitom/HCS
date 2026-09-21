#pragma once

#include <dirent.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <format>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace hcs_utility {

/**
 * 机器护栏：启动前读一遍 USB 实时路径依赖的内核状态，把"调优没生效/被重启冲掉"在
 * 第一时间报出来，而不是等它变成一条 26.7 ms 的 rtt 尾再查三天。
 *
 * 全部只读（sysfs/procfs + access()），不需要 root、不修改任何状态。需要修改的项由
 * hcs_bringup/tools/hcs_rt_tune.sh 在开机时以 root 落地；本护栏负责发现"它没生效"。
 *
 * 判据全部来自 HOST_TUNING.md 的实测，逐条标了出处：
 *   - sched_rt_runtime_us != -1：RT 线程被周期性掐死，max ~50 ms 级（10.10.4 第一行）
 *   - xHCI 中断线程掉回 FIFO 50：多板共享时 max ~1 ms（第 8 节）；sysfs/chrt 重启即回退（10.8）
 *   - xHCI 中断线程所在核的 C1 开着：EP0 p50 +22us、生产 p99 +4~18us（10.9.4）
 *   - 中断线程落在降频核（E 核/LP E 核）：p50 +21us（2.2；用 cpuinfo_max_freq 判断，机器无关）
 *   - 5321 枚举成 12M 全速：p50 +48us，版本串完全看不出来，唯一信号是 speed 文件（10.1）
 *   - 总线上有非 runtime PID 的 a511 设备：停在 DFU bootloader 的板子滞留总线，
 *     会造成微帧量化的 300us -> 1.4ms 延迟簇，纯"被枚举"即可致病（11.1）
 *   - irqbalance 在跑：运行期挪中断制造抖动（第 3 节）
 *   - IMOD 读数：中断聚合当前值。它被 imod 工具改过后驱动不会改回，会无声跨重启保持（11.3）
 *
 * 线程安全：run() 无状态。调用方决定打日志的方式（executor 里用 std::call_once 包一层）。
 */
class MachineGuard {
public:
    enum class Level { kOk, kInfo, kWarn, kCritical };
    struct Finding {
        Level level;
        std::string text;
    };

    /// 跑全部检查。每项检查自身失败（文件读不到等）降级为 kInfo，不抛异常。
    static std::vector<Finding> run() {
        std::vector<Finding> findings;
        check_rt_runtime(findings);
        check_xhci_irq_threads(findings);
        check_irqbalance(findings);
        check_boards(findings);
        check_imod(findings);
        return findings;
    }

    /// 有 kCritical 时返回 1，适合直接当 exit code。
    static int worst_level(const std::vector<Finding>& findings) {
        int worst = static_cast<int>(Level::kOk);
        for (const auto& finding : findings)
            worst = std::max(worst, static_cast<int>(finding.level));
        return worst;
    }

private:
    [[nodiscard]] static std::optional<std::string> read_line(std::string_view path) {
        std::ifstream file{std::string{path}};
        if (!file)
            return std::nullopt;
        std::string line;
        std::getline(file, line);
        while (!line.empty()
               && (line.back() == '\n' || line.back() == '\r' || line.back() == ' '))
            line.pop_back();
        return line;
    }

    static void add(
        std::vector<Finding>& findings, Level level, std::string text) {
        findings.push_back(Finding{level, std::move(text)});
    }

    // ── RT throttling ─────────────────────────────────────────────────────

    static void check_rt_runtime(std::vector<Finding>& findings) {
        const auto value = read_line("/proc/sys/kernel/sched_rt_runtime_us");
        if (!value) {
            add(findings, Level::kInfo, "sched_rt_runtime_us unreadable");
            return;
        }
        if (*value == "-1")
            add(findings, Level::kOk, "sched_rt_runtime_us=-1 (RT throttling off)");
        else
            add(
                findings, Level::kCritical,
                std::format(
                    "sched_rt_runtime_us={} (not -1): RT threads can be throttled, "
                    "measured ~50 ms stalls (HOST_TUNING 10.10.4). "
                    "Fix: sysctl -w kernel.sched_rt_runtime_us=-1",
                    *value));
    }

    // ── xHCI 中断线程：核 + FIFO 优先级 + 该核 C1 ────────────────────────────

    struct IrqThread {
        int pid = 0;
        std::string irq;
        int rt_priority = 0;
        int policy = 0;
        std::string core;
    };

    static std::optional<IrqThread> parse_irq_thread(int pid, const std::string& comm) {
        // comm 形如 "irq/135-xhci_hcd"，取 IRQ 号。
        const auto dash = comm.find('/');
        const auto end = comm.find('-');
        if (dash == std::string::npos || end == std::string::npos || end <= dash + 1)
            return std::nullopt;
        IrqThread thread;
        thread.irq = comm.substr(dash + 1, end - dash - 1);
        thread.pid = pid;

        // /proc/<pid>/stat：comm 含空格且被括号包裹，从最后一个 ')' 之后按空格切。
        // 注意 ')' 后有一个前导空格，必须先跳过，否则全部字段错位一位。
        // rt_priority 是第 40 个字段（')' 后 token 下标 37），policy 第 41 个（下标 38）。
        const auto stat = read_line(std::format("/proc/{}/stat", pid));
        if (!stat)
            return std::nullopt;
        const auto close_paren = stat->rfind(')');
        if (close_paren == std::string::npos)
            return std::nullopt;
        std::vector<std::string> tokens;
        size_t position = stat->find_first_not_of(' ', close_paren + 1);
        while (position != std::string::npos && position < stat->size()) {
            const auto next = stat->find(' ', position);
            tokens.emplace_back(stat->substr(position, next - position));
            if (next == std::string::npos)
                break;
            position = next + 1;
        }
        if (tokens.size() < 39)
            return std::nullopt;
        thread.rt_priority = std::atoi(tokens[37].c_str());
        thread.policy = std::atoi(tokens[38].c_str());
        return thread;
    }

    /// 有效亲和（MSI 向量硬件上只能指向一个核；写的是允许集，生效的才是真的，2.2 节）。
    static std::string irq_effective_core(const std::string& irq) {
        if (const auto value = read_line(std::format("/proc/irq/{}/effective_affinity_list", irq)))
            return *value;
        if (const auto value = read_line(std::format("/proc/irq/{}/smp_affinity_list", irq)))
            return *value;
        return "?";
    }

    static std::optional<unsigned long> cpu_max_freq(const std::string& core) {
        const auto value =
            read_line(std::format("/sys/devices/system/cpu/cpu{}/cpufreq/cpuinfo_max_freq", core));
        if (!value)
            return std::nullopt;
        return std::strtoul(value->c_str(), nullptr, 10);
    }

    /// 该核名为 C1* 的 idle state 是否被 disable。返回：nullopt = 查不到（机器无关，跳过）。
    static std::optional<bool> c1_disabled(const std::string& core) {
        const std::string base = std::format("/sys/devices/system/cpu/cpu{}/cpuidle", core);
        DIR* directory = opendir(base.c_str());
        if (directory == nullptr)
            return std::nullopt;
        std::optional<bool> result;
        while (const auto* entry = readdir(directory)) {
            const std::string name = entry->d_name;
            if (name.find("state") != 0)
                continue;
            const auto state_name = read_line(base + "/" + name + "/name");
            if (!state_name || state_name->compare(0, 2, "C1") != 0)
                continue;
            result = read_line(base + "/" + name + "/disable").value_or("0") == "1";
            break;
        }
        closedir(directory);
        return result;
    }

    /// 唤醒代价检查。代价的真正来源是 C1E（MSR_POWER_CTL 0x1FC bit1，进 C1 时降
    /// 压降频、退出爬坡，实测 15-20 us；HOST_TUNING 10.9.3），不是 C1 本身：
    /// 2026-09-15 实测全机关 C1E 后，"C1 正常睡眠"与"禁 C1 强制 POLL 空转"延迟
    /// 持平（EP0 p50 73.5 vs 72.7），而空转白烧一个核——正确形态是 C1E=0 且
    /// C1 正常。C1E 只能经 /dev/cpu/N/msr 读（root），读不到就把结论降级为 info。
    static void check_idle_cost(std::vector<Finding>& findings, const std::string& core) {
        int msr_fd = open("/dev/cpu/0/msr", O_RDONLY | O_CLOEXEC);
        const bool msr_readable = msr_fd >= 0;
        if (msr_fd >= 0)
            close(msr_fd);

        if (msr_readable) {
            // root / 已放开权限：直接读 C1E 位，给出准确结论。
            const int fd = open("/dev/cpu/0/msr", O_RDONLY | O_CLOEXEC);
            uint64_t value = 0;
            const bool read_ok =
                fd >= 0 && pread(fd, &value, sizeof(value), 0x1FC) == sizeof(value);
            if (fd >= 0)
                close(fd);
            if (!read_ok) {
                add(findings, Level::kInfo, "MSR 0x1FC read failed; C1E state unknown");
                return;
            }
            if ((value >> 1) & 1U)
                add(
                    findings, Level::kWarn,
                    std::format(
                        "C1E is ON (MSR 0x1FC bit1): C1 wake costs 15-20 us on every core, "
                        "measured EP0 p50 +22 us (HOST_TUNING 10.9.3). Fix: sudo tools/c1e off"));
            else
                add(findings, Level::kOk, "C1E off machine-wide (cheap C1 wake)");
            return;
        }

        // 无 root：C1 状态仍可读。C1 被禁是"用空转换延迟"的旧方案——有效但不省；
        // C1 开着则无法区分"C1E 已关（好）"和"C1E 开着（+22 us）"。
        const auto disabled = c1_disabled(core);
        if (disabled && *disabled) {
            add(
                findings, Level::kInfo,
                std::format(
                    "cpu{}: C1 disabled (POLL-spin trade, latency OK but the core never "
                    "idles); prefer C1E off + C1 enabled (tools/c1e)", core));
        } else {
            add(
                findings, Level::kInfo,
                std::format(
                    "cpu{}: C1 enabled; C1E state unverifiable without root -- verify with "
                    "`sudo tools/c1e` (C1E=0 is the desired state, HOST_TUNING 10.9.3)", core));
        }
    }

    /// xHCI 中断核上的硬中断邻居。硬中断不受 SCHED_FIFO 保护，会直接抢占中断线程；
    /// nvme0q* 是 blk-mq 的 per-CPU 亲和（sysfs 只读），挪不动且只在磁盘 IO 时发声，
    /// 单列为 info。其余设备中断与 xHCI 共核即警告（驱动会在运行期把中断挪回来，
    /// HOST_TUNING 1.3 的失效模式，所以要每次启动都查）。
    static void check_irq_neighbors(
        std::vector<Finding>& findings, const std::string& core) {
        std::ifstream file{"/proc/interrupts"};
        if (!file)
            return;
        std::string header;
        std::getline(file, header);
        // 头行 " CPU0 CPU1 ..."：core 所在的计数列（0 基）与总列数。
        int column = -1;
        int total = 0;
        {
            auto stream = std::istringstream{header};
            std::string token;
            while (stream >> token) {
                if (token == std::format("cpu{}", core) || token == std::format("CPU{}", core))
                    column = total;
                ++total;
            }
        }
        if (column < 0)
            return;
        std::vector<std::string> neighbors;
        std::vector<std::string> unmovable;
        std::string line;
        while (std::getline(file, line)) {
            const auto colon = line.find(':');
            if (colon == std::string::npos || colon == 0)
                continue;
            const auto first_digit = line.find_first_not_of(' ');
            const auto irq = line.substr(first_digit, colon - first_digit);
            if (irq.find_first_not_of("0123456789") != std::string::npos)
                continue; // LOC/RES 等事件计数行没有亲和性，跳过
            // 计数列是开机以来的累计值，分不出"历史"与"现任"邻居（中断挪走后旧计数
            // 还挂在原列上）；现居核以 effective_affinity_list 为准。
            const auto effective = read_line(std::format("/proc/irq/{}/effective_affinity_list", irq));
            if (!effective || *effective != core)
                continue;
            auto stream = std::istringstream{line.substr(colon + 1)};
            std::vector<std::string> tokens;
            std::string token;
            while (stream >> token)
                tokens.push_back(token);
            if (tokens.size() < static_cast<std::size_t>(total) + 1)
                continue;
            std::string name;
            for (std::size_t index = total + 1; index < tokens.size(); ++index)
                name += tokens[index] + " ";
            if (name.find("xhci_hcd") != std::string::npos)
                continue;
            if (name.find("nvme0q") != std::string::npos) {
                unmovable.push_back(name);
                continue;
            }
            neighbors.push_back(name);
        }
        if (!neighbors.empty()) {
            std::string joined;
            for (const auto& name : neighbors)
                joined += name + "; ";
            add(
                findings, Level::kWarn,
                std::format(
                    "cpu{} (xHCI IRQ core) shares hard IRQs with: {} -- hard IRQs preempt the "
                    "FIFO thread; steer them away (rt_tune does this at boot)", core, joined));
        }
        if (!unmovable.empty()) {
            std::string joined;
            for (const auto& name : unmovable)
                joined += name + "; ";
            add(
                findings, Level::kInfo,
                std::format(
                    "cpu{} also carries unmovable blk-mq IRQs ({}): fires only on real disk "
                    "IO", core, joined));
        }
    }

    static void check_xhci_irq_threads(std::vector<Finding>& findings) {
        DIR* proc = opendir("/proc");
        if (proc == nullptr) {
            add(findings, Level::kInfo, "cannot scan /proc for xHCI IRQ threads");
            return;
        }

        // 全机最大核频，用来判断中断线程是否落在降频核（P/E 核异构机器上成立，同构机器无感）。
        unsigned long global_max_freq = 0;
        for (int cpu = 0; cpu < 256; ++cpu)
            if (const auto frequency = cpu_max_freq(std::to_string(cpu)))
                global_max_freq = std::max(global_max_freq, *frequency);

        unsigned found = 0;
        while (const auto* entry = readdir(proc)) {
            char* end_pointer = nullptr;
            const long pid = strtol(entry->d_name, &end_pointer, 10);
            if (end_pointer == entry->d_name || *end_pointer != '\0')
                continue;
            const auto comm = read_line(std::format("/proc/{}/comm", static_cast<int>(pid)));
            if (!comm || comm->find("xhci_hcd") == std::string::npos
                || comm->find("irq/") != 0)
                continue;
            const auto thread = parse_irq_thread(static_cast<int>(pid), *comm);
            if (!thread)
                continue;
            ++found;
            const std::string core = irq_effective_core(thread->irq);
            const bool is_fifo = thread->policy == 1; // SCHED_FIFO
            add(
                findings, Level::kInfo,
                std::format(
                    "xHCI IRQ {} thread: pid={} core={} policy={} rt_priority={}", thread->irq,
                    thread->pid, core, is_fifo ? "fifo" : "other", thread->rt_priority));

            if (thread->rt_priority < 90 || !is_fifo)
                add(
                    findings, Level::kWarn,
                    std::format(
                        "xHCI IRQ {} thread at priority {} (not FIFO 90): with >=2 boards on one "
                        "controller the tail reaches ~1 ms; reboots revert it (HOST_TUNING 8/10.8). "
                        "Fix: chrt -f -p 90 {}",
                        thread->irq, is_fifo ? thread->rt_priority : 0, thread->pid));

            if (core.find('-') != std::string::npos || core == "?") {
                add(findings, Level::kInfo, "IRQ core ambiguous; skipping C1/frequency checks");
            } else {
                if (const auto max_frequency = cpu_max_freq(core);
                    global_max_freq != 0 && max_frequency && *max_frequency < global_max_freq)
                    add(
                        findings, Level::kWarn,
                        std::format(
                            "xHCI IRQ thread core cpu{} runs at {} kHz max vs {} kHz best core: "
                            "E/LP E core placement costs ~21 us p50 (HOST_TUNING 2.2)",
                            core, *max_frequency, global_max_freq));
                check_idle_cost(findings, core);
                check_irq_neighbors(findings, core);
            }
        }
        closedir(proc);
        if (found == 0)
            add(findings, Level::kInfo, "no xHCI IRQ threads found (no USB controller?)");
    }

    // ── irqbalance ────────────────────────────────────────────────────────

    static void check_irqbalance(std::vector<Finding>& findings) {
        DIR* proc = opendir("/proc");
        if (proc == nullptr)
            return;
        bool running = false;
        while (const auto* entry = readdir(proc)) {
            char* end_pointer = nullptr;
            const long pid = strtol(entry->d_name, &end_pointer, 10);
            if (end_pointer == entry->d_name || *end_pointer != '\0')
                continue;
            if (const auto comm = read_line(std::format("/proc/{}/comm", static_cast<int>(pid)));
                comm && *comm == "irqbalance") {
                running = true;
                break;
            }
        }
        closedir(proc);
        if (running)
            add(
                findings, Level::kWarn,
                "irqbalance is running: it migrates IRQs at runtime and adds jitter "
                "(HOST_TUNING 3). Fix: systemctl disable --now irqbalance");
        else
            add(findings, Level::kOk, "irqbalance not running");
    }

    // ── 板卡枚举：速度 + bootloader 滞留 ───────────────────────────────────

    static void check_boards(std::vector<Finding>& findings) {
        DIR* devices = opendir("/sys/bus/usb/devices");
        if (devices == nullptr) {
            add(findings, Level::kInfo, "cannot scan /sys/bus/usb/devices");
            return;
        }
        unsigned found = 0;
        while (const auto* entry = readdir(devices)) {
            const std::string base = std::string{"/sys/bus/usb/devices/"} + entry->d_name;
            const auto vendor = read_line(base + "/idVendor");
            if (!vendor || *vendor != "a511")
                continue;
            ++found;
            const auto product_id = read_line(base + "/idProduct").value_or("?");
            const auto speed = read_line(base + "/speed").value_or("?");
            const auto serial = read_line(base + "/serial").value_or("no-serial");
            const auto product = read_line(base + "/product").value_or("");
            // 速度是唯一可靠的信号：版本串对 12M/480M 完全相同（10.1）。
            if (product_id == "5321" || product_id == "5322" || product_id == "6e84") {
                if (speed == "480")
                    add(
                        findings, Level::kOk,
                        std::format(
                            "board {} ({}) at 480M HS", product_id, serial.substr(0, 8)));
                else
                    add(
                        findings, Level::kCritical,
                        std::format(
                            "board {} ({}) enumerated at {}M, expected 480M: +48 us p50, "
                            "version string is identical (HOST_TUNING 10.1). Reflash and reseat",
                            product_id, serial.substr(0, 8), speed));
            } else if (product_id == "0723") {
                add(
                    findings, Level::kOk,
                    std::format(
                        "board 0723 (mc02) at {}M FS (native Full-Speed, not a fault)", speed));
            } else {
                add(
                    findings, Level::kWarn,
                    std::format(
                        "a511 device with unknown PID {} ({}): likely a DFU bootloader sitting "
                        "on the bus -- its mere enumeration causes microframe-quantized 300 us "
                        "latency clusters on other boards (HOST_TUNING 11.1). Flash it or "
                        "authorize=0",
                        product_id, product));
            }
        }
        closedir(devices);
        if (found == 0)
            add(findings, Level::kWarn, "no a511 boards enumerated on any USB bus");
    }

    // ── xHCI IMOD（中断聚合）────────────────────────────────────────────────

    /// 读每个 xHCI 控制器的 IMOD（256ns 单位）。resource0 组可读时免 root（udev 规则），
    /// 读不到就报 kInfo 而不是失败——这个检查是尽力而为。
    static void check_imod(std::vector<Finding>& findings) {
        DIR* devices = opendir("/sys/bus/pci/devices");
        if (devices == nullptr)
            return;
        while (const auto* entry = readdir(devices)) {
            const std::string base = std::string{"/sys/bus/pci/devices/"} + entry->d_name;
            const auto device_class = read_line(base + "/class");
            if (!device_class || device_class->find("0x0c0330") == std::string::npos)
                continue;

            const int fd = open((base + "/resource0").c_str(), O_RDONLY | O_CLOEXEC);
            if (fd < 0) {
                add(
                    findings, Level::kInfo,
                    std::format("{}: resource0 unreadable without root; IMOD unchecked", entry->d_name));
                continue;
            }
            // runtime suspend（D3）下的控制器 MMIO 不可读——读了直接 fault。本机实测
            // 空闲的雷电 xHCI 就挂在 suspended 上；这恰好演示了为什么 rt_tune 要给
            // 挂着板子的控制器设 power/control=on。
            const auto runtime_status = read_line(base + "/power/runtime_status");
            if (runtime_status && *runtime_status != "active") {
                close(fd);
                add(
                    findings, Level::kInfo,
                    std::format(
                        "{}: controller runtime-suspended ({}); IMOD unchecked", entry->d_name,
                        *runtime_status));
                continue;
            }
            // CAPLENGTH 在 0x00，运行寄存器区偏移在 RTSOFF(0x18)，IMOD 在 RTS+0x24；
            // 映射 16KB 覆盖全部布局（tools/imod.c 同一算术）。
            void* mapping = mmap(nullptr, 0x4000, PROT_READ, MAP_SHARED, fd, 0);
            close(fd);
            if (mapping == MAP_FAILED) {
                add(
                    findings, Level::kInfo,
                    std::format("{}: resource0 mmap failed ({}); IMOD unchecked", entry->d_name, errno));
                continue;
            }
            const auto* registers = static_cast<const uint8_t*>(mapping);
            uint32_t rtsoff = 0;
            std::memcpy(&rtsoff, registers + 0x18, 4);
            const uint32_t rts = rtsoff & 0xFFFFFE0u;
            if (rts == 0 || rts + 0x28 > 0x4000) {
                munmap(mapping, 0x4000);
                add(
                    findings, Level::kInfo,
                    std::format(
                        "{}: unexpected RTS offset 0x{:x}; IMOD unchecked", entry->d_name, rtsoff));
                continue;
            }
            uint32_t imod = 0;
            std::memcpy(&imod, registers + rts + 0x24, 4);
            munmap(mapping, 0x4000);
            add(
                findings, Level::kInfo,
                std::format(
                    "xHCI {} IMOD = {} ns {}", entry->d_name, imod * 256u,
                    imod == 0 ? "(no aggregation)" : "(default kernel value is 40960 ns)"));
        }
        closedir(devices);
    }
};

} // namespace hcs_utility
