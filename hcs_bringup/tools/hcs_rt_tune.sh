#!/bin/bash
# HCS 实时调优（需要 root）：把 libhcs/HOST_TUNING.md 实测出的、会在重启后丢失的
# 那几项在开机时重新落地（10.8 的"重启后缺三样"）。
#
#   sudo ./hcs_rt_tune.sh            # 应用
#   sudo ./hcs_rt_tune.sh --check    # 只报告将要做什么，不改任何状态
#
# 安装为开机服务（专用控制机）：
#   sudo gcc -O2 -o /usr/local/sbin/c1e  <libhcs>/tools/c1e.c
#   sudo gcc -O2 -o /usr/local/sbin/imod <libhcs>/tools/imod.c
#   sudo cp hcs_rt_tune.sh /usr/local/sbin/
#   sudo cp hcs_rt_tune.service /etc/systemd/system/
#   sudo systemctl daemon-reload && sudo systemctl enable --now hcs_rt_tune.service
#
# 与应用侧的分工：本脚本负责系统级（中断线程优先级、C1E、governor/EPP、
# RT throttling、xHCI 控制器 runtime PM）；应用自己的 mlock/prefault/SCHED_FIFO
# 由 hcs_utility::RealtimeArm 负责，两边不重复。
# cpu_dma_latency 不在这里持（fd 必须由活进程持有，oneshot 退出即失效，10.8）。
# C-state 的正确形态是 2026-09-15 实测定下的：全机关 C1E（MSR 0x1FC bit1，退出
# C1 不再降压爬坡）+ 所有核 C1 正常——延迟与"禁 C1 强制 POLL 空转"持平
# （EP0 p50 73.5 vs 72.7 us）但不白烧任何一个核。此前"禁中断核 C1"的方案作废。

set -u

CHECK=0

# 机器相关的设置放 /etc/default/hcs_rt_tune（不存在就全用默认）：
#   XHCI_IRQ_CPU=<n>   把每个 xHCI 中断固定到 CPU n（非隔离的 P 核）。不设则沿用内核在
#                      irqaffinity 里挑的那颗，每次开机可能不同。
XHCI_IRQ_CPU=""
[ -r /etc/default/hcs_rt_tune ] && . /etc/default/hcs_rt_tune
[ "${1:-}" = "--check" ] && CHECK=1

say() { printf '[rt_tune] %s\n' "$1"; }

apply() { # apply <描述> <命令...>：--check 只打印，不执行
    if [ "$CHECK" = 1 ]; then
        say "would: $1"
    else
        if shift && "$@" >/dev/null 2>&1; then
            say "applied: $*"
        else
            say "FAILED: $*"
        fi
    fi
}

# ── RT throttling：不关是 max ~50ms 级（HOST_TUNING 10.10.4）──────────────────
CURRENT="$(cat /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null)"
if [ "$CURRENT" = "-1" ]; then
    say "sched_rt_runtime_us=-1 already"
else
    apply "sysctl -w kernel.sched_rt_runtime_us=-1" \
        sysctl -w kernel.sched_rt_runtime_us=-1
fi

# ── governor / EPP：把频率顶住（1.1/1.2；EPP 是独立的提示位，两者不是一回事）──
for GOV in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    if [ "$(cat "$GOV" 2>/dev/null)" != "performance" ]; then
        apply "governor=performance on $GOV" sh -c "echo performance > $GOV"
    fi
done
for EPP in /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference; do
    if [ "$(cat "$EPP" 2>/dev/null)" != "performance" ]; then
        apply "EPP=performance on $EPP" sh -c "echo performance > $EPP"
    fi
done

# ── C1E：全机关掉（MSR 0x1FC bit1），并保证所有核 C1 正常（不禁、不空转）──────
# C1E 进 C1 时降压降频，退出爬坡实测 15-20 us（10.9.3）。关掉它之后 C1 退化为
# 普通 halt 唤醒，所有核（中断核、ctrl 核、隔离对）一起受益，无需任何核空转。
C1E_BIN="/usr/local/sbin/c1e"
if [ -x "$C1E_BIN" ]; then
    if [ "$CHECK" = 1 ]; then
        say "would: $C1E_BIN off (C1E off machine-wide)"
    else
        if "$C1E_BIN" off | grep -q "C1E=0"; then
            say "C1E off machine-wide"
        else
            say "FAILED: $C1E_BIN off"
        fi
    fi
else
    say "SKIP: c1e tool not found (build tools/c1e.c, see header)"
fi

# 恢复曾被"禁 C1 换延迟"方案禁掉的 C1（现在 C1E 已关，C1 正常睡眠就是最优态）。
for CPUC in /sys/devices/system/cpu/cpu*/cpuidle; do
    [ -d "$CPUC" ] || continue
    for STATE in "$CPUC"/state*; do
        [ -d "$STATE" ] || continue
        case "$(cat "$STATE/name" 2>/dev/null)" in
        C1 | C1E | C1_ACPI) # 精确匹配：C1* 会把 C10 也算进来
            if [ "$(cat "$STATE/disable" 2>/dev/null)" = "1" ]; then
                apply "re-enable C1 on $CPUC (C1E is off; spinning is obsolete)" \
                    sh -c "echo 0 > $STATE/disable"
            fi
            ;;
        esac
    done
done

# ── 隔离核：只留 POLL 与 C1，禁掉更深的睡眠 ──────────────────────────────────────
# isolcpus + nohz_full 的核空闲时连 1 ms 的时钟节拍都没有，cpuidle 判断"要闲很久"，
# 每次都进最深一级（AD600 上是 C10）；回包唤醒 IO 线程时要先从那里爬出来。AD600 实测
# （2026-10-03，经典 CAN 1M 往返，HOST_TUNING 12.7）：隔离核允许深睡时比不隔离还差，
# 禁掉之后 p50 / p99 各好约 10 us。核号从 /sys/devices/system/cpu/isolated 发现。
ISOLATED_LIST="$(cat /sys/devices/system/cpu/isolated 2>/dev/null)"
if [ -n "$ISOLATED_LIST" ]; then
    for RANGE in $(echo "$ISOLATED_LIST" | tr ',' ' '); do
        for C in $(seq "${RANGE%-*}" "${RANGE#*-}"); do
            for STATE in /sys/devices/system/cpu/cpu"$C"/cpuidle/state*; do
                [ -d "$STATE" ] || continue
                case "$(cat "$STATE/name" 2>/dev/null)" in
                POLL | C1 | C1E | C1_ACPI) continue ;; # 不能写 C1*，会把 C10 漏过去
                esac
                [ "$(cat "$STATE/disable" 2>/dev/null)" = "1" ] && continue
                apply "cpu$C: disable $(cat "$STATE/name") (isolated core)" \
                    sh -c "echo 1 > $STATE/disable"
            done
        done
    done
fi

# ── xHCI 中断线程：FIFO 90（第 8 节多板硬要求）────────────────────────────────
for THREAD in /proc/[0-9]*/comm; do
    COMM="$(cat "$THREAD" 2>/dev/null)" || continue
    case "$COMM" in
    irq/*-xhci_hcd) ;;
    *) continue ;;
    esac
    PID="$(basename "$(dirname "$THREAD")")"
    IRQ="${COMM#irq/}"; IRQ="${IRQ%%-*}"

    # /proc/<pid>/stat：第 40 字段 = rt_priority，第 41 字段 = policy（1 = SCHED_FIFO）。
    POLICY="$(awk '{print $41}' "/proc/$PID/stat" 2>/dev/null)"
    PRIO="$(awk '{print $40}' "/proc/$PID/stat" 2>/dev/null)"
    if [ "$POLICY" = "1" ] && [ "$PRIO" = "90" ]; then
        say "xHCI irq/$IRQ thread (pid $PID) already FIFO 90"
    else
        # 注意 chrt 参数顺序：-p 模式下是 "chrt -f -p <prio> <pid>"；
        # 写成 "chrt -f 90 -p $pid" 会把 -p 当作要执行的程序（rc=127）。
        apply "xHCI irq/$IRQ thread (pid $PID) -> FIFO 90" chrt -f -p 90 "$PID"
    fi

    # 中断核本身的 C-state 不再单独处理：C1E 已全机关掉（上方），
    # 中断核的 C1 唤醒就是普通 halt，无需禁 C1 空转（10.9.4 方案已被取代）。
done

# ── xHCI 中断固定到一颗核（XHCI_IRQ_CPU）──────────────────────────────────────
# 内核每次开机在 irqaffinity 允许的核里任挑一颗。AD600 上落到 cpu5 比落到 cpu1 的 p99.9
# 差 30-50 us（同一次开机交错实测，三种配置里两种复现，HOST_TUNING 12.9），所以要钉住。
# 必须在下面的中断邻居清理之前：清理按 xHCI 中断的实际落点决定把别的中断请走。
if [ -n "$XHCI_IRQ_CPU" ]; then
    for X in $(grep -E "^ *[0-9]+:" /proc/interrupts | grep xhci_hcd | awk '{print $1}' | tr -d ':'); do
        if [ "$(cat "/proc/irq/$X/effective_affinity_list" 2>/dev/null)" = "$XHCI_IRQ_CPU" ]; then
            say "xHCI irq $X already on cpu$XHCI_IRQ_CPU"
        else
            apply "xHCI irq $X -> cpu$XHCI_IRQ_CPU" sh -c "echo $XHCI_IRQ_CPU > /proc/irq/$X/smp_affinity_list"
        fi
    done
fi

# ── 中断邻居清理：可挪设备中断请出隔离核与 xHCI 中断核 ────────────────────────
# 硬中断不受 SCHED_FIFO 保护，落到 xHCI 中断核或隔离核上就直接抢占中断线程/ctrl
# 线程。nvme0q* 是 blk-mq 的 per-CPU 亲和（sysfs 只读，HOST_TUNING 1.3），挪不动
# 也不必挪——它只在真有磁盘 IO 时发声。IRQ 号与核号每次开机都变，必须动态发现。
XHCI_IRQS=$(grep -E "^ *[0-9]+:" /proc/interrupts | grep xhci_hcd | awk '{print $1}' | tr -d ':' | tr '\n' ' ')
ISOLATED=$(sed -n 's/.*isolcpus=\([^ ]*\).*/\1/p' /proc/cmdline | tr ',' ' ')
# 每个 xHCI 中断各自落在哪颗核都要保护：机器上可能有不止一个 xHCI 控制器（AD600 的雷电
# 控制器 0d.0 排在 PCH 控制器 14.0 前面），只看第一个会保护错核。
XCORES=$(for X in $XHCI_IRQS; do cat "/proc/irq/$X/effective_affinity_list" 2>/dev/null; done | sort -un | tr '\n' ' ')
TGT=""
for C in $(seq 0 31); do
    [ -d "/sys/devices/system/cpu/cpu$C" ] || continue
    case " $ISOLATED " in *" $C "*) continue ;; esac
    case " $XCORES " in *" $C "*) continue ;; esac
    TGT=$C
    break
done
if [ -n "$TGT" ]; then
    grep -E "^ *[0-9]+:" /proc/interrupts | while read -r line; do
        IRQ=$(echo "$line" | awk '{print $1}' | tr -d ':')
        case " $XHCI_IRQS " in *" $IRQ "*) continue ;; esac
        NAME=$(echo "$line" | awk '{for(i=11;i<=NF;i++) printf "%s ", $i}')
        case "$NAME" in *nvme0q*) continue ;; esac
        EFF=$(cat "/proc/irq/$IRQ/effective_affinity_list" 2>/dev/null) || continue
        NEED=0
        case " $XCORES " in *" $EFF "*) NEED=1 ;; esac
        case " $ISOLATED " in *" $EFF "*) NEED=1 ;; esac
        [ "$NEED" = 1 ] || continue
        if [ "$CHECK" = 1 ]; then
            say "would: move irq $IRQ ($EFF -> $TGT): $NAME"
        elif echo "$TGT" > "/proc/irq/$IRQ/smp_affinity_list" 2>/dev/null; then
            say "moved irq $IRQ ($EFF -> $TGT): $NAME"
        else
            say "unmovable irq $IRQ ($EFF): $NAME"
        fi
    done
else
    say "SKIP: no housekeeping target core found for IRQ steering"
fi

# ── xHCI 中断合并（IMOD）→ 0 ─────────────────────────────────────────────────
# Linux 给每个 xHCI 的中断器 0 写 40.96 us 的 IMOD，控制器每次重新初始化都会写回。
# AD600 单板回环实测（2026-10-03，HOST_TUNING 12.9）：改成 0 后往返 p99 三轮都低 3-4 us。
# 早先"IMOD 实读 0"的结论来自只映射了 4 KB 的旧版 tools/imod.c（PCH xHCI 的运行寄存器
# 在 0x2000 之后），是错读。
IMOD_BIN="/usr/local/sbin/imod"
if [ -x "$IMOD_BIN" ]; then
    for PCI in /sys/bus/pci/devices/*; do
        [ "$(cat "$PCI/class" 2>/dev/null)" = "0x0c0330" ] || continue
        [ "$(cat "$PCI/power/runtime_status" 2>/dev/null)" = "active" ] || continue
        BDF="$(basename "$PCI")"
        if "$IMOD_BIN" "$BDF" | grep -q "IMOD=0 "; then
            say "$BDF IMOD already 0"
        else
            apply "$BDF IMOD -> 0 (no interrupt moderation)" "$IMOD_BIN" "$BDF" 0
        fi
    done
else
    say "SKIP: imod tool not found (build libhcs tools/imod.c to /usr/local/sbin/imod)"
fi

# ── xHCI 控制器 + USB root hub 的 runtime PM（文档未覆盖的新项）───────────────
# power/control=auto 时控制器/root hub 可能在空闲期 runtime suspend，第一笔回包
# 要付唤醒代价，且 xHCI 控制器重新初始化会把 IMOD 重写回 40 us 默认值。
for PCI in /sys/bus/pci/devices/*; do
    [ "$(cat "$PCI/class" 2>/dev/null)" = "0x0c0330" ] || continue
    if [ "$(cat "$PCI/power/control" 2>/dev/null)" != "on" ]; then
        apply "$PCI power/control -> on (prevent xHCI runtime suspend)" \
            sh -c "echo on > $PCI/power/control"
    fi
done
for HUB in /sys/bus/usb/devices/usb*; do
    if [ "$(cat "$HUB/power/control" 2>/dev/null)" != "on" ]; then
        apply "$HUB power/control -> on (root hub runtime PM)" \
            sh -c "echo on > $HUB/power/control"
    fi
done

if [ "$CHECK" = 1 ]; then
    say "done (check mode, nothing written)"
else
    say "done"
fi
