#!/bin/bash
# HCS 实时调优（需要 root）：把 libhcs/HOST_TUNING.md 实测出的、会在重启后丢失的
# 那几项在开机时重新落地（10.8 的"重启后缺三样"）。
#
#   sudo ./hcs_rt_tune.sh            # 应用
#   sudo ./hcs_rt_tune.sh --check    # 只报告将要做什么，不改任何状态
#
# 安装为开机服务（专用控制机）：
#   sudo gcc -O2 -o /usr/local/sbin/c1e ~/Desktop/libhcs/tools/c1e.c
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
        C1*)
            if [ "$(cat "$STATE/disable" 2>/dev/null)" = "1" ]; then
                apply "re-enable C1 on $CPUC (C1E is off; spinning is obsolete)" \
                    sh -c "echo 0 > $STATE/disable"
            fi
            ;;
        esac
    done
done

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

# ── 中断邻居清理：可挪设备中断请出隔离核与 xHCI 中断核 ────────────────────────
# 硬中断不受 SCHED_FIFO 保护，落到 xHCI 中断核或隔离核上就直接抢占中断线程/ctrl
# 线程。nvme0q* 是 blk-mq 的 per-CPU 亲和（sysfs 只读，HOST_TUNING 1.3），挪不动
# 也不必挪——它只在真有磁盘 IO 时发声。IRQ 号与核号每次开机都变，必须动态发现。
XHCI_IRQS=$(grep -E "^ *[0-9]+:" /proc/interrupts | grep xhci_hcd | awk '{print $1}' | tr -d ':' | tr '\n' ' ')
ISOLATED=$(sed -n 's/.*isolcpus=\([^ ]*\).*/\1/p' /proc/cmdline | tr ',' ' ')
XCORE=$(cat "/proc/irq/${XHCI_IRQS%% *}/effective_affinity_list" 2>/dev/null)
TGT=""
for C in $(seq 0 31); do
    [ -d "/sys/devices/system/cpu/cpu$C" ] || continue
    case " $ISOLATED " in *" $C "*) continue ;; esac
    [ "$C" = "$XCORE" ] && continue
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
        [ "$EFF" = "$XCORE" ] && NEED=1
        case " $ISOLATED " in *" $EFF "*) NEED=1 ;; esac
        [ "$NEED" = 1 ] || continue
        if echo "$TGT" > "/proc/irq/$IRQ/smp_affinity_list" 2>/dev/null; then
            say "moved irq $IRQ ($EFF -> $TGT): $NAME"
        else
            say "unmovable irq $IRQ ($EFF): $NAME"
        fi
    done
else
    say "SKIP: no housekeeping target core found for IRQ steering"
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
