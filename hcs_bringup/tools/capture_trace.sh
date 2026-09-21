#!/usr/bin/env bash
# 录一段内核 trace，给 HcsLinkProbe 的慢回包对时刻用。需要 root（tracefs 写权限）。
#
#   sudo hcs_bringup/tools/capture_trace.sh [时长秒] [输出前缀] [工作目录]
#
# -C mono：trace 时间戳直接用 CLOCK_MONOTONIC，和探针慢回包记录里的
#          arrival=<...> ns 同一条时间轴，按纳秒就能对上，不用换算偏移。
# 事件：   -e irq 覆盖 irq_handler_* 和 softirq_*（PREEMPT_RT 内核没有独立的
#          softirq 子系统名，softirq_entry/exit/raise 都挂在 irq 下）；
#          sched_switch / sched_wakeup 看线程有没有被及时调度；
#          xhci_urb_enqueue/giveback 看 URB 在内核里滞留了多久，
#          把"内核没交付"和"线程没被调度"分开。
# 缓冲：   每核 8 MB。sched_switch 在 1 kHz 控制循环下很吵，小了会丢事件。
set -euo pipefail

duration=${1:-20}
prefix=${2:-}
outdir=${3:-.}
dat="${outdir}/trace${prefix:+_$prefix}.dat"

events=(-e irq -e sched:sched_switch -e sched:sched_wakeup
    -e xhci-hcd:xhci_urb_enqueue -e xhci-hcd:xhci_urb_giveback
    -e xhci-hcd:xhci_get_port_status -e xhci-hcd:xhci_handle_port_status
    -e irq_vectors:local_timer_entry -e irq_vectors:local_timer_exit
    -e irq_vectors:reschedule_entry -e irq_vectors:reschedule_exit
    -e workqueue:workqueue_execute_start -e workqueue:workqueue_execute_end)

echo "xHCI IRQ affinity（分析时重点看这些核）:"
for irq in $(awk -F: '/xhci/ {gsub(/ /, "", $1); print $1}' /proc/interrupts); do
    echo "  irq $irq -> cpu $(cat "/proc/irq/$irq/smp_affinity_list")"
done
echo "events: ${events[*]}"
echo "buffer: 8 MB/cpu, duration: ${duration}s, output: $dat"

trace-cmd reset >/dev/null 2>&1
exec trace-cmd record -C mono -b 8000 "${events[@]}" -o "$dat" sleep "$duration"
