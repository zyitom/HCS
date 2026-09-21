#!/usr/bin/env python3
"""从 trace-cmd report 文本重建每拍的内核侧 rtt。

对每块板（slot 5 = 5321，slot 6 = mc02）：
  out_enqueue(cycle x) -> 该拍之后第一个同 slot 的 ep1in giveback = 回包交付。
  delay = in_giveback - out_enqueue，这就是探针 rtt 里"内核+硬件+设备"那一段。
IN giveback 严格按序（每拍恰好一个 IN），所以按"之后第一个"配对是稳的。
同时统计 CPU4 上 irq=135 的相邻间隔，找 >500us 的中断空窗。
"""
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "traces/baselineA.txt"
# 可选第二参数：slot:name,slot:name（默认 5:5321,6:mc02）
BOARDS = {}
if len(sys.argv) > 2:
    for pair in sys.argv[2].split(","):
        s, name = pair.split(":")
        BOARDS[int(s)] = name
else:
    BOARDS = {5: "5321", 6: "mc02"}

line_re = re.compile(
    r"\[\s*(\d+)\]\s+([\d.]+):\s+(\w+):\s+(\S+):\s+urb (0x[0-9a-f]+) "
    r"pipe (\d+) slot (\d+)")
irq_re = re.compile(r"\[\s*(\d+)\]\s+([\d.]+): irq_handler_entry:\s+irq=135 ")
out_enqueue = {s: None for s in BOARDS}  # (ts, urb)
results = {s: [] for s in BOARDS}        # (out_ts, in_ts, delay_us, out_urb)
irq_ts = []

with open(path) as f:
    for line in f:
        if "xhci_urb" in line:
            m = line_re.search(line)
            if not m:
                continue
            ts = float(m.group(2))
            event, urb, _pipe, slot = m.group(3), m.group(5), m.group(6), int(m.group(7))
            if slot not in BOARDS:
                continue
            if event == "xhci_urb_enqueue" and "ep1out" in line:
                out_enqueue[slot] = (ts, urb)
            elif event == "xhci_urb_giveback" and "ep1in" in line:
                if out_enqueue[slot] is not None:
                    o_ts, o_urb = out_enqueue[slot]
                    delay_us = (ts - o_ts) * 1e6
                    results[slot].append((o_ts, ts, delay_us, o_urb))
                    out_enqueue[slot] = None
        elif "irq_handler_entry" in line and "irq=135" in line:
            m = irq_re.search(line)
            if m:
                irq_ts.append(float(m.group(2)))

for slot, name in BOARDS.items():
    rows = results[slot]
    print(f"\n=== {name} (slot {slot}): {len(rows)} 拍 ===")
    delays = sorted(r[2] for r in rows)
    if delays:
        n = len(delays)
        print(f"  p50={delays[n // 2]:.0f}us  p99={delays[int(n * 0.99)]:.0f}us  "
              f"max={delays[-1]:.0f}us  >=500us: {sum(d >= 500 for d in delays)}  "
              f">=1ms: {sum(d >= 1000 for d in delays)}")
    for o_ts, i_ts, d, urb in rows:
        if d >= 300:
            print(f"  out={o_ts:.6f} in={i_ts:.6f} delay={d:.0f}us urb={urb}")

print(f"\n=== xHCI irq=135 相邻间隔（{len(irq_ts)} 次中断） ===")
irq_ts.sort()
gaps = [(b - a, a, b) for a, b in zip(irq_ts, irq_ts[1:])]
gaps_sorted = sorted(gaps, reverse=True)
print(f"  中位间隔={sorted(g[0] for g in gaps)[len(gaps) // 2] * 1e6:.0f}us  "
      f"最大={gaps_sorted[0][0] * 1e6:.0f}us  >500us: {sum(1 for g in gaps if g[0] > 0.0005)}")
for g, a, b in gaps_sorted[:15]:
    print(f"  gap={g * 1e6:.0f}us  from {a:.6f} to {b:.6f}")
