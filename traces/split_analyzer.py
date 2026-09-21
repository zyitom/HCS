#!/usr/bin/env python3
"""把 rtt_split.csv 的三轮时钟拆成下行段/上行段。

字段：seq, submit_ns, recv_ns（主机 CLOCK_MONOTONIC）、board_us（板端 PTPC0，
被 USB SOF 驯化到主机节拍，但绝对偏移未知且两块晶振有 ppm 级漂移）。

模型：Phase A = submit -> CAN2 收到回环帧（USB 下行事务 + 固件转发 + CAN 帧线上时间）
     Phase B = CAN2 收到 -> 主机回调（固件读邮箱 + IN 轮询残差 + IMOD + giveback）
Phase A_k = b_k + D - s_k，Phase B_k = g_k - (b_k + D)，A+B = g-s = total（精确）。

D 不能直接测，但每拍都有物理下限：A >= CAN 帧线上时间（1M/5M 8 字节 FD ≈ 45µs），
B >= 固件读邮箱 + 一次 IN 轮询（≈ 5µs）。全部拍取交集就夹出 D 的可行窗口，
窗口宽度即拆分的不确定度。两轴晶振 ppm 漂移先用线性回归扣掉。
"""
import csv
import statistics
import sys

FLOOR_A = 45.0  # µs：CAN 1M/5M 8 字节帧的线上时间下限
FLOOR_B = 5.0   # µs：读邮箱 + 武装 IN + 至少一次轮询

path = sys.argv[1] if len(sys.argv) > 1 else "rtt_split.csv"

rows = []
with open(path) as f:
    for row in csv.DictReader(f):
        rows.append((
            int(row["seq"]),
            int(row["submit_ns"]) / 1e3,  # µs
            int(row["recv_ns"]) / 1e3,
            float(row["board_us"]),
        ))
rows.sort()
print(f"{len(rows)} 样本")

total = [g - s for _, s, g, _ in rows]

# 两轴晶振 ppm 漂移：b_k - s_k 对 seq 回归，斜率非零就先扣掉
raw = [b - s for _, s, _, b in rows]
n = len(rows)
mean_k = (n - 1) / 2
mean_r = sum(raw) / n
cov = sum((k - mean_k) * (raw[k] - mean_r) for k in range(n))
var_k = sum((k - mean_k) ** 2 for k in range(n))
slope = cov / var_k if var_k else 0.0
cadence = (s_k_last := rows[-1][1]) and (rows[-1][1] - rows[0][1]) / max(n - 1, 1)
print(f"漂移斜率 {slope:+.4f} µs/拍（两轴相对 ppm ≈ {slope / cadence * 1e6:+.1f}，节奏 {cadence:.0f} µs/拍）")
b_corr = [b - slope * k for k, (_, _, _, b) in enumerate(rows)]

# D 可行窗口（必须用漂移修正后的 b）
def window(floor_a):
    lo = max(s_k - b_k + floor_a for (_, s_k, _, _), b_k in zip(rows, b_corr))
    hi = min(g_k - b_k - FLOOR_B for (_, _, g_k, _), b_k in zip(rows, b_corr))
    return lo, hi

raw_corr = [r - slope * k for k, r in enumerate(raw)]
print(f"漂移修正后 raw 的 σ = {statistics.stdev(raw_corr):.2f} µs，极差 "
      f"{max(raw_corr) - min(raw_corr):.1f} µs")

# 两种时间戳语义都算：capture_on_sof 若盖在 CAN 帧起始，CAN 线上时间落在 B 段；
# 若盖在接收完成，落在 A 段。窗口宽的那个是自洽的。
for floor_a, tag in [(10.0, "假设 b=帧起始(SOF)：A=USB下行+固件, B=含CAN线上时间"),
                     (45.0, "假设 b=接收完成：A=含CAN线上时间, B=纯上行")]:
    lo, hi = window(floor_a)
    ok = hi > lo
    print(f"\n== {tag} ==")
    print(f"   D 窗口 [{lo:.1f}, {hi:.1f}]，宽 {hi - lo:.1f} µs {'✓' if ok else '✗ 空'}")
    if not ok:
        continue
    d = (lo + hi) / 2

    def split_at(d):
        a = [b_k + d - s_k for (_, s_k, _, _), b_k in zip(rows, b_corr)]
        bb = [g_k - b_k - d for (_, _, g_k, _), b_k in zip(rows, b_corr)]
        return a, bb

    def stats(name, xs):
        xs_sorted = sorted(xs)
        p = lambda q: xs_sorted[min(int(len(xs_sorted) * q), len(xs_sorted) - 1)]
        print(f"   {name:8s} min/p50/p99/max = {p(0):6.1f} / {p(0.5):6.1f} / "
              f"{p(0.99):6.1f} / {p(1.0):6.1f}  σ={statistics.stdev(xs):5.1f}")

    a, bb = split_at(d)
    stats("Phase A", a)
    stats("Phase B", bb)
    pairs = sorted(zip(total, bb), reverse=True)[:20]
    tm = statistics.mean(t for t, _ in pairs)
    print(f"   最慢 20 拍：total {tm:.0f} µs，Phase B 均值 {statistics.mean(b for _, b in pairs):.0f} "
          f"µs（{statistics.mean(b for _, b in pairs) / tm * 100:.0f}%）")
