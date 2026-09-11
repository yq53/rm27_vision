#!/usr/bin/env python3
"""对比两次 eval_demo 的汇总结果。

用法:
    python3 02_tracker/scripts/compare_eval.py <tagA> <tagB> [更多tag...]

读取 results/eval_<tag>_summary.txt，输出并排对比表（含 delta）。
"""

import os
import re
import sys

KEYS = [
    "frames",
    "frames_with_armor",
    "rate",  # frames_with_armor 的 rate
    "pnp_ok",
    "reproj_px",
    "dist_raw",
    "dist_ekf",
    "jitter_raw",
    "jitter_ekf",
    "yaw_deg",
    "detect_ms",
    "pnp_ms",
]


def parse_summary(path):
    """把 summary.txt 解析成 {指标: 数值} 的扁平字典。"""
    if not os.path.exists(path):
        raise SystemExit(f"找不到 {path}，请先运行 eval_demo 生成该 tag 的结果")
    text = open(path, encoding="utf-8").read()
    values = {}
    for line in text.splitlines():
        # frames=687 / frames_with_armor=498  rate=72.4891%
        m = re.match(r"^(\w+)=(-?[\d.]+)(?:\s+rate=(-?[\d.]+)%)?", line.strip())
        if m:
            values[m.group(1)] = float(m.group(2))
            if m.group(1) == "frames_with_armor" and m.group(3):
                values["rate"] = float(m.group(3))
            continue
        # reproj_px mean=4.1066  median=3.1520 / dist_raw mean=0.89  dist_ekf mean=1.05（同一行多组）
        pairs = re.findall(r"(\w+) mean=(-?[\d.]+)", line.strip())
        if pairs:
            for name, val in pairs:
                values[name + "_mean"] = float(val)
            continue
        # jitter_raw mean_abs=0.0853 std=0.1017 n=339
        m = re.match(r"^(\w+) mean_abs=(-?[\d.]+)", line.strip())
        if m:
            values[m.group(1) + "_meanabs"] = float(m.group(2))
            continue
        # time detect_ms=37.0  pnp_ms=0.06
        m = re.match(r"^time detect_ms=(-?[\d.]+)\s+pnp_ms=(-?[\d.]+)", line.strip())
        if m:
            values["detect_ms"] = float(m.group(1))
            values["pnp_ms"] = float(m.group(2))
    return values


def pick(values, key):
    """把对外指标名映射到解析出的键。"""
    mapping = {
        "frames": "frames",
        "frames_with_armor": "frames_with_armor",
        "rate": "rate",
        "pnp_ok": "pnp_ok",
        "reproj_px": "reproj_px_mean",
        "dist_raw": "dist_raw_mean",
        "dist_ekf": "dist_ekf_mean",
        "jitter_raw": "jitter_raw_meanabs",
        "jitter_ekf": "jitter_ekf_meanabs",
        "yaw_deg": "yaw_deg_meanabs",
        "detect_ms": "detect_ms",
        "pnp_ms": "pnp_ms",
    }
    return values.get(mapping[key])


def main():
    tags = sys.argv[1:]
    if len(tags) < 2:
        raise SystemExit(__doc__)
    data = {
        t: parse_summary(os.path.join("results", f"eval_{t}_summary.txt")) for t in tags
    }
    base = tags[0]
    width = max(len(k) for k in KEYS) + 2
    header = "指标".ljust(width) + "".join(t.rjust(16) for t in tags)
    if len(tags) > 1:
        header += "Δ(最后-首个)".rjust(18)
    print(header)
    print("-" * len(header))
    for key in KEYS:
        row = key.ljust(width)
        for t in tags:
            v = pick(data[t], key)
            row += ("-" if v is None else f"{v:.4f}").rjust(16)
        first, last = pick(data[base], key), pick(data[tags[-1]], key)
        delta = "-" if (first is None or last is None) else f"{last - first:+.4f}"
        row += delta.rjust(18)
        print(row)
    print("\n注：检出率=rate(%)；reproj/dist/jitter/yaw 为均值；jitter 为相邻帧距离跳动均值(m)。")


if __name__ == "__main__":
    main()
