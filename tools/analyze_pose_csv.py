#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""分析姿态 CSV（四元数/欧拉角）的小工具。

输入 CSV 需包含列：
- server_time_ms（毫秒时间戳）
- roll_deg, pitch_deg, yaw_deg（角度，单位：deg）

输出：
- roll/pitch/yaw 的均值、标准差、最小/最大值
- yaw 随时间的线性拟合斜率（deg/s, deg/min），以及首尾差值

不依赖第三方库（无 pandas）。

用法示例（Windows 推荐用 py）：
  py tools/analyze_pose_csv.py e:\\DOWNLOAD\\pose-data-20260407-193631.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from typing import Iterable, Optional


REQUIRED_COLUMNS = {
    "server_time_ms",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
}


@dataclass
class OnlineStats:
    n: int = 0
    mean: float = 0.0
    m2: float = 0.0
    min_v: Optional[float] = None
    max_v: Optional[float] = None

    def add(self, x: float) -> None:
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        delta2 = x - self.mean
        self.m2 += delta * delta2

        if self.min_v is None or x < self.min_v:
            self.min_v = x
        if self.max_v is None or x > self.max_v:
            self.max_v = x

    @property
    def variance(self) -> float:
        if self.n <= 1:
            return 0.0
        return self.m2 / (self.n - 1)

    @property
    def std(self) -> float:
        return math.sqrt(self.variance)


@dataclass
class OnlineLinearFit:
    """在线线性回归：y = m*x + b，仅输出 m。"""

    n: int = 0
    mean_x: float = 0.0
    mean_y: float = 0.0
    sxx: float = 0.0
    sxy: float = 0.0

    def add(self, x: float, y: float) -> None:
        self.n += 1
        dx = x - self.mean_x
        self.mean_x += dx / self.n
        dy = y - self.mean_y
        self.mean_y += dy / self.n
        self.sxx += dx * (x - self.mean_x)
        self.sxy += dx * (y - self.mean_y)

    @property
    def slope(self) -> float:
        if self.sxx == 0.0:
            return 0.0
        return self.sxy / self.sxx


def _parse_float(row: dict, key: str, line_no: int) -> float:
    raw = row.get(key, "")
    try:
        return float(raw)
    except Exception as exc:  # noqa: BLE001
        raise ValueError(f"第 {line_no} 行字段 {key} 无法解析为浮点数: {raw!r}") from exc


def iter_rows(csv_path: str) -> Iterable[tuple[int, dict]]:
    with open(csv_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV 没有表头（header）")

        missing = sorted(REQUIRED_COLUMNS - set(reader.fieldnames))
        if missing:
            raise ValueError(f"CSV 缺少必要列: {', '.join(missing)}")

        for i, row in enumerate(reader, start=2):
            yield i, row


def analyze(csv_path: str, *, skip_seconds: float = 0.0, take_seconds: float = 0.0) -> int:
    roll_stats = OnlineStats()
    pitch_stats = OnlineStats()
    yaw_stats = OnlineStats()

    yaw_fit = OnlineLinearFit()

    first_t0_ms: Optional[float] = None
    first_yaw: Optional[float] = None
    last_yaw: Optional[float] = None
    last_t_s: Optional[float] = None

    for line_no, row in iter_rows(csv_path):
        t_ms = _parse_float(row, "server_time_ms", line_no)
        if first_t0_ms is None:
            first_t0_ms = t_ms

        t_s = (t_ms - first_t0_ms) / 1000.0

        if skip_seconds and t_s < skip_seconds:
            continue
        if take_seconds and t_s > (skip_seconds + take_seconds):
            break

        roll = _parse_float(row, "roll_deg", line_no)
        pitch = _parse_float(row, "pitch_deg", line_no)
        yaw = _parse_float(row, "yaw_deg", line_no)

        roll_stats.add(roll)
        pitch_stats.add(pitch)
        yaw_stats.add(yaw)

        yaw_fit.add(t_s, yaw)

        if first_yaw is None:
            first_yaw = yaw
        last_yaw = yaw
        last_t_s = t_s

    if roll_stats.n == 0:
        print("没有可分析的数据（可能 skip/take 过滤后为空）", file=sys.stderr)
        return 2

    def fmt_stats(name: str, st: OnlineStats) -> str:
        return (
            f"{name}: n={st.n} mean={st.mean:.6f} std={st.std:.6f} "
            f"min={st.min_v:.6f} max={st.max_v:.6f}"
        )

    print(fmt_stats("roll_deg", roll_stats))
    print(fmt_stats("pitch_deg", pitch_stats))
    print(fmt_stats("yaw_deg", yaw_stats))

    m = yaw_fit.slope
    print(f"yaw 线性拟合漂移: m={m:.9f} deg/s  (={m*60.0:.6f} deg/min)")

    if first_yaw is not None and last_yaw is not None and last_t_s is not None:
        print(
            f"yaw 首尾: first={first_yaw:.6f} last={last_yaw:.6f} "
            f"delta={last_yaw-first_yaw:.6f} duration_s={last_t_s:.3f}"
        )

    return 0


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description="分析姿态 CSV（roll/pitch/yaw + yaw 漂移）")
    p.add_argument("csv", help="CSV 文件路径")
    p.add_argument("--skip-seconds", type=float, default=0.0, help="跳过开头 N 秒（默认 0）")
    p.add_argument("--take-seconds", type=float, default=0.0, help="只分析随后 N 秒（默认 0=全量）")
    args = p.parse_args(argv)

    try:
        return analyze(args.csv, skip_seconds=max(0.0, args.skip_seconds), take_seconds=max(0.0, args.take_seconds))
    except Exception as exc:  # noqa: BLE001
        print(f"错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
