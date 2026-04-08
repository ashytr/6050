#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Receive RAW IMU lines from ESP32 UART and compare Euler solutions.

ESP32 output (one line per sample):
  RAW,t_ms,ax,ay,az,gx,gy,gz[,esp_roll,esp_pitch,esp_yaw]

- ax/ay/az: int16 raw accelerometer
- gx/gy/gz: int16 raw gyro (already bias-corrected on ESP32 side)

This script computes Euler angles using:
1) Complementary filter (Euler integration + accel correction)
2) Mahony AHRS (IMU-only)

It then plots roll/pitch/yaw curves.

Dependencies:
  pip install pyserial numpy matplotlib

Example:
  python tools/compare_euler_algos.py --port COM7 --baud 115200 --seconds 20 \
    --accel-range-g 4 --gyro-range-dps 500 --out tools/raw_capture.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from typing import Optional, List, Tuple

import numpy as np
import serial
import matplotlib.pyplot as plt


@dataclass
class Sample:
    t_s: float
    ax: int
    ay: int
    az: int
    gx: int
    gy: int
    gz: int
    esp_r: Optional[float] = None
    esp_p: Optional[float] = None
    esp_y: Optional[float] = None


def parse_raw_line(line: str) -> Optional[Sample]:
    # Fast path for the expected CSV.
    if not line.startswith("RAW,"):
        return None

    parts = line.strip().split(",")
    if len(parts) < 8:
        return None

    try:
        t_ms = int(parts[1])
        ax = int(parts[2])
        ay = int(parts[3])
        az = int(parts[4])
        gx = int(parts[5])
        gy = int(parts[6])
        gz = int(parts[7])

        esp_r = esp_p = esp_y = None
        if len(parts) >= 11:
            esp_r = float(parts[8])
            esp_p = float(parts[9])
            esp_y = float(parts[10])

        return Sample(
            t_s=t_ms / 1000.0,
            ax=ax,
            ay=ay,
            az=az,
            gx=gx,
            gy=gy,
            gz=gz,
            esp_r=esp_r,
            esp_p=esp_p,
            esp_y=esp_y,
        )
    except ValueError:
        return None


def accel_angles_deg(ax_g: float, ay_g: float, az_g: float) -> Tuple[float, float]:
    # Matches the ESP C code:
    #   acc_roll = atan2(ay, az)
    #   acc_pitch = -atan2(ax, az)
    roll = math.degrees(math.atan2(ay_g, az_g))
    pitch = -math.degrees(math.atan2(ax_g, az_g))
    return roll, pitch


def quat_to_euler_deg(q0: float, q1: float, q2: float, q3: float) -> Tuple[float, float, float]:
    # Same convention as ESP code (roll/pitch/yaw)
    roll = math.degrees(math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)))
    pitch_arg = 2.0 * (q0 * q2 - q3 * q1)
    pitch_arg = max(-1.0, min(1.0, pitch_arg))
    pitch = math.degrees(math.asin(pitch_arg))
    yaw = math.degrees(math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)))
    return roll, pitch, yaw


class ComplementaryEuler:
    def __init__(self, alpha: float = 0.98):
        self.alpha = alpha
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.inited = False

    def update(self, dt: float, ax_g: float, ay_g: float, az_g: float, gx_dps: float, gy_dps: float, gz_dps: float):
        # Axis mapping matches ESP implementation:
        #   gyro_roll += gy * dt
        #   gyro_pitch += gx * dt
        if not self.inited:
            ar, ap = accel_angles_deg(ax_g, ay_g, az_g)
            self.roll_deg = ar
            self.pitch_deg = ap
            self.yaw_deg = 0.0
            self.inited = True
            return

        # Integrate gyro
        gyro_roll = self.roll_deg + gy_dps * dt
        gyro_pitch = self.pitch_deg + gx_dps * dt
        self.yaw_deg = self.yaw_deg + gz_dps * dt

        # Acc correction
        acc_roll, acc_pitch = accel_angles_deg(ax_g, ay_g, az_g)

        self.roll_deg = self.alpha * gyro_roll + (1.0 - self.alpha) * acc_roll
        self.pitch_deg = self.alpha * gyro_pitch + (1.0 - self.alpha) * acc_pitch


class MahonyIMU:
    def __init__(self, kp: float = 4.8, ki: float = 0.0015):
        self.kp = kp
        self.ki = ki
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.int_x = 0.0
        self.int_y = 0.0
        self.int_z = 0.0

    def update(self, dt: float, ax_g: float, ay_g: float, az_g: float, gx_rps: float, gy_rps: float, gz_rps: float):
        # Normalize accel
        acc_norm_sq = ax_g * ax_g + ay_g * ay_g + az_g * az_g
        if acc_norm_sq > 1e-6:
            inv = 1.0 / math.sqrt(acc_norm_sq)
            ax = ax_g * inv
            ay = ay_g * inv
            az = az_g * inv

            # Estimated gravity (from quaternion)
            q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
            vx = 2.0 * (q1 * q3 - q0 * q2)
            vy = 2.0 * (q0 * q1 + q2 * q3)
            vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

            # Error = cross(acc, v)
            ex = ay * vz - az * vy
            ey = az * vx - ax * vz
            ez = ax * vy - ay * vx

            # Integrate error
            if self.ki > 0.0:
                self.int_x += ex * dt
                self.int_y += ey * dt
                self.int_z += ez * dt
                gx_rps += self.ki * self.int_x
                gy_rps += self.ki * self.int_y
                gz_rps += self.ki * self.int_z

            # Proportional correction
            gx_rps += self.kp * ex
            gy_rps += self.kp * ey
            gz_rps += self.kp * ez

        # Quaternion derivative
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        q_dot0 = 0.5 * (-q1 * gx_rps - q2 * gy_rps - q3 * gz_rps)
        q_dot1 = 0.5 * (q0 * gx_rps + q2 * gz_rps - q3 * gy_rps)
        q_dot2 = 0.5 * (q0 * gy_rps - q1 * gz_rps + q3 * gx_rps)
        q_dot3 = 0.5 * (q0 * gz_rps + q1 * gy_rps - q2 * gx_rps)

        q0 += q_dot0 * dt
        q1 += q_dot1 * dt
        q2 += q_dot2 * dt
        q3 += q_dot3 * dt

        # Normalize
        n = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if n > 1e-9:
            q0 /= n
            q1 /= n
            q2 /= n
            q3 /= n

        self.q0, self.q1, self.q2, self.q3 = q0, q1, q2, q3

    def euler_deg(self) -> Tuple[float, float, float]:
        return quat_to_euler_deg(self.q0, self.q1, self.q2, self.q3)


def capture_samples(port: str, baud: int, seconds: float, warmup_s: float = 1.0) -> List[Sample]:
    samples: List[Sample] = []

    with serial.Serial(port=port, baudrate=baud, timeout=0.2) as ser:
        t0_wall = time.time()
        warmed = False

        while True:
            if time.time() - t0_wall > (seconds + warmup_s):
                break

            raw = ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="ignore")
            except Exception:
                continue

            s = parse_raw_line(line)
            if s is None:
                continue

            # Warmup: drop first warmup_s seconds (by wall time) to allow stable output
            if not warmed:
                if time.time() - t0_wall < warmup_s:
                    continue
                warmed = True

            samples.append(s)

    # Rebase time to start at 0
    if samples:
        t0 = samples[0].t_s
        for s in samples:
            s.t_s -= t0

    return samples


def compute_dt(times: np.ndarray) -> np.ndarray:
    if len(times) < 2:
        return np.array([], dtype=np.float64)
    dt = np.diff(times)
    # Replace any non-positive dt (rare parsing glitches) with median
    med = float(np.median(dt[dt > 0])) if np.any(dt > 0) else 0.01
    dt = np.where(dt > 0, dt, med)
    return dt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--warmup", type=float, default=1.0)
    ap.add_argument("--accel-range-g", type=float, default=4.0, help="ESP accel range in g, e.g. 2/4/8/16")
    ap.add_argument("--gyro-range-dps", type=float, default=500.0, help="ESP gyro range in dps, e.g. 250/500/1000/2000")
    ap.add_argument("--alpha", type=float, default=0.98, help="Complementary filter alpha")
    ap.add_argument("--mahony-kp", type=float, default=4.8)
    ap.add_argument("--mahony-ki", type=float, default=0.0015)
    ap.add_argument("--out", default="", help="Optional CSV path to save capture")
    args = ap.parse_args()

    print(f"Capturing {args.seconds}s from {args.port} @ {args.baud}...")
    samples = capture_samples(args.port, args.baud, args.seconds, warmup_s=args.warmup)
    if not samples:
        raise SystemExit("No RAW samples captured. Check ENABLE_RAW_UART_STREAM and port/baud.")

    t = np.array([s.t_s for s in samples], dtype=np.float64)
    dt = compute_dt(t)

    # Scales
    accel_g_per_lsb = args.accel_range_g / 32768.0
    gyro_dps_per_lsb = args.gyro_range_dps / 32768.0

    ax_g = np.array([s.ax for s in samples], dtype=np.float64) * accel_g_per_lsb
    ay_g = np.array([s.ay for s in samples], dtype=np.float64) * accel_g_per_lsb
    az_g = np.array([s.az for s in samples], dtype=np.float64) * accel_g_per_lsb

    gx_dps = np.array([s.gx for s in samples], dtype=np.float64) * gyro_dps_per_lsb
    gy_dps = np.array([s.gy for s in samples], dtype=np.float64) * gyro_dps_per_lsb
    gz_dps = np.array([s.gz for s in samples], dtype=np.float64) * gyro_dps_per_lsb

    # Optional ESP euler
    esp_has_euler = samples[0].esp_r is not None
    if esp_has_euler:
        esp_r = np.array([s.esp_r for s in samples], dtype=np.float64)
        esp_p = np.array([s.esp_p for s in samples], dtype=np.float64)
        esp_y = np.array([s.esp_y for s in samples], dtype=np.float64)

    comp = ComplementaryEuler(alpha=args.alpha)
    mahony = MahonyIMU(kp=args.mahony_kp, ki=args.mahony_ki)

    comp_r: List[float] = []
    comp_p: List[float] = []
    comp_y: List[float] = []

    mah_r: List[float] = []
    mah_p: List[float] = []
    mah_y: List[float] = []

    for i in range(len(samples)):
        if i == 0:
            dt_i = float(dt[0]) if len(dt) else 0.01
        else:
            dt_i = float(dt[i - 1])

        comp.update(dt_i, float(ax_g[i]), float(ay_g[i]), float(az_g[i]), float(gx_dps[i]), float(gy_dps[i]), float(gz_dps[i]))
        comp_r.append(comp.roll_deg)
        comp_p.append(comp.pitch_deg)
        comp_y.append(comp.yaw_deg)

        gx_rps_i = math.radians(float(gx_dps[i]))
        gy_rps_i = math.radians(float(gy_dps[i]))
        gz_rps_i = math.radians(float(gz_dps[i]))
        mahony.update(dt_i, float(ax_g[i]), float(ay_g[i]), float(az_g[i]), gx_rps_i, gy_rps_i, gz_rps_i)
        r, p, y = mahony.euler_deg()
        mah_r.append(r)
        mah_p.append(p)
        mah_y.append(y)

    if args.out:
        with open(args.out, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            header = ["t_s", "ax", "ay", "az", "gx", "gy", "gz"]
            if esp_has_euler:
                header += ["esp_roll", "esp_pitch", "esp_yaw"]
            w.writerow(header)
            for s in samples:
                row = [s.t_s, s.ax, s.ay, s.az, s.gx, s.gy, s.gz]
                if esp_has_euler:
                    row += [s.esp_r, s.esp_p, s.esp_y]
                w.writerow(row)
        print(f"Saved capture to: {args.out}")

    # Plot
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 9))
    axes[0].plot(t, comp_r, label=f"Complementary α={args.alpha}")
    axes[0].plot(t, mah_r, label=f"Mahony kp={args.mahony_kp} ki={args.mahony_ki}")
    if esp_has_euler:
        axes[0].plot(t, esp_r, label="ESP Euler (internal)", linestyle="--", alpha=0.7)
    axes[0].set_ylabel("Roll (deg)")
    axes[0].grid(True)
    axes[0].legend(loc="upper right")

    axes[1].plot(t, comp_p, label="Complementary")
    axes[1].plot(t, mah_p, label="Mahony")
    if esp_has_euler:
        axes[1].plot(t, esp_p, label="ESP Euler", linestyle="--", alpha=0.7)
    axes[1].set_ylabel("Pitch (deg)")
    axes[1].grid(True)

    axes[2].plot(t, comp_y, label="Complementary")
    axes[2].plot(t, mah_y, label="Mahony")
    if esp_has_euler:
        axes[2].plot(t, esp_y, label="ESP Euler", linestyle="--", alpha=0.7)
    axes[2].set_ylabel("Yaw (deg)")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True)

    fig.suptitle("Euler Angle Comparison from RAW IMU Stream")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
