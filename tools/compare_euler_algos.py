#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
从 ESP32 串口接收 RAW IMU 数据，并对比不同算法的欧拉角解算曲线。

ESP32 输出格式（每行 1 个样本）：
    RAW,t_ms,ax,ay,az,gx,gy,gz

- ax/ay/az：int16 加速度计原始值
- gx/gy/gz：int16 陀螺仪原始值（已在 ESP32 侧做零偏扣除）

本脚本在 Python 端计算三套欧拉角（单位：deg），用于对比不同算法的差异：
1) 互补滤波（欧拉角积分 + 加速度修正）
    - 预测：用陀螺角速度积分得到角度变化
    - 观测：用加速度计估算的重力方向修正 roll/pitch
    - 特点：实现简单、跟手；但 yaw 没有观测量，只能陀螺积分，时间久会漂移

2) Mahony AHRS（仅 IMU，无磁力计）
    - 在四元数域融合陀螺 + 加速度（通过 PI 校正重力方向误差）
    - 特点：roll/pitch 通常更稳；但 yaw 同样缺少磁力计观测，仍会漂移

3) 卡尔曼滤波（经典 1 维 Kalman：状态=[角度, 陀螺零偏]）
    - roll/pitch：用加速度角作为观测，用陀螺角速度作为预测，同时估计陀螺零偏
    - yaw：没有磁力计观测，本脚本里 yaw 仍是陀螺积分（图例会标注“航向=陀螺积分”）

补充说明：
- 本脚本的轴映射与 ESP32 侧示例代码保持一致（roll 主要由 GyroY 积分、pitch 主要由 GyroX 积分）。
- RAW 数据的量程换算由参数 --accel-range-g / --gyro-range-dps 控制（需与你在 ESP32 端配置一致）。

并绘制 roll/pitch/yaw 三轴曲线。

依赖：
    pip install pyserial numpy matplotlib

示例：
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


# Matplotlib 中文显示：按顺序尝试常见中文字体，避免图例/标题乱码
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial Unicode MS"]
plt.rcParams["axes.unicode_minus"] = False


@dataclass
class Sample:
    t_s: float
    ax: int
    ay: int
    az: int
    gx: int
    gy: int
    gz: int


def parse_raw_line(line: str) -> Optional[Sample]:
    # 仅解析我们期望的 CSV 行
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
        # 说明：如果 ESP32 行尾带了额外列，这里会自动忽略，不影响解析。
        return Sample(
            t_s=t_ms / 1000.0,
            ax=ax,
            ay=ay,
            az=az,
            gx=gx,
            gy=gy,
            gz=gz,
        )
    except ValueError:
        return None


def accel_angles_deg(ax_g: float, ay_g: float, az_g: float) -> Tuple[float, float]:
    # 与 ESP32 侧 C 代码保持一致：
    #   acc_roll = atan2(ay, az)
    #   acc_pitch = -atan2(ax, az)
    roll = math.degrees(math.atan2(ay_g, az_g))
    pitch = -math.degrees(math.atan2(ax_g, az_g))
    return roll, pitch


def quat_to_euler_deg(q0: float, q1: float, q2: float, q3: float) -> Tuple[float, float, float]:
    # 与 ESP32 侧相同的欧拉角约定（roll/pitch/yaw）
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
        # 轴映射与 ESP32 侧实现保持一致：
        #   gyro_roll  += gy * dt
        #   gyro_pitch += gx * dt
        if not self.inited:
            ar, ap = accel_angles_deg(ax_g, ay_g, az_g)
            self.roll_deg = ar
            self.pitch_deg = ap
            self.yaw_deg = 0.0
            self.inited = True
            return

        # 陀螺积分
        gyro_roll = self.roll_deg + gy_dps * dt
        gyro_pitch = self.pitch_deg + gx_dps * dt
        self.yaw_deg = self.yaw_deg + gz_dps * dt

        # 加速度修正
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
        # 加速度归一化
        acc_norm_sq = ax_g * ax_g + ay_g * ay_g + az_g * az_g
        if acc_norm_sq > 1e-6:
            inv = 1.0 / math.sqrt(acc_norm_sq)
            ax = ax_g * inv
            ay = ay_g * inv
            az = az_g * inv

            # 由四元数估计重力方向
            q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
            vx = 2.0 * (q1 * q3 - q0 * q2)
            vy = 2.0 * (q0 * q1 + q2 * q3)
            vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

            # 误差：acc 与 v 的叉乘
            ex = ay * vz - az * vy
            ey = az * vx - ax * vz
            ez = ax * vy - ay * vx

            # 误差积分项
            if self.ki > 0.0:
                self.int_x += ex * dt
                self.int_y += ey * dt
                self.int_z += ez * dt
                gx_rps += self.ki * self.int_x
                gy_rps += self.ki * self.int_y
                gz_rps += self.ki * self.int_z

            # 比例项
            gx_rps += self.kp * ex
            gy_rps += self.kp * ey
            gz_rps += self.kp * ez

        # 四元数微分方程
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        q_dot0 = 0.5 * (-q1 * gx_rps - q2 * gy_rps - q3 * gz_rps)
        q_dot1 = 0.5 * (q0 * gx_rps + q2 * gz_rps - q3 * gy_rps)
        q_dot2 = 0.5 * (q0 * gy_rps - q1 * gz_rps + q3 * gx_rps)
        q_dot3 = 0.5 * (q0 * gz_rps + q1 * gy_rps - q2 * gx_rps)

        q0 += q_dot0 * dt
        q1 += q_dot1 * dt
        q2 += q_dot2 * dt
        q3 += q_dot3 * dt

        # 归一化
        n = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if n > 1e-9:
            q0 /= n
            q1 /= n
            q2 /= n
            q3 /= n

        self.q0, self.q1, self.q2, self.q3 = q0, q1, q2, q3

    def euler_deg(self) -> Tuple[float, float, float]:
        return quat_to_euler_deg(self.q0, self.q1, self.q2, self.q3)


class Kalman1DAngle:
    """经典 1D Kalman：状态为 [角度, 陀螺零偏]。

    - 预测：用 gyro_rate（deg/s）积分角度
    - 观测：用 accel 计算的角度（deg）
    """

    def __init__(self, q_angle: float = 0.001, q_bias: float = 0.003, r_measure: float = 0.03):
        self.q_angle = q_angle
        self.q_bias = q_bias
        self.r_measure = r_measure

        self.angle_deg = 0.0
        self.bias_dps = 0.0
        # 协方差矩阵 P（2x2）
        self.p00 = 0.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 0.0
        self.inited = False

    def set_angle(self, angle_deg: float):
        self.angle_deg = angle_deg
        self.inited = True

    def update(self, dt: float, gyro_rate_dps: float, meas_angle_deg: float) -> float:
        if not self.inited:
            self.set_angle(meas_angle_deg)

        # 1) 预测
        rate = gyro_rate_dps - self.bias_dps
        self.angle_deg += dt * rate

        # P = A P A^T + Q
        # A = [[1, -dt],[0, 1]]
        p00 = self.p00 + dt * (dt * self.p11 - self.p01 - self.p10) + self.q_angle
        p01 = self.p01 - dt * self.p11
        p10 = self.p10 - dt * self.p11
        p11 = self.p11 + self.q_bias
        self.p00, self.p01, self.p10, self.p11 = p00, p01, p10, p11

        # 2) 更新
        # K = P H^T / (H P H^T + R),  H = [1, 0]
        s = self.p00 + self.r_measure
        if s <= 1e-12:
            return self.angle_deg
        k0 = self.p00 / s
        k1 = self.p10 / s

        y = meas_angle_deg - self.angle_deg
        self.angle_deg += k0 * y
        self.bias_dps += k1 * y

        # P = (I - K H) P
        p00 = self.p00
        p01 = self.p01
        p10 = self.p10
        p11 = self.p11
        self.p00 = p00 - k0 * p00
        self.p01 = p01 - k0 * p01
        self.p10 = p10 - k1 * p00
        self.p11 = p11 - k1 * p01

        return self.angle_deg


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

            # 预热：丢弃前 warmup_s 秒（按墙钟时间），让输出先稳定
            if not warmed:
                if time.time() - t0_wall < warmup_s:
                    continue
                warmed = True

            samples.append(s)

    # 时间从 0 开始
    if samples:
        t0 = samples[0].t_s
        for s in samples:
            s.t_s -= t0

    return samples


def compute_dt(times: np.ndarray) -> np.ndarray:
    if len(times) < 2:
        return np.array([], dtype=np.float64)
    dt = np.diff(times)
    # 将非正 dt（偶发解析/串口抖动）替换为中位数
    med = float(np.median(dt[dt > 0])) if np.any(dt > 0) else 0.01
    dt = np.where(dt > 0, dt, med)
    return dt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="串口号，例如 COM7")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=15.0, help="采集时长（秒）")
    ap.add_argument("--warmup", type=float, default=1.0, help="预热丢弃时长（秒）")
    ap.add_argument("--accel-range-g", type=float, default=4.0, help="ESP32 端加速度量程（g），如 2/4/8/16")
    ap.add_argument("--gyro-range-dps", type=float, default=500.0, help="ESP32 端陀螺量程（dps），如 250/500/1000/2000")
    ap.add_argument("--alpha", type=float, default=0.98, help="互补滤波系数 α（越大越信陀螺）")
    ap.add_argument("--mahony-kp", type=float, default=4.8)
    ap.add_argument("--mahony-ki", type=float, default=0.0015)
    ap.add_argument("--kalman-q-angle", type=float, default=0.001, help="卡尔曼：角度过程噪声 Q_angle")
    ap.add_argument("--kalman-q-bias", type=float, default=0.003, help="卡尔曼：零偏过程噪声 Q_bias")
    ap.add_argument("--kalman-r-measure", type=float, default=0.03, help="卡尔曼：观测噪声 R_measure")
    ap.add_argument("--out", default="", help="可选：保存采集数据到 CSV 文件")
    args = ap.parse_args()

    print(f"开始采集：{args.port} @ {args.baud}，时长 {args.seconds}s ...")
    samples = capture_samples(args.port, args.baud, args.seconds, warmup_s=args.warmup)
    if not samples:
        raise SystemExit("没有采集到 RAW 数据：请检查 ESP32 是否开启 ENABLE_RAW_UART_STREAM，以及串口号/波特率是否正确。")

    t = np.array([s.t_s for s in samples], dtype=np.float64)
    dt = compute_dt(t)

    # 量程换算
    accel_g_per_lsb = args.accel_range_g / 32768.0
    gyro_dps_per_lsb = args.gyro_range_dps / 32768.0

    ax_g = np.array([s.ax for s in samples], dtype=np.float64) * accel_g_per_lsb
    ay_g = np.array([s.ay for s in samples], dtype=np.float64) * accel_g_per_lsb
    az_g = np.array([s.az for s in samples], dtype=np.float64) * accel_g_per_lsb

    gx_dps = np.array([s.gx for s in samples], dtype=np.float64) * gyro_dps_per_lsb
    gy_dps = np.array([s.gy for s in samples], dtype=np.float64) * gyro_dps_per_lsb
    gz_dps = np.array([s.gz for s in samples], dtype=np.float64) * gyro_dps_per_lsb

    comp = ComplementaryEuler(alpha=args.alpha)
    mahony = MahonyIMU(kp=args.mahony_kp, ki=args.mahony_ki)
    kal_roll = Kalman1DAngle(q_angle=args.kalman_q_angle, q_bias=args.kalman_q_bias, r_measure=args.kalman_r_measure)
    kal_pitch = Kalman1DAngle(q_angle=args.kalman_q_angle, q_bias=args.kalman_q_bias, r_measure=args.kalman_r_measure)
    kal_yaw_deg = 0.0

    comp_r: List[float] = []
    comp_p: List[float] = []
    comp_y: List[float] = []

    mah_r: List[float] = []
    mah_p: List[float] = []
    mah_y: List[float] = []

    kal_r: List[float] = []
    kal_p: List[float] = []
    kal_y: List[float] = []

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

        # 卡尔曼：roll/pitch 用加速度角作观测；yaw 无观测，仅陀螺积分（会漂移）
        acc_r, acc_p = accel_angles_deg(float(ax_g[i]), float(ay_g[i]), float(az_g[i]))
        k_r = kal_roll.update(dt_i, float(gy_dps[i]), acc_r)
        k_p = kal_pitch.update(dt_i, float(gx_dps[i]), acc_p)
        kal_yaw_deg += float(gz_dps[i]) * dt_i
        kal_r.append(k_r)
        kal_p.append(k_p)
        kal_y.append(kal_yaw_deg)

    if args.out:
        with open(args.out, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            header = ["t_s", "ax", "ay", "az", "gx", "gy", "gz"]
            w.writerow(header)
            for s in samples:
                row = [s.t_s, s.ax, s.ay, s.az, s.gx, s.gy, s.gz]
                w.writerow(row)
        print(f"已保存采集数据到：{args.out}")

    # 绘图
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 9))
    axes[0].plot(t, comp_r, label=f"互补滤波（α={args.alpha}）")
    axes[0].plot(t, mah_r, label=f"Mahony（kp={args.mahony_kp}，ki={args.mahony_ki}）")
    axes[0].plot(t, kal_r, label="卡尔曼滤波")
    axes[0].set_ylabel("横滚 Roll（度）")
    axes[0].grid(True)
    axes[0].legend(loc="upper right")

    axes[1].plot(t, comp_p, label="互补滤波")
    axes[1].plot(t, mah_p, label="Mahony")
    axes[1].plot(t, kal_p, label="卡尔曼滤波")
    axes[1].set_ylabel("俯仰 Pitch（度）")
    axes[1].grid(True)

    axes[2].plot(t, comp_y, label="互补滤波")
    axes[2].plot(t, mah_y, label="Mahony")
    axes[2].plot(t, kal_y, label="卡尔曼（航向=陀螺积分）")
    axes[2].set_ylabel("航向 Yaw（度）")
    axes[2].set_xlabel("时间（秒）")
    axes[2].grid(True)

    fig.suptitle("RAW IMU 数据：不同欧拉角解算算法曲线对比")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
