# ESP32 + MPU6050 WebSocket 姿态发送示例

该项目基于 ESP-IDF，实现以下流程：

- ESP32 以 STA 模式连接 WiFi
- 初始化 MPU6050 并执行陀螺零偏校准
- 进行四元数姿态解算（`q0~q3`）
- 通过 WebSocket 周期发送姿态数据到服务器
- 定时发送 `ping` 并估算单向链路延迟 `lat_es`

## 目录结构

```text
.
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   ├── main.c
│   ├── mpu6050.c
│   └── mpu6050.h
└── sdkconfig
```

## 关键配置

在 `main/main.c` 中修改：

- `WIFI_SSID` / `WIFI_PASS`
- `WEBSOCKET_URI`
- `SAMPLE_RATE_HZ`（传感器采样率）

在 `main/mpu6050.h` 中修改硬件引脚：

- `MPU6050_I2C_SDA_GPIO`
- `MPU6050_I2C_SCL_GPIO`

## 构建与烧录（ESP-IDF）

先加载 ESP-IDF 环境，再执行：

```bash
idf.py set-target esp32c3
idf.py build
idf.py -p <PORT> flash monitor
```

> 如果命令找不到，请先在终端执行 ESP-IDF 的环境导出脚本（如 `export.ps1`）。

## 原始数据输出 + Python 解算对比

用于对比不同欧拉角解算算法（互补滤波、Mahony 等）对曲线的影响：ESP32 仅输出 MPU6050 原始数据（CSV 行），Python 端接收并解算/绘图。

### 1) 开启串口 RAW 输出

在 `main/main.c` 中将：

- `ENABLE_RAW_UART_STREAM` 从 `0` 改为 `1`
- 需要更高输出频率时可调整 `RAW_UART_STREAM_HZ`（默认 100Hz，考虑到 115200 串口带宽）

串口每行格式：

```text
RAW,t_ms,ax,ay,az,gx,gy,gz,esp_roll,esp_pitch,esp_yaw
```

其中 `ax..az/gx..gz` 为 int16 原始值（gyro 已在 ESP32 内做零偏扣除）。

### 2) Python 接收并绘制对比曲线

安装依赖：

```bash
pip install pyserial numpy matplotlib
```

运行（根据你的量程配置调整 `--accel-range-g/--gyro-range-dps`）：

```bash
python tools/compare_euler_algos.py --port COM7 --baud 115200 --seconds 20 \
	--accel-range-g 4 --gyro-range-dps 500 --out tools/raw_capture.csv
```

脚本会绘制 roll/pitch/yaw 三条对比曲线，并可选保存采集到的 CSV。
