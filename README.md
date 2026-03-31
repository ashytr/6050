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
