#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_websocket_client.h"

#include "mpu6050.h"          // 您的 MPU6050 驱动头文件

// ==================== WiFi 配置 ====================
#define WIFI_SSID       "esp_test"
#define WIFI_PASS       "00008888"
#define MAX_RETRY       5

static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

static const char *TAG = "MAIN";

// ==================== WebSocket 配置 ====================
#define WEBSOCKET_URI   "ws://47.108.159.151:3000/esp" 

static esp_websocket_client_handle_t websocket_client;

// 采样率（Hz）
#define SAMPLE_RATE_HZ          200
#define SOLVER_HZ               SAMPLE_RATE_HZ
#define TX_HZ                   20

// ==================== 原始数据 UART 输出（用于 Python 离线/在线解算对比） ====================
// 为保证“不影响原来的代码逻辑”，默认关闭；需要时把 0 改为 1。
#define ENABLE_RAW_UART_STREAM  1
// 建议不要超过串口带宽；115200 下 100Hz 更稳，必要时可提高串口波特率再改到 200Hz。
#define RAW_UART_STREAM_HZ      100
// 输出格式：RAW,t_ms,ax,ay,az,gx,gy,gz（均为 int16 原始值；gyro 已减零偏）
// 可选附带当前 ESP32 内部解算欧拉角（单位：deg），便于对照。
#define RAW_UART_INCLUDE_EULER  1

// 全局保存最近一次测得的 ESP->Server 单向延迟 (ms)
static uint32_t g_ms_one_way = 0;
static MPU6050_t g_mpu_data;
static portMUX_TYPE g_mpu_lock = portMUX_INITIALIZER_UNLOCKED;

#if ENABLE_RAW_UART_STREAM
static void raw_uart_stream_task(void *pvParameters){
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t interval = pdMS_TO_TICKS(1000 / RAW_UART_STREAM_HZ);
    if (interval == 0) {
        interval = 1;
    }

    MPU6050_t mpu_snapshot;

    while (1) {
        taskENTER_CRITICAL(&g_mpu_lock);
        mpu_snapshot = g_mpu_data;
        taskEXIT_CRITICAL(&g_mpu_lock);

        uint32_t t_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

#if RAW_UART_INCLUDE_EULER
        // 用 printf 输出，避免 ESP_LOG 前缀干扰 Python 解析
        printf("RAW,%lu,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f\n",
               (unsigned long)t_ms,
               (int)mpu_snapshot.AccX, (int)mpu_snapshot.AccY, (int)mpu_snapshot.AccZ,
               (int)mpu_snapshot.GyroX, (int)mpu_snapshot.GyroY, (int)mpu_snapshot.GyroZ,
               mpu_snapshot.roll, mpu_snapshot.pitch, mpu_snapshot.yaw);
#else
        printf("RAW,%lu,%d,%d,%d,%d,%d,%d\n",
               (unsigned long)t_ms,
               (int)mpu_snapshot.AccX, (int)mpu_snapshot.AccY, (int)mpu_snapshot.AccZ,
               (int)mpu_snapshot.GyroX, (int)mpu_snapshot.GyroY, (int)mpu_snapshot.GyroZ);
#endif

        vTaskDelayUntil(&last_wake, interval);
    }
}
#endif

// ==================== WiFi 事件处理 ====================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected, retry...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void){
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization finished.");

    // 等待连接成功
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi connected");
}

// ==================== WebSocket 事件处理 ====================
static void websocket_event_handler(void *handler_args, esp_event_base_t base,
                                    int32_t event_id, void *event_data){
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WebSocket connected");
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "WebSocket disconnected");
        break;
    case WEBSOCKET_EVENT_DATA: {
        ESP_LOGD(TAG, "Received: %.*s", data->data_len, (char *)data->data_ptr);

        char msg[64] = {0};
        int copy_len = data->data_len < (int)sizeof(msg) - 1
                       ? data->data_len
                       : (int)sizeof(msg) - 1;
        memcpy(msg, data->data_ptr, copy_len);

        if (strstr(msg, "\"type\":\"pong\"")) {
            uint32_t tick_sent = 0;
            if (sscanf(msg, "{\"type\":\"pong\",\"tick\":%lu}", &tick_sent) == 1) {
                uint32_t tick_now  = (uint32_t)xTaskGetTickCount();
                uint32_t tick_diff = tick_now - tick_sent;
                uint32_t ms_rtt    = tick_diff * portTICK_PERIOD_MS;
                uint32_t ms_one_way = ms_rtt / 2;

                g_ms_one_way = ms_one_way;  // ★ 更新全局延迟

                ESP_LOGI("LAT", "ESP->Server RTT=%lu ms, one-way≈%lu ms",
                         (unsigned long)ms_rtt, (unsigned long)ms_one_way);
            }
        }
        break;
    }
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "WebSocket error");
        break;
    default:
        break;
    }
}

static void websocket_init(void){
    esp_websocket_client_config_t websocket_config = {
        .uri = WEBSOCKET_URI,
        .task_stack = 4096,
        .task_prio = tskIDLE_PRIORITY + 5,
        .buffer_size = 1024,
        .reconnect_timeout_ms = 5000,
        .network_timeout_ms = 5000,
        .disable_auto_reconnect = false,   // 启用自动重连
    };
    websocket_client = esp_websocket_client_init(&websocket_config);
    ESP_ERROR_CHECK(esp_websocket_register_events(websocket_client,
                                                  WEBSOCKET_EVENT_ANY,
                                                  websocket_event_handler,
                                                  NULL));

    // ★ 启动 WebSocket 连接
    ESP_ERROR_CHECK(esp_websocket_client_start(websocket_client));
}

// ==================== 传感器数据发送任务 ====================
static void data_transmission_task(void *pvParameters){
    char json_buffer[200];
    MPU6050_t mpu_snapshot;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(1000 / TX_HZ);

    while (1) {
        uint32_t lat_es = g_ms_one_way;
        taskENTER_CRITICAL(&g_mpu_lock);
        mpu_snapshot = g_mpu_data;
        taskEXIT_CRITICAL(&g_mpu_lock);

        int len = snprintf(json_buffer, sizeof(json_buffer),
                           "{\"type\":\"quat\","
                           "\"q0\":%.6f,\"q1\":%.6f,\"q2\":%.6f,\"q3\":%.6f,"
                           "\"lat_es\":%lu}",
                           mpu_snapshot.q0, mpu_snapshot.q1, mpu_snapshot.q2, mpu_snapshot.q3,
                           (unsigned long)lat_es);

        if (esp_websocket_client_is_connected(websocket_client)) {
            ESP_LOGD("TASK", "Send: %s", json_buffer);
            esp_websocket_client_send_text(websocket_client,
                                           json_buffer, len, pdMS_TO_TICKS(10));
        } else {
            ESP_LOGW("TASK", "WebSocket not connected");
        }

        vTaskDelayUntil(&last_wake, interval);
    }
}

static void attitude_log_task(void *pvParameters){
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(500);
    MPU6050_t mpu_snapshot;

    while (1) {
        taskENTER_CRITICAL(&g_mpu_lock);
        mpu_snapshot = g_mpu_data;
        taskEXIT_CRITICAL(&g_mpu_lock);

        ESP_LOGI("ATT", "R=%.2f P=%.2f Y=%.2f drift=%.4f dps",
                 mpu_snapshot.roll, mpu_snapshot.pitch, mpu_snapshot.yaw,
                 mpu6050_get_yaw_drift_rate_dps());
        vTaskDelayUntil(&last_wake, interval);
    }
}

static void attitude_solver_task(void *pvParameters){
    MPU6050_t local_data = {0};
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t interval = pdMS_TO_TICKS(1000 / SOLVER_HZ);
    if (interval == 0) {
        interval = 1;
    }

    while (1) {
        mpu6050_get_angle_plus(&local_data);
        taskENTER_CRITICAL(&g_mpu_lock);
        g_mpu_data = local_data;
        taskEXIT_CRITICAL(&g_mpu_lock);
        vTaskDelayUntil(&last_wake, interval);
    }
}

// ==================== 延迟测量任务（ESP32 -> 服务器 RTT） ====================
static void latency_task(void *pvParameters){
    char buf[64];
    
    vTaskDelay(pdMS_TO_TICKS(500));
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(5000); // 每 5 秒测一次

    while (1) {
        if (esp_websocket_client_is_connected(websocket_client)) {
            uint32_t t_start = (uint32_t)xTaskGetTickCount(); // tick 计数
            // 发送心跳，携带 tick
            // 这里的 %u 也改成 %lu
            int len = snprintf(buf, sizeof(buf),
                               "{\"type\":\"ping\",\"tick\":%lu}", (unsigned long)t_start);
            esp_websocket_client_send_text(websocket_client, buf, len, pdMS_TO_TICKS(10));
            // 以及这里
            ESP_LOGI("LAT", "Send ping, tick=%lu", (unsigned long)t_start);
        } else {
            ESP_LOGW("LAT", "WebSocket not connected");
        }

        vTaskDelayUntil(&last_wake, interval);
    }
}

// ==================== 主函数 ====================
void app_main(void){
    // 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化 WiFi（阻塞直到连接成功）
    wifi_init_sta();

    // 初始化 MPU6050
    mpu6050_config_t mpu_cfg = {
        .sample_rate_hz = SAMPLE_RATE_HZ,      // 200Hz
        .filter        = Band_43Hz,           // 根据需要调整
        .gyro_range    = gyro_500,
        .accel_range   = acc_4g,
        .use_fifo      = false,
        .use_interrupt = false,
    };
    ESP_ERROR_CHECK(mpu6050_init(&mpu_cfg));
    mpu6050_reset_yaw_drift_estimator();

    // 可选：设置当前姿态为零点
    mpu6050_set_angle_zero(&g_mpu_data);
    // 初始化 WebSocket
    websocket_init();

    // 先启动 200Hz 姿态解算，再按 20Hz 发送
    xTaskCreate(attitude_solver_task, "solver_task", 4096, NULL, 6, NULL);

#if ENABLE_RAW_UART_STREAM
    // 原始数据输出（供 Python 接收/对比算法），不影响原有 WebSocket 逻辑
    xTaskCreate(raw_uart_stream_task, "raw_uart", 3072, NULL, 3, NULL);
#endif

    // 姿态日志输出（含自适应漂移估计）
    //xTaskCreate(attitude_log_task, "att_log", 3072, NULL, 4, NULL);
    // 创建数据传输任务
    xTaskCreate(data_transmission_task, "data_task", 4096, NULL, 5, NULL);
    // 创建延迟测量任务
    xTaskCreate(latency_task, "lat_task", 3072, NULL, 4, NULL);

    ESP_LOGI(TAG, "System initialized, main task ended.");
}
