/**
 * @file    main.c
 * @brief   ESP32-C3 应用入口 —— WiFi STA、WebSocket 客户端、
 *          MPU6050 传感器融合、数据流推送至远程服务器。
 *
 * 任务架构（FreeRTOS 抢占式优先级调度）
 * ======================================
 * 优先级 6 —— solver_task     200 Hz   MPU6050 Mahony AHRS 姿态解算
 * 优先级 5 —— data_task        20 Hz   JSON 四元数 → WebSocket
 * 优先级 4 —— lat_task          0.2 Hz WebSocket ping/pong RTT 探测
 * 优先级 3 —— raw_uart_task   100 Hz   原始 CSV 经 UART 输出（可选）
 *
 * 200 Hz → 20 Hz 的解耦是本设计的核心决策：解算任务高速运行以保证
 * 积分精度，而无线发送任务以较低频率运行以避免拥堵 WiFi 链路。
 * 临界区自旋锁保护 solver_task 与消费者之间的共享 g_mpu_data 结构。
 *
 * 延迟测量
 * ========
 * 每 5 秒，lat_task 发送 {"type":"ping","tick":<FreeRTOS tick>}。
 * 服务器原样回传 {"type":"pong","tick":<相同 tick>}。收到后计算
 * RTT = (tick_now - tick_sent) × portTICK_PERIOD_MS，
 * 单向估计 = RTT / 2。该估计值作为 "lat_es" 附加到每条四元数消息中，
 * 使浏览器能计算端到端总延迟。
 */
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
#include "cJSON.h"

#include "mpu6050.h"

// ---- WiFi 配置 —— 可通过编译时 -D 覆盖 -----------------------------------
#ifndef WIFI_SSID
#define WIFI_SSID       "esp_test"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS       "00008888"
#endif
#ifndef WEBSOCKET_URI
#define WEBSOCKET_URI   "ws://47.108.159.151:3000/esp"
#endif

#define MAX_RETRY       5

static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
static int wifi_retry_count = 0;

static const char *TAG = "MAIN";

static esp_websocket_client_handle_t websocket_client;

// ---- 采样率 (Hz) ----------------------------------------------------------
#define SAMPLE_RATE_HZ          200    // MPU6050 内部采样率
#define SOLVER_HZ               SAMPLE_RATE_HZ  // 姿态解算频率
#define TX_HZ                   20     // WebSocket 发送频率

// ---- 原始 UART 数据流（用于 Python 端算法对比）----------------------------
// 输出格式：RAW,t_ms,ax,ay,az,gx,gy,gz
// 所有值为 int16 原始 ADC；陀螺仪已扣除零偏。
// 设为 1 启用；115200 波特率下 100 Hz 安全。
#define ENABLE_RAW_UART_STREAM  1
#define RAW_UART_STREAM_HZ      100

// ---- 共享状态（由 g_mpu_lock 自旋锁保护）----------------------------------
static uint32_t g_ms_one_way = 0;   // 最近一次估计的单向延迟 (ms)
static MPU6050_t g_mpu_data;        // 最新的姿态采样
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

        // 用 printf 输出，避免 ESP_LOG 前缀干扰解析
        printf("RAW,%lu,%d,%d,%d,%d,%d,%d\n",
               (unsigned long)t_ms,
               (int)mpu_snapshot.AccX, (int)mpu_snapshot.AccY, (int)mpu_snapshot.AccZ,
               (int)mpu_snapshot.GyroX, (int)mpu_snapshot.GyroY, (int)mpu_snapshot.GyroZ);

        vTaskDelayUntil(&last_wake, interval);
    }
}
#endif

// WiFi 事件处理 —— 带指数退避重连
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_retry_count++;
        int backoff_s = (wifi_retry_count < 6) ? (1 << (wifi_retry_count - 1)) : 60;
        ESP_LOGI(TAG, "WiFi disconnected, retry #%d in %ds...", wifi_retry_count, backoff_s);
        vTaskDelay(pdMS_TO_TICKS(backoff_s * 1000));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_retry_count = 0;
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

// ---- 远程命令处理 --------------------------------------------------------
// 解析来自浏览器/服务端的 cmd 消息，执行对应操作并返回 ack。
// 支持的命令：get_config（查询参数）、zero_calibrate（零位校准）。
// 使用 cJSON 做结构化解析，避免 strstr 的误匹配。
static void handle_cmd(const char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGW(TAG, "Failed to parse cmd JSON");
        return;
    }

    cJSON *cmd_item = cJSON_GetObjectItem(root, "cmd");
    if (!cmd_item || !cJSON_IsString(cmd_item)) {
        ESP_LOGW(TAG, "Cmd missing or invalid type");
        cJSON_Delete(root);
        return;
    }

    const char *cmd = cmd_item->valuestring;

    if (strcmp(cmd, "zero_calibrate") == 0) {
        ESP_LOGI(TAG, "Remote: zero_calibrate");
        MPU6050_t tmp;
        mpu6050_set_angle_zero(&tmp);

        const char *ack = "{\"type\":\"cmd_ack\",\"cmd\":\"zero_calibrate\","
                          "\"status\":\"ok\",\"message\":\"Calibration done\"}";
        esp_websocket_client_send_text(websocket_client, ack, strlen(ack),
                                       pdMS_TO_TICKS(100));
    }
    else if (strcmp(cmd, "get_config") == 0) {
        ESP_LOGI(TAG, "Remote: get_config");
        char ack[256];
        snprintf(ack, sizeof(ack),
            "{\"type\":\"cmd_ack\",\"cmd\":\"get_config\",\"status\":\"ok\","
            "\"params\":{"
            "\"sample_rate_hz\":%d,"
            "\"gyro_range\":\"%s\","
            "\"accel_range\":\"%s\","
            "\"filter\":\"%s\","
            "\"wifi_ssid\":\"%s\","
            "\"server_uri\":\"%s\","
            "\"tx_hz\":%d,"
            "\"solver_hz\":%d"
            "}}",
            SAMPLE_RATE_HZ,
            "±500 deg/s",
            "±4g",
            "43 Hz",
            WIFI_SSID,
            WEBSOCKET_URI,
            TX_HZ,
            SOLVER_HZ
        );
        esp_websocket_client_send_text(websocket_client, ack, strlen(ack),
                                       pdMS_TO_TICKS(100));
    }
    else {
        ESP_LOGW(TAG, "Unknown cmd: %s", cmd);
        const char *ack = "{\"type\":\"cmd_ack\",\"cmd\":\"unknown\","
                          "\"status\":\"error\",\"message\":\"Unknown command\"}";
        esp_websocket_client_send_text(websocket_client, ack, strlen(ack),
                                       pdMS_TO_TICKS(100));
    }

    cJSON_Delete(root);
}

// WebSocket 事件处理
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

        char msg[256] = {0};
        int copy_len = data->data_len < (int)sizeof(msg) - 1
                       ? data->data_len
                       : (int)sizeof(msg) - 1;
        memcpy(msg, data->data_ptr, copy_len);

        // 使用 cJSON 做结构化解析，替代 strstr 的脆弱匹配
        cJSON *root = cJSON_Parse(msg);
        if (!root) break;

        cJSON *type_item = cJSON_GetObjectItem(root, "type");
        if (!type_item || !cJSON_IsString(type_item)) {
            cJSON_Delete(root);
            break;
        }

        const char *msg_type = type_item->valuestring;

        if (strcmp(msg_type, "pong") == 0) {
            cJSON *tick_item = cJSON_GetObjectItem(root, "tick");
            if (tick_item && cJSON_IsNumber(tick_item)) {
                uint32_t tick_sent = (uint32_t)tick_item->valuedouble;
                uint32_t tick_now  = (uint32_t)xTaskGetTickCount();
                uint32_t tick_diff = tick_now - tick_sent;
                uint32_t ms_rtt    = tick_diff * portTICK_PERIOD_MS;
                uint32_t ms_one_way = ms_rtt / 2;

                g_ms_one_way = ms_one_way;

                ESP_LOGI("LAT", "ESP->Server RTT=%lu ms, one-way≈%lu ms",
                         (unsigned long)ms_rtt, (unsigned long)ms_one_way);
            }
        }
        else if (strcmp(msg_type, "cmd") == 0) {
            handle_cmd(msg);
        }

        cJSON_Delete(root);
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
        .disable_auto_reconnect = false,  // 启用自动重连
    };
    websocket_client = esp_websocket_client_init(&websocket_config);
    ESP_ERROR_CHECK(esp_websocket_register_events(websocket_client,
                                                  WEBSOCKET_EVENT_ANY,
                                                  websocket_event_handler,
                                                  NULL));

    // 启动 WebSocket 连接
    ESP_ERROR_CHECK(esp_websocket_client_start(websocket_client));
}

// 传感器数据发送任务
static void data_transmission_task(void *pvParameters){
    char json_buffer[256];
    MPU6050_t mpu_snapshot;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(1000 / TX_HZ);

    while (1) {
        uint32_t lat_es = g_ms_one_way;
        taskENTER_CRITICAL(&g_mpu_lock);
        mpu_snapshot = g_mpu_data;
        taskEXIT_CRITICAL(&g_mpu_lock);

        mpu6050_get_temp(&mpu_snapshot);

        uint32_t uptime_s = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS / 1000);
        int len = snprintf(json_buffer, sizeof(json_buffer),
                           "{\"type\":\"quat\","
                           "\"q0\":%.6f,\"q1\":%.6f,\"q2\":%.6f,\"q3\":%.6f,"
                           "\"lat_es\":%lu,"
                           "\"temp\":%.2f,"
                           "\"uptime\":%lu}",
                           mpu_snapshot.q0, mpu_snapshot.q1, mpu_snapshot.q2, mpu_snapshot.q3,
                           (unsigned long)lat_es,
                           mpu_snapshot.temp,
                           (unsigned long)uptime_s);

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

// I2C 看门狗任务 —— 优先级 7，高于 solver_task(6)。
// 若 MPU6050 拉死 SDA 导致 I2C 驱动层死循环，solver_task 将永久
// 阻塞且心跳停止。本任务每 500ms 检查心跳；连续 3 次无心跳则强制
// 复位 I2C 总线并重新添加设备，恢复后心跳自动恢复。
static void i2c_watchdog_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(2000));  // 等待系统完全启动

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        if (!mpu6050_i2c_heartbeat_ok()) {
            if (mpu6050_i2c_is_stuck()) {
                ESP_LOGE("I2C_WDT", "Solver stuck! Forcing I2C recovery...");
                mpu6050_i2c_force_recover();
            }
        }
    }
}

// 延迟测量任务（ESP32 -> 服务器 RTT）
static void latency_task(void *pvParameters){
    char buf[64];
    
    vTaskDelay(pdMS_TO_TICKS(500));
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(5000); // 每 5 秒测一次

    while (1) {
        if (esp_websocket_client_is_connected(websocket_client)) {
            uint32_t t_start = (uint32_t)xTaskGetTickCount();
            int len = snprintf(buf, sizeof(buf),
                               "{\"type\":\"ping\",\"tick\":%lu}", (unsigned long)t_start);
            esp_websocket_client_send_text(websocket_client, buf, len, pdMS_TO_TICKS(10));
            ESP_LOGI("LAT", "Send ping, tick=%lu", (unsigned long)t_start);
        } else {
            ESP_LOGW("LAT", "WebSocket not connected");
        }

        vTaskDelayUntil(&last_wake, interval);
    }
}


// =========================================================================
// 启动流程
// ========
// NVS → WiFi（阻塞至获取 IP）→ MPU6050 初始化 + 归零校准 →
// WebSocket 连接 → 创建 FreeRTOS 任务 → main 返回（任务持续运行）。
// =========================================================================
void app_main(void){
    // NVS 是 WiFi 必需的；如果分区损坏则擦除后重新初始化。
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

    // 设置当前姿态为零点
    mpu6050_set_angle_zero(&g_mpu_data);
    // 初始化 WebSocket
    websocket_init();

    // I2C 看门狗 —— 优先级 7，高于 solver(6)，能在 I2C 卡死时
    // 抢占并复位总线，防止 solver 永久阻塞触发系统 WDT。
    xTaskCreate(i2c_watchdog_task, "i2c_wdt", 2048, NULL, 7, NULL);

    // 先启动 200Hz 姿态解算，再按 20Hz 发送
    xTaskCreate(attitude_solver_task, "solver_task", 4096, NULL, 6, NULL);

#if ENABLE_RAW_UART_STREAM
    // 原始数据输出（供 Python 接收/对比算法）
    xTaskCreate(raw_uart_stream_task, "raw_uart", 3072, NULL, 3, NULL);
#endif
    // 创建数据传输任务
    xTaskCreate(data_transmission_task, "data_task", 4096, NULL, 5, NULL);
    // 创建延迟测量任务
    xTaskCreate(latency_task, "lat_task", 3072, NULL, 4, NULL);

    ESP_LOGI(TAG, "System initialized, main task ended.");
}
