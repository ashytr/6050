#include "mpu6050.h"

static const char *TAG = "MPU6050";

// I2C总线和设备句柄
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t mpu6050_dev_handle = NULL;

// 内部状态
static float mpu6050_dt = 0.005f;   // 默认200Hz -> 5ms
static float gyro_scale = 0.0f;     // 弧度/LSB
static float accel_scale = 0.0f;    // g/LSB
static bool mpu6050_inited = false;

// 零点校准值
static int16_t gyro_zero_x = 0, gyro_zero_y = 0, gyro_zero_z = 0;

// 角度偏移（用于归零）
static float angle_yaw_offset = 0;
static float angle_roll_offset = 0;
static float angle_pitch_offset = 0;
static float yaw_drift_rate_dps = 0.0f;

#if MPU6050_USE_FILTER
static pt1_filter_t pt1_acc_x, pt1_acc_y, pt1_acc_z;
static pt1_filter_t pt1_gyro_x, pt1_gyro_y, pt1_gyro_z;
#endif

// 延时函数（毫秒）
#define mpu6050_delay_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))

// 快速平方根倒数
static inline float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    int32_t i = *(int32_t *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// I2C写寄存器
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_transmit(mpu6050_dev_handle, write_buf, 2, 
                               MPU6050_I2C_TIMEOUT_MS);
}

// I2C读寄存器
static esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t *data)
{
    return i2c_master_transmit_receive(mpu6050_dev_handle, &reg, 1, data, 1,
                                       MPU6050_I2C_TIMEOUT_MS);
}

// I2C连续读取
static esp_err_t mpu6050_read_reg_burst(uint8_t reg, uint8_t len, uint8_t *data)
{
    return i2c_master_transmit_receive(mpu6050_dev_handle, &reg, 1, data, len,
                                       MPU6050_I2C_TIMEOUT_MS);
}

#if MPU6050_USE_FILTER
void pt1_filter_init(pt1_filter_t *filter, float cutoff_hz, float sample_rate_hz)
{
    float rc = 1.0f / (2.0f * M_PI * cutoff_hz);
    float dt = 1.0f / sample_rate_hz;
    filter->alpha = dt / (rc + dt);
    filter->last_output = 0.0f;
    filter->initialized = false;
}

float pt1_filter_apply(pt1_filter_t *filter, float input)
{
    if (!filter->initialized) {
        filter->last_output = input;
        filter->initialized = true;
        return input;
    }
    filter->last_output += filter->alpha * (input - filter->last_output);
    return filter->last_output;
}
#endif

// 软件校准
static int8_t mpu6050_calibrate(uint16_t samples)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint8_t buf[6];
    int16_t gx, gy, gz;

    // 采集零点
    for (uint16_t i = 0; i < samples; i++) {
        if (mpu6050_read_reg_burst(MPU6050_GYRO_XOUT_H, 6, buf) != ESP_OK) {
            return -1;
        }
        gx = (int16_t)((buf[0] << 8) | buf[1]);
        gy = (int16_t)((buf[2] << 8) | buf[3]);
        gz = (int16_t)((buf[4] << 8) | buf[5]);
        
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        mpu6050_delay_ms(5);
    }

    gyro_zero_x = sum_x / samples;
    gyro_zero_y = sum_y / samples;
    gyro_zero_z = sum_z / samples;

    // 验证校准
    sum_x = sum_y = sum_z = 0;
    for (uint16_t i = 0; i < 50; i++) {
        if (mpu6050_read_reg_burst(MPU6050_GYRO_XOUT_H, 6, buf) != ESP_OK) {
            return -1;
        }
        gx = (int16_t)((buf[0] << 8) | buf[1]) - gyro_zero_x;
        gy = (int16_t)((buf[2] << 8) | buf[3]) - gyro_zero_y;
        gz = (int16_t)((buf[4] << 8) | buf[5]) - gyro_zero_z;
        
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        mpu6050_delay_ms(5);
    }

    // 检查校准效果（假设静止时角速度应接近0）
    if (abs(sum_x / 50) > 5 || abs(sum_y / 50) > 5 || abs(sum_z / 50) > 5) {
        return -1;  // 校准失败
    }
    return 0;
}

// 初始化I2C总线
static esp_err_t mpu6050_i2c_bus_init(void)
{
    if (i2c_bus_handle != NULL) {
        return ESP_OK;  // 已初始化
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = MPU6050_I2C_PORT,
        .sda_io_num = MPU6050_I2C_SDA_GPIO,
        .scl_io_num = MPU6050_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,  // 启用内部上拉
    };

    return i2c_new_master_bus(&bus_config, &i2c_bus_handle);
}

// 初始化MPU6050设备
esp_err_t mpu6050_init(const mpu6050_config_t *config)
{
    esp_err_t ret;
    uint8_t who_am_i = 0;

    if (config == NULL || config->sample_rate_hz == 0 || config->sample_rate_hz > 1000) {
        return ESP_ERR_INVALID_ARG;
    }

    if (mpu6050_inited) {
        return ESP_OK;
    }

    // 初始化I2C总线
    ret = mpu6050_i2c_bus_init();
    if (ret != ESP_OK) {
        printf("I2C bus init failed: %d\n", ret);
        return ret;
    }

    // 添加设备到总线
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDRESS,
        .scl_speed_hz = MPU6050_I2C_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &mpu6050_dev_handle);
    if (ret != ESP_OK) {
        printf("I2C add device failed: %d\n", ret);
        return ret;
    }

    // 计算时间间隔
    mpu6050_dt = 1.0f / config->sample_rate_hz;

    // 计算量程比例因子
    switch (config->gyro_range) {
        case gyro_250:  gyro_scale = 250.0f / 32768.0f * (M_PI / 180.0f); break;
        case gyro_500:  gyro_scale = 500.0f / 32768.0f * (M_PI / 180.0f); break;
        case gyro_1000: gyro_scale = 1000.0f / 32768.0f * (M_PI / 180.0f); break;
        case gyro_2000: gyro_scale = 2000.0f / 32768.0f * (M_PI / 180.0f); break;
    }

    switch (config->accel_range) {
        case acc_2g:  accel_scale = 2.0f / 32768.0f; break;
        case acc_4g:  accel_scale = 4.0f / 32768.0f; break;
        case acc_8g:  accel_scale = 8.0f / 32768.0f; break;
        case acc_16g: accel_scale = 16.0f / 32768.0f; break;
    }

    // 复位设备
    ret = mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) goto init_fail;
    mpu6050_delay_ms(100);

    // 唤醒
    ret = mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) goto init_fail;
    mpu6050_delay_ms(10);

    ret = mpu6050_read_reg(MPU6050_WHO_AM_I, &who_am_i);
    if (ret == ESP_OK) {
        if (who_am_i != 0x68 && who_am_i != 0x69) {
            ESP_LOGW(TAG, "Unexpected WHO_AM_I=0x%02X, continue init", who_am_i);
        }
    } else {
        ESP_LOGW(TAG, "Read WHO_AM_I failed (%s), continue init", esp_err_to_name(ret));
    }

    // 设置采样率分频
    uint8_t smplrt_div = (1000 / config->sample_rate_hz) - 1;
    ret = mpu6050_write_reg(MPU6050_SMPLRT_DIV, smplrt_div);
    if (ret != ESP_OK) goto init_fail;

    // 配置低通滤波器
    ret = mpu6050_write_reg(MPU6050_CONFIG, config->filter);
    if (ret != ESP_OK) goto init_fail;

    // 配置陀螺仪量程
    ret = mpu6050_write_reg(MPU6050_GYRO_CONFIG, config->gyro_range);
    if (ret != ESP_OK) goto init_fail;

    // 配置加速度计量程
    ret = mpu6050_write_reg(MPU6050_ACCEL_CONFIG, config->accel_range);
    if (ret != ESP_OK) goto init_fail;

    // 配置中断
    ret = mpu6050_write_reg(MPU6050_INTBP_CFG_REG, 0x00);
    if (ret != ESP_OK) goto init_fail;
    
    if (config->use_interrupt) {
        ret = mpu6050_write_reg(MPU6050_INT_ENABLE, 0x01);
    } else {
        ret = mpu6050_write_reg(MPU6050_INT_ENABLE, 0x00);
    }
    if (ret != ESP_OK) goto init_fail;

    // 配置FIFO
    if (config->use_fifo) {
        ret = mpu6050_write_reg(MPU6050_FIFO_EN, 0x78);  // 启用Accel+Gyro
        ret |= mpu6050_write_reg(MPU6050_USER_CTRL, 0x40); // 启用FIFO
    } else {
        ret = mpu6050_write_reg(MPU6050_FIFO_EN, 0x00);
        ret |= mpu6050_write_reg(MPU6050_USER_CTRL, 0x00);
    }
    if (ret != ESP_OK) goto init_fail;

    // 设置时钟源为X轴陀螺仪
    ret = mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) goto init_fail;

    mpu6050_delay_ms(200);

    // 执行校准
    int8_t cal_result = -1;
    for (int i = 0; i < 3 && cal_result != 0; i++) {
        cal_result = mpu6050_calibrate(64);
    }
    
    if (cal_result != 0) {
        printf("MPU6050 calibration failed!\n");
        // 继续初始化，但使用默认零点
    }

#if MPU6050_USE_FILTER
    // 初始化滤波器
    pt1_filter_init(&pt1_acc_x, 48, config->sample_rate_hz);
    pt1_filter_init(&pt1_acc_y, 48, config->sample_rate_hz);
    pt1_filter_init(&pt1_acc_z, 48, config->sample_rate_hz);
    pt1_filter_init(&pt1_gyro_x, 88, config->sample_rate_hz);
    pt1_filter_init(&pt1_gyro_y, 88, config->sample_rate_hz);
    pt1_filter_init(&pt1_gyro_z, 88, config->sample_rate_hz);
#endif

    printf("MPU6050 init success, sample rate: %d Hz\n", config->sample_rate_hz);
    mpu6050_inited = true;
    return ESP_OK;

init_fail:
    printf("MPU6050 init failed at step, err: %d\n", ret);
    if (mpu6050_dev_handle != NULL) {
        i2c_master_bus_rm_device(mpu6050_dev_handle);
        mpu6050_dev_handle = NULL;
    }
    return ret;
}

// 读取原始数据
static esp_err_t mpu6050_get_raw(MPU6050_t *data)
{
    uint8_t buf[14];
    
    if (mpu6050_read_reg_burst(MPU6050_ACCEL_XOUT_H, 14, buf) != ESP_OK) {
        return ESP_FAIL;
    }

    // 解析数据（大端序）
    data->AccX = (int16_t)((buf[0] << 8) | buf[1]);
    data->AccY = (int16_t)((buf[2] << 8) | buf[3]);
    data->AccZ = (int16_t)((buf[4] << 8) | buf[5]);
    data->rawTemp = (int16_t)((buf[6] << 8) | buf[7]);
    data->GyroX = (int16_t)((buf[8] << 8) | buf[9]);
    data->GyroY = (int16_t)((buf[10] << 8) | buf[11]);
    data->GyroZ = (int16_t)((buf[12] << 8) | buf[13]);

    // 减去零点偏移
    data->GyroX -= gyro_zero_x;
    data->GyroY -= gyro_zero_y;
    data->GyroZ -= gyro_zero_z;

#if MPU6050_USE_FILTER
    data->AccX = (int16_t)pt1_filter_apply(&pt1_acc_x, (float)data->AccX);
    data->AccY = (int16_t)pt1_filter_apply(&pt1_acc_y, (float)data->AccY);
    data->AccZ = (int16_t)pt1_filter_apply(&pt1_acc_z, (float)data->AccZ);
    data->GyroX = (int16_t)pt1_filter_apply(&pt1_gyro_x, (float)data->GyroX);
    data->GyroY = (int16_t)pt1_filter_apply(&pt1_gyro_y, (float)data->GyroY);
    data->GyroZ = (int16_t)pt1_filter_apply(&pt1_gyro_z, (float)data->GyroZ);
#endif
    return ESP_OK;
}

// 基础互补滤波（欧拉角，注意万向节锁问题）
void mpu6050_get_angle(MPU6050_t *data)
{
    static float gyro_roll = 0.0f, gyro_pitch = 0.0f;
    static TickType_t last_tick = 0;
    
    if (mpu6050_get_raw(data) != ESP_OK) {
        return;
    }

    TickType_t now_tick = xTaskGetTickCount();
    float dt = mpu6050_dt;
    if (last_tick != 0) {
        TickType_t diff = now_tick - last_tick;
        if (diff > 0) {
            dt = (float)diff * ((float)portTICK_PERIOD_MS / 1000.0f);
        }
    }
    last_tick = now_tick;

    // 转换为物理单位
    float ax = data->AccX * accel_scale;
    float ay = data->AccY * accel_scale;
    float az = data->AccZ * accel_scale;
    float gx = data->GyroX * gyro_scale;  // 已经是rad/s
    float gy = data->GyroY * gyro_scale;
    float gz = data->GyroZ * gyro_scale;

    // 加速度幅值判断运动状态
    float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
    float gyro_weight = (acc_mag > 1.2f || acc_mag < 0.8f) ? 0.95f : 0.98f;

    // 陀螺仪积分（角度）
    gyro_roll += gy * dt * (180.0f / M_PI);
    gyro_pitch += gx * dt * (180.0f / M_PI);

    // 互补滤波
    float acc_roll = atan2f(ay, az) * (180.0f / M_PI);
    float acc_pitch = -atan2f(ax, az) * (180.0f / M_PI);

    data->roll = gyro_weight * gyro_roll + (1.0f - gyro_weight) * acc_roll;
    data->pitch = gyro_weight * gyro_pitch + (1.0f - gyro_weight) * acc_pitch;
    gyro_roll = data->roll;
    gyro_pitch = data->pitch;
    
    // Yaw纯积分（会漂移）
    static float yaw = 0.0f;
    yaw += gz * dt * (180.0f / M_PI);
    data->yaw = yaw;

    // 应用零点偏移
    data->roll -= angle_roll_offset;
    data->pitch -= angle_pitch_offset;
    data->yaw -= angle_yaw_offset;
}

// 四元数+自适应滤波（推荐，无万向节锁）
void mpu6050_get_angle_plus(MPU6050_t *data)
{
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    static float integral_x = 0.0f, integral_y = 0.0f, integral_z = 0.0f;
    static uint16_t init_cnt = 0;
    static float prev_yaw = 0.0f;
    static float yaw_unwrapped = 0.0f;
    static bool first_yaw = true;
    static TickType_t last_tick = 0;
    static float still_time_s = 0.0f;

    if (mpu6050_get_raw(data) != ESP_OK) {
        return;
    }

    TickType_t now_tick = xTaskGetTickCount();
    float dt = mpu6050_dt;
    if (last_tick != 0) {
        TickType_t diff = now_tick - last_tick;
        if (diff > 0) {
            dt = (float)diff * ((float)portTICK_PERIOD_MS / 1000.0f);
        }
    }
    last_tick = now_tick;

    // 转换为物理单位
    float ax = data->AccX * accel_scale;
    float ay = data->AccY * accel_scale;
    float az = data->AccZ * accel_scale;
    float gx = data->GyroX * gyro_scale;  // rad/s
    float gy = data->GyroY * gyro_scale;
    float gz = data->GyroZ * gyro_scale;
    float gz_dps_raw = gz * (180.0f / M_PI);

    float acc_mag_sq = ax*ax + ay*ay + az*az;

    // 动态增益调整
    float kp, ki;
    if (init_cnt < 400) {
        init_cnt++;
        kp = 8.0f;
        ki = 0.002f;
    } else {
        bool acc_valid = (acc_mag_sq > 0.64f && acc_mag_sq < 1.44f);
        kp = acc_valid ? 4.8f : 3.6f;
        ki = acc_valid ? 0.0015f : 0.001f;
    }

    // 加速度计校正（当有效时）
    if (acc_mag_sq > 0.01f) {
        float recip_norm = inv_sqrt(acc_mag_sq);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        // 计算重力参考方向
        float vx = 2.0f * (q1*q3 - q0*q2);
        float vy = 2.0f * (q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // 误差
        float ex = ay*vz - az*vy;
        float ey = az*vx - ax*vz;
        float ez = ax*vy - ay*vx;

        // PI校正
        if (ki > 0.0f) {
            integral_x += ex * dt;
            integral_y += ey * dt;
            integral_z += ez * dt;
            gx += ki * integral_x;
            gy += ki * integral_y;
        }
        gx += kp * ex;
        gy += kp * ey;
    }

    bool is_still = (acc_mag_sq > 0.90f && acc_mag_sq < 1.10f &&
                     fabsf(gx) < 0.10f && fabsf(gy) < 0.10f && fabsf(gz) < 0.10f);
    if (is_still) {
        still_time_s += dt;
        if (still_time_s > 1.0f) {
            yaw_drift_rate_dps = 0.995f * yaw_drift_rate_dps + 0.005f * gz_dps_raw;
        }
    } else {
        still_time_s = 0.0f;
    }

    gz -= yaw_drift_rate_dps * (M_PI / 180.0f);

    // 四元数微分方程
    float q_dot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float q_dot1 = 0.5f * (q0*gx + q2*gz - q3*gy);
    float q_dot2 = 0.5f * (q0*gy - q1*gz + q3*gx);
    float q_dot3 = 0.5f * (q0*gz + q1*gy - q2*gx);

    q0 += q_dot0 * dt;
    q1 += q_dot1 * dt;
    q2 += q_dot2 * dt;
    q3 += q_dot3 * dt;

    // 归一化
    float norm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    data->q0 = q0;
    data->q1 = q1;
    data->q2 = q2;
    data->q3 = q3;

    // 四元数转欧拉角
    data->roll = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * (180.0f / M_PI);
    float pitch_arg = 2.0f * (q0*q2 - q3*q1);
    if (pitch_arg > 1.0f) pitch_arg = 1.0f;
    if (pitch_arg < -1.0f) pitch_arg = -1.0f;
    data->pitch = asinf(pitch_arg) * (180.0f / M_PI);
    float current_yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * (180.0f / M_PI);

    // Yaw解包裹（连续化）
    if (first_yaw) {
        yaw_unwrapped = current_yaw;
        first_yaw = false;
    } else {
        float diff = current_yaw - prev_yaw;
        if (diff > 180.0f) {
            yaw_unwrapped += diff - 360.0f;
        } else if (diff < -180.0f) {
            yaw_unwrapped += diff + 360.0f;
        } else {
            yaw_unwrapped += diff;
        }
    }
    prev_yaw = current_yaw;

    data->yaw = yaw_unwrapped;

    // 应用偏移
    data->roll -= angle_roll_offset;
    data->pitch -= angle_pitch_offset;
    data->yaw -= angle_yaw_offset;
}

void mpu6050_set_angle_zero(MPU6050_t *data)
{
    // 先读取一次当前角度
    mpu6050_get_angle_plus(data);
    mpu6050_delay_ms(50);
    mpu6050_get_angle_plus(data);
    
    angle_roll_offset = data->roll;
    angle_pitch_offset = data->pitch;
    angle_yaw_offset = data->yaw;
    
    printf("Angle zero set: R=%.2f, P=%.2f, Y=%.2f\n", 
           angle_roll_offset, angle_pitch_offset, angle_yaw_offset);
}

uint8_t mpu6050_read_id(void)
{
    uint8_t id = 0;
    mpu6050_read_reg(MPU6050_WHO_AM_I, &id);
    return id;
}

float mpu6050_get_temp(MPU6050_t *data)
{
    if (mpu6050_get_raw(data) != ESP_OK) {
        return data->temp;
    }
    data->temp = (float)data->rawTemp / 340.0f + 36.53f;
    return data->temp;
}

float mpu6050_get_yaw_drift_rate_dps(void)
{
    return yaw_drift_rate_dps;
}

void mpu6050_reset_yaw_drift_estimator(void)
{
    yaw_drift_rate_dps = 0.0f;
}
