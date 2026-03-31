#ifndef _MPU6050_h_
#define _MPU6050_h_

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"

// 配置：是否使用内部滤波器
#ifndef MPU6050_USE_FILTER
#define MPU6050_USE_FILTER 1
#endif

// I2C配置
#define MPU6050_I2C_SDA_GPIO        8       // 根据你的硬件修改
#define MPU6050_I2C_SCL_GPIO        9       // 根据你的硬件修改
#define MPU6050_I2C_PORT            I2C_NUM_0
#define MPU6050_I2C_FREQ_HZ         400000  // 400kHz快速模式
#define MPU6050_I2C_TIMEOUT_MS      100

// MPU6050地址
#define MPU6050_ADDRESS             0x68    // AD0接地时的地址，接VCC为0x69

// 寄存器定义（与原版相同）
#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_CONFIG              0x1A
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_FIFO_EN             0x23
#define MPU6050_INTBP_CFG_REG       0x37
#define MPU6050_INT_ENABLE          0x38
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_SIGNAL_PATH_RESET   0x68
#define MPU6050_USER_CTRL           0x6A
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_WHO_AM_I            0x75

// 数据结构
typedef struct {
    // 原始数据
    int16_t AccX, AccY, AccZ;
    int16_t GyroX, GyroY, GyroZ;
    int16_t rawTemp;
    // 欧拉角
    float yaw, roll, pitch;
    float temp;
    // 四元数
    float q0, q1, q2, q3;
} MPU6050_t;

// 滤波器带宽
typedef enum {
    Band_256Hz = 0x00,
    Band_186Hz = 0x01,
    Band_96Hz  = 0x02,
    Band_43Hz  = 0x03,
    Band_21Hz  = 0x04,
    Band_10Hz  = 0x05,
    Band_5Hz   = 0x06
} mpu6050_filter_t;

// 陀螺仪量程
typedef enum {
    gyro_250  = 0x00,
    gyro_500  = 0x08,
    gyro_1000 = 0x10,
    gyro_2000 = 0x18
} mpu6050_gyro_range_t;

// 加速度计量程
typedef enum {
    acc_2g  = 0x00,
    acc_4g  = 0x08,
    acc_8g  = 0x10,
    acc_16g = 0x18
} mpu6050_accel_range_t;

// 初始化配置结构体
typedef struct {
    uint16_t sample_rate_hz;            // 采样率Hz
    mpu6050_filter_t filter;            // 低通滤波带宽
    mpu6050_gyro_range_t gyro_range;    // 陀螺仪量程
    mpu6050_accel_range_t accel_range;  // 加速度计量程
    bool use_fifo;                      // 是否使用FIFO
    bool use_interrupt;                 // 是否使用中断
} mpu6050_config_t;

// 函数声明
esp_err_t mpu6050_init(const mpu6050_config_t *config);
void mpu6050_get_angle(MPU6050_t *data);        // 基础互补滤波
void mpu6050_get_angle_plus(MPU6050_t *data);   // 四元数+自适应滤波
void mpu6050_set_angle_zero(MPU6050_t *data);
uint8_t mpu6050_read_id(void);
float mpu6050_get_temp(MPU6050_t *data);
float mpu6050_get_yaw_drift_rate_dps(void);
void mpu6050_reset_yaw_drift_estimator(void);

// 内部使用的滤波器（简化版PT1）
#if MPU6050_USE_FILTER
typedef struct {
    float alpha;        // 滤波系数
    float last_output;
    bool initialized;
} pt1_filter_t;

void pt1_filter_init(pt1_filter_t *filter, float cutoff_hz, float sample_rate_hz);
float pt1_filter_apply(pt1_filter_t *filter, float input);
#endif

#endif // MPU6050
