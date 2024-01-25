#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_matrix.h"
#include "esp_log.h"

#define DT 10 // integration step in ms
#define MUTEX_MAX_WAIT (DT / portTICK_PERIOD_MS)

#define STD_DEV_V 0.01 // process noise
#define STD_DEV_W 0.0001 // sensor noise

typedef struct {
    i2c_port_t i2c_num;
    uint8_t i2c_addr;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t i2c_freq;
} mpu_i2c_conf_t;

typedef struct {
    float acce_roll;
    float acce_pitch;
    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;
} euler_angle_t;


void mpu_init(mpu_i2c_conf_t mpu_conf);

void mpu_get_data(mpu6050_acce_value_t* acce, mpu6050_gyro_value_t* gyro, mpu6050_temp_value_t* temp);

void mpu_get_euler_angle(euler_angle_t* euler_angle);

void mpu_get_roll_pitch(complimentary_angle_t* complimentary_angle);
