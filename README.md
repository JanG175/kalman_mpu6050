# MPU6050 with Kalman filter ESP-IDF component

# Notes
* Adjast Kalman filter with macros: `STD_DEV_V` and `STD_DEV_W` in `kalman_mpu6050.h`.

## Sources
* https://github.com/espressif/esp-bsp/tree/8a042287f67573e757ff25f8bb6372c3fffb5313/components/mpu6050
* https://github.com/JanG175/esp_matrix

## How 2 use?
```C
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kalman_mpu6050.h"
#include "esp_log.h"

#define I2C_NUM        I2C_NUM_1
#define I2C_ADDR       MPU6050_I2C_ADDRESS
#define I2C_SDA_PIN    14
#define I2C_SCL_PIN    19
#define I2C_FREQ       400000


void app_main(void)
{
    mpu_i2c_conf_t mpu_conf = {
        .i2c_num = I2C_NUM,
        .i2c_addr = I2C_ADDR,
        .sda_pin = I2C_SDA_PIN,
        .scl_pin = I2C_SCL_PIN,
        .i2c_freq = I2C_FREQ
    };
    mpu_init(mpu_conf);

    while (1)
    {
        euler_angle_t euler_angle;
        mpu_get_euler_angle(&euler_angle);

        printf(">kalman_acce_roll:");
        printf("%f\n", euler_angle.acce_roll);
        printf(">kalman_acce_pitch:");
        printf("%f\n", euler_angle.acce_pitch);
        printf(">kalman_gyro_roll:");
        printf("%f\n", euler_angle.gyro_roll);
        printf(">kalman_gyro_pitch:");
        printf("%f\n", euler_angle.gyro_pitch);
        printf(">kalman_gyro_yaw:");
        printf("%f\n", euler_angle.gyro_yaw);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
```