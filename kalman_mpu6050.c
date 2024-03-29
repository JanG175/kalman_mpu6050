#include <stdio.h>
#include "kalman_mpu6050.h"

static const char* TAG = "kalman_mpu6050";
static mpu6050_handle_t mpu;

static SemaphoreHandle_t mutex = NULL;
euler_angle_t euler;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static float g_yaw = 0.0f;


/**
 * @brief calculate euler angle from accelerometer data
 * 
 * @param acce_data accelerometer data
 * @param roll roll angle
 * @param pitch pitch angle
 * @param yaw yaw angle
*/
static void calculate_euler_angle_from_accel(mpu6050_acce_value_t* acce_data, complimentary_angle_t* complimentary_angle)
{
    complimentary_angle->roll = atan2f(acce_data->acce_y, acce_data->acce_z) * 180.0f / M_PI;

    complimentary_angle->pitch = atan2f(-acce_data->acce_x, acce_data->acce_z) * 180.0f / M_PI;
}


/**
 * @brief task read euler angle from MPU6050 sensor
 * 
 * @param euler euler angle
*/
static void kalman_euler_angle_read(void* pvParameters)
{
    portENTER_CRITICAL(&spinlock);
    float yaw_drift = g_yaw;
    portEXIT_CRITICAL(&spinlock);

    double dt = (double)DT / 1000.0;
    double std_dev_v = STD_DEV_V;
    double std_dev_w = STD_DEV_W;

    // init measurement
    mpu6050_acce_value_t acce_data;
    mpu6050_gyro_value_t gyro_data;
    mpu6050_temp_value_t temp_data;

    mpu_get_data(&acce_data, &gyro_data, &temp_data);

    complimentary_angle_t complimentary_angle;
    calculate_euler_angle_from_accel(&acce_data, &complimentary_angle);

    // zero roll and pitch with accelerometer
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
        euler.gyro_roll = complimentary_angle.roll;
        euler.gyro_pitch = complimentary_angle.pitch;

        xSemaphoreGive(mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take mutex");
    }

    /*

    Xpri' = A * Xpri + B * U
      Y   = C * Xpri + D * U

    --    --   --     --   --   --   --  --            
    | phi' |   | 1 -dt |   | phi |   | dt |   --     --
    |      | = |       | * |     | + |    | * | Wgyro |
    |  g'  |   | 0   1 |   |  g  |   |  0 |   --     --
    --    --   --     --   --   --   --  --            

                  --   --
        --   --   | phi |
    Y = | 1 0 | * |     |
        --   --   |  g  |
                  --   --

    */

    // state space model
    matrix_t A;
    matrix_alloc(&A, 2, 2);
    A.array[0][0] = 1.0;
    A.array[0][1] = -dt;
    A.array[1][0] = 0.0;
    A.array[1][1] = 1.0;

    matrix_t B;
    matrix_alloc(&B, 2, 1);
    B.array[0][0] = dt;
    B.array[1][0] = 0.0;

    matrix_t C;
    matrix_alloc(&C, 1, 2);
    C.array[0][0] = 1.0;
    C.array[0][1] = 0.0;

    // noise
    matrix_t V;
    matrix_alloc(&V, 2, 2);
    V.array[0][0] = pow(std_dev_v, 2.0);
    V.array[0][1] = 0.0;
    V.array[1][0] = 0.0;
    V.array[1][1] = pow(std_dev_v, 2.0);

    matrix_t W;
    matrix_alloc(&W, 1, 1);
    W.array[0][0] = pow(std_dev_w, 2.0);

    // initial states
    matrix_t Xpri;
    matrix_alloc(&Xpri, 2, 2);
    Xpri.array[0][0] = 0.0;
    Xpri.array[0][1] = 0.0;
    Xpri.array[1][0] = 0.0;
    Xpri.array[1][1] = 0.0;

    matrix_t Ppri;
    matrix_alloc(&Ppri, 2, 2);
    Ppri.array[0][0] = 1.0;
    Ppri.array[0][1] = 0.0;
    Ppri.array[1][0] = 0.0;
    Ppri.array[1][1] = 1.0;

    matrix_t Xpost;
    matrix_alloc(&Xpost, 2, 2);
    Xpost.array[0][0] = complimentary_angle.roll;
    Xpost.array[0][1] = complimentary_angle.pitch;
    Xpost.array[1][0] = 0.0;
    Xpost.array[1][1] = 0.0;

    matrix_t Ppost;
    matrix_alloc(&Ppost, 2, 2);
    Ppost.array[0][0] = 1.0;
    Ppost.array[0][1] = 0.0;
    Ppost.array[1][0] = 0.0;
    Ppost.array[1][1] = 1.0;

    matrix_t U;
    matrix_alloc(&U, 1, 2);
    U.array[0][0] = 0.0;
    U.array[0][1] = 0.0;

    matrix_t Y;
    matrix_alloc(&Y, 1, 2);
    U.array[0][0] = 0.0;
    U.array[0][1] = 0.0;

    matrix_t E;
    matrix_alloc(&E, 1, 2);

    matrix_t S;
    matrix_alloc(&S, 1, 1);

    matrix_t K;
    matrix_alloc(&K, 2, 1);

    // auxilary matrices
    matrix_t AXpost;
    matrix_alloc(&AXpost, A.rows, Xpost.cols);

    matrix_t BU;
    matrix_alloc(&BU, B.rows, U.cols);

    matrix_t APpost;
    matrix_alloc(&APpost, A.rows, Ppost.cols);

    matrix_t At;
    matrix_alloc(&At, A.cols, A.rows);
    matrix_trans(&A, &At);

    matrix_t APpostAt;
    matrix_alloc(&APpostAt, APpost.rows, At.cols);

    matrix_t CXpri;
    matrix_alloc(&CXpri, C.rows, Xpri.cols);

    matrix_t CPpri;
    matrix_alloc(&CPpri, C.rows, Ppri.cols);

    matrix_t Ct;
    matrix_alloc(&Ct, C.cols, C.rows);
    matrix_trans(&C, &Ct);

    matrix_t CPpriCt;
    matrix_alloc(&CPpriCt, CPpri.rows, Ct.cols);

    matrix_t Sinv;
    matrix_alloc(&Sinv, S.rows, S.cols);

    matrix_t PpriCt;
    matrix_alloc(&PpriCt, Ppri.rows, Ct.cols);

    matrix_t KE;
    matrix_alloc(&KE, K.rows, E.cols);

    matrix_t KS;
    matrix_alloc(&KS, K.rows, S.cols);

    matrix_t Kt;
    matrix_alloc(&Kt, K.cols, K.rows);

    matrix_t KSKt;
    matrix_alloc(&KSKt, KS.rows, Kt.cols);

    // Kalman filter
    while (1)
    {
        // new measurement
        mpu_get_data(&acce_data, &gyro_data, &temp_data);

        calculate_euler_angle_from_accel(&acce_data, &complimentary_angle);

        U.array[0][0] = gyro_data.gyro_x;
        U.array[0][1] = gyro_data.gyro_y;

        Y.array[0][0] = complimentary_angle.roll;
        Y.array[0][1] = complimentary_angle.pitch;

        // Xpri
        matrix_mul(&A, &Xpost, &AXpost);
        matrix_mul(&B, &U, &BU);
        matrix_add(&AXpost, &BU, &Xpri);

        // Ppri
        matrix_mul(&A, &Ppost, &APpost);
        matrix_mul(&APpost, &At, &APpostAt);
        matrix_add(&APpostAt, &V, &Ppri);

        // E
        matrix_mul(&C, &Xpri, &CXpri);
        matrix_sub(&Y, &CXpri, &E);

        // S
        matrix_mul(&C, &Ppri, &CPpri);
        matrix_mul(&CPpri, &Ct, &CPpriCt);
        matrix_add(&CPpriCt, &W, &S);

        // K
        matrix_inv(&S, &Sinv);
        matrix_mul(&Ppri, &Ct, &PpriCt);
        matrix_mul(&PpriCt, &Sinv, &K);

        // Xpost
        matrix_mul(&K, &E, &KE);
        matrix_add(&Xpri, &KE, &Xpost);

        // Ppost
        matrix_mul(&K, &S, &KS);
        matrix_trans(&K, &Kt);
        matrix_mul(&KS, &Kt, &KSKt);
        matrix_sub(&Ppri, &KSKt, &Ppost);

        // calculate euler angle from gyro
        if (xSemaphoreTake(mutex, MUTEX_MAX_WAIT) == pdTRUE)
        {
            euler.acce_roll = Xpost.array[0][0];
            euler.acce_pitch = Xpost.array[0][1];

            euler.gyro_roll += (gyro_data.gyro_x - Xpost.array[1][0]) * dt;
            euler.gyro_pitch += (gyro_data.gyro_y - Xpost.array[1][1]) * dt;
            euler.gyro_yaw += (gyro_data.gyro_z - (Xpost.array[1][0] + Xpost.array[1][1] + yaw_drift) / 3.0) * dt;

            xSemaphoreGive(mutex);

            vTaskDelay(DT / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGW(TAG, "Failed to take mutex");
        }
    }

    // deallocate memory
    matrix_dealloc(&A);
    matrix_dealloc(&B);
    matrix_dealloc(&C);
    matrix_dealloc(&V);
    matrix_dealloc(&W);
    matrix_dealloc(&E);
    matrix_dealloc(&S);
    matrix_dealloc(&K);
    matrix_dealloc(&U);
    matrix_dealloc(&Y);
    matrix_dealloc(&Xpri);
    matrix_dealloc(&Ppri);
    matrix_dealloc(&Xpost);
    matrix_dealloc(&Ppost);
    matrix_dealloc(&AXpost);
    matrix_dealloc(&BU);
    matrix_dealloc(&APpost);
    matrix_dealloc(&At);
    matrix_dealloc(&APpostAt);
    matrix_dealloc(&CXpri);
    matrix_dealloc(&CPpri);
    matrix_dealloc(&Ct);
    matrix_dealloc(&CPpriCt);
    matrix_dealloc(&Sinv);
    matrix_dealloc(&PpriCt);
    matrix_dealloc(&KE);
    matrix_dealloc(&KS);
    matrix_dealloc(&Kt);
    matrix_dealloc(&KSKt);

    vTaskDelete(NULL);
}


/**
 * @brief initialize MPU6050 sensor
 * 
 * @param mpu_conf MPU6050 I2C configuration
*/
void mpu_init(mpu_i2c_conf_t mpu_conf)
{
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = mpu_conf.sda_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = mpu_conf.scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = mpu_conf.i2c_freq;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    ESP_ERROR_CHECK(i2c_param_config(mpu_conf.i2c_num, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(mpu_conf.i2c_num, conf.mode, 0, 0, 0));

    mpu = mpu6050_create(mpu_conf.i2c_num, mpu_conf.i2c_addr);
    if (mpu == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mpu6050");
        return;
    }

    ESP_ERROR_CHECK(mpu6050_wake_up(mpu));

    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
        euler.acce_roll = 0.0f;
        euler.acce_pitch = 0.0f;
        euler.gyro_roll = 0.0f;
        euler.gyro_pitch = 0.0f;
        euler.gyro_yaw = 0.0f;

        xSemaphoreGive(mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take mutex");
    }

    mpu_find_yaw_drift();

    xTaskCreate(kalman_euler_angle_read, "kalman euler angle read", 4096, NULL, 3, NULL);
}


/**
 * @brief get data from MPU6050 sensor
 * 
 * @param acce acce data
 * @param gyro gyro data
 * @param temp temp data
*/
void mpu_get_data(mpu6050_acce_value_t* acce, mpu6050_gyro_value_t* gyro, mpu6050_temp_value_t* temp)
{
    ESP_ERROR_CHECK(mpu6050_get_acce(mpu, acce));
    ESP_ERROR_CHECK(mpu6050_get_gyro(mpu, gyro));
    ESP_ERROR_CHECK(mpu6050_get_temp(mpu, temp));
}


/**
 * @brief get euler angle
 * 
 * @param euler_angle euler angle
*/
void mpu_get_euler_angle(euler_angle_t* euler_angle)
{
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
        euler_angle->acce_roll = euler.acce_roll;
        euler_angle->acce_pitch = euler.acce_pitch;

        euler_angle->gyro_roll = euler.gyro_roll;
        euler_angle->gyro_pitch = euler.gyro_pitch;
        euler_angle->gyro_yaw = euler.gyro_yaw;

        xSemaphoreGive(mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take mutex");
    }
}


/**
 * @brief get complimentary angle
 * 
 * @param complimentary_angle complimentary angle (roll, pitch)
*/
void mpu_get_roll_pitch(complimentary_angle_t* complimentary_angle)
{
    mpu6050_acce_value_t acce_data;
    mpu6050_gyro_value_t gyro_data;
    mpu6050_temp_value_t temp_data;

    mpu_get_data(&acce_data, &gyro_data, &temp_data);

    mpu6050_complimentory_filter(mpu, &acce_data, &gyro_data, complimentary_angle);
}


/**
 * @brief find yaw drift
 * 
*/
void mpu_find_yaw_drift()
{
    mpu6050_gyro_value_t gyro_data;
    float last_gyro_z = 0.0f;
    float yaw_drift = 0.0f;

    ESP_LOGW(TAG, "Finding yaw drift - keep the sensor still...");
    for (uint32_t i = 0; i <= 100; i++)
    {
        mpu6050_get_gyro(mpu, &gyro_data);

        if (i == 0)
        {
            last_gyro_z = gyro_data.gyro_z;
        }
        else
        {
            yaw_drift += (gyro_data.gyro_z - last_gyro_z);
            last_gyro_z = gyro_data.gyro_z;
        }

        vTaskDelay(DT / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Yaw drift found!");

    yaw_drift /= 100.0f;

    portENTER_CRITICAL(&spinlock);
    g_yaw = yaw_drift;
    portEXIT_CRITICAL(&spinlock);
}