idf_component_register(SRCS "kalman_mpu6050.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver espressif__mpu6050 esp_matrix)
