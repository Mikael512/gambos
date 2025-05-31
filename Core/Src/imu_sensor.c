#include "imu_sensor.h"
#include "i2c_task.h"
#include <stdio.h>


void SensorInitTask(void *pvParameters) {
    printf("Sensor init task started\r\n");
    ImuSensor_t* imu = (ImuSensor_t*) pvParameters;

    for (int i = 0; i < 3; ++i) {
        imu->acc[i] = 0;
        imu->mag[i] = 0;
        imu->gyro[i] = 0;
    }

    uint8_t acc_ctrl = 0x67;
    i2c_mem_write(ACC_ADDRESS, CTRL_REG1_A, &acc_ctrl, 1, pdMS_TO_TICKS(100));

    uint8_t mag_cra = 0x10;
    i2c_mem_write(MAG_ADDRESS, CRA_REG_M, &mag_cra, 1, pdMS_TO_TICKS(100));

    uint8_t mag_crb = 0x80;
    i2c_mem_write(MAG_ADDRESS, CRB_REG_M, &mag_crb, 1, pdMS_TO_TICKS(1000));

    uint8_t mag_mr = 0x00;
    i2c_mem_write(MAG_ADDRESS, MR_REG_M, &mag_mr, 1, pdMS_TO_TICKS(1000));

    uint8_t gyr_ctrl1 = 0x0F;
    i2c_mem_write(GYRO_ADDRESS, CTRL_REG1, &gyr_ctrl1, 1, pdMS_TO_TICKS(100));

    uint8_t gyr_ctrl2 = 0x29;
    i2c_mem_write(GYRO_ADDRESS, CTRL_REG2, &gyr_ctrl2, 1, pdMS_TO_TICKS(100));

    // Initialization complete
    printf("All sensors initialized. Deleting init task.\r\n");

    // Delete the current task
    vTaskDelete(NULL);
}