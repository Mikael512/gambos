#include "ism330dhcx.h"
#include "i2c_task.h"
#include <stdio.h>


void SensorInitTask(void *pvParameters) {
    printf("Sensor init task started\r\n");

    uint8_t acc_ctrl1 = 0x40;
    i2c_mem_write(ISM330DHCX, CTRL1_XL, &acc_ctrl1, 1, pdMS_TO_TICKS(100));

    uint8_t gyro_ctrl1 = 0x40;
    i2c_mem_write(ISM330DHCX, CTRL2_G, &gyro_ctrl1, 1, pdMS_TO_TICKS(100));



    // Initialization complete
    printf("All sensors initialized. Deleting init task.\r\n");

    // Delete the current task
    vTaskDelete(NULL);
}