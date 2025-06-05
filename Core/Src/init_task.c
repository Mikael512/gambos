#include "ism330dhcx.h"
#include "i2c_task.h"
#include <stdio.h>


void InitTask(void *pvParameters) {
    printf("Sensor init task started\r\n");

    // Accelerometer to 104 Hz ODR and +/-2g scale
    uint8_t acc_ctrl1 = 0x40;
    i2c_mem_write(ISM330DHCX, CTRL1_XL, &acc_ctrl1, 1, pdMS_TO_TICKS(100));

    // Gyroscope to 104 Hz ODR and +/-250 dps scale
    uint8_t gyro_ctrl1 = 0x40;
    i2c_mem_write(ISM330DHCX, CTRL2_G, &gyro_ctrl1, 1, pdMS_TO_TICKS(100));

    // Initialization complete
    printf("All sensors initialized. Deleting init task.\r\n");

    // Delete the current task
    vTaskDelete(NULL);
}