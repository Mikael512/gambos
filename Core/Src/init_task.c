#include "ism330dhcx.h"
#include "iis2mdc.h"
#include "i2c_task.h"
#include <stdio.h>


void InitTask(void *pvParameters) {
    printf("Sensor initialization task started\r\n");
    vTaskDelay(pdMS_TO_TICKS(100));

    // Accelerometer to 104 Hz ODR and +/-2g scale
    uint8_t acc_ctrl1 = 0x40;
    i2c_mem_write(ISM330DHCX, CTRL1_XL, &acc_ctrl1, 1, pdMS_TO_TICKS(100));

    // Gyroscope to 104 Hz ODR and +/-250 dps scale
    uint8_t gyro_ctrl1 = 0x40;
    i2c_mem_write(ISM330DHCX, CTRL2_G, &gyro_ctrl1, 1, pdMS_TO_TICKS(100));

    // Magnetometer to 50 Hz ODR, temperature compensation, continuous mode
    uint8_t mag_cfga = 0x88;
    i2c_mem_write(IIS2MDC, CFG_REG_A, &mag_cfga, 1, pdMS_TO_TICKS(100));

    // Initialization complete
    printf("All sensors initialized. Deleting initialization task.\r\n");

    // Delete the current task
    vTaskDelete(NULL);
}