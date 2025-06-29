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

    // [Hard Iron Bias] X: 0.0758, Y: -0.0547, Z: 0.0967
    // Hard iron bias for X: 0.0758 --> raw value: 0.0758 / 0.0015 = 50 = 0x0032
    uint8_t mag_hard_offset_x[2] = {0x32, 0x00};
    uint8_t zero[2] = {0x00, 0x00};
    i2c_mem_write(IIS2MDC, OFFSET_X_REG_L | 0x80, zero, 2, pdMS_TO_TICKS(100));

    // Hard iron bias for Y: -0.0547 --> raw value: -0.0547 / 0.0015 = -36 = 0xFFDC
    uint8_t mag_hard_offset_y[2] = {0xDC, 0xFF};
    i2c_mem_write(IIS2MDC, OFFSET_Y_REG_L | 0x80, zero, 2, pdMS_TO_TICKS(100));

    // Hard iron bias for Z: 0.0967 --> raw value: 0.0967 / 0.0015 = 64 = 0x0040
    uint8_t mag_hard_offset_z[2] = {0x40, 0x00};
    i2c_mem_write(IIS2MDC, OFFSET_Z_REG_L | 0x80, zero, 2, pdMS_TO_TICKS(100));

    // Magnetometer to 50 Hz ODR, temperature compensation, continuous mode
    uint8_t mag_cfga = 0x88;
    i2c_mem_write(IIS2MDC, CFG_REG_A, &mag_cfga, 1, pdMS_TO_TICKS(100));

    // Initialization complete
    printf("All sensors initialized. Deleting initialization task.\r\n");

    // Delete the current task
    vTaskDelete(NULL);
}