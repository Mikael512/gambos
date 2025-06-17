#include "iis2mdc.h"
#include "i2c_task.h"
#include <stdio.h>


void MagnetometerTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("Magnetometer task started\r\n");

    xLastWakeTime = xTaskGetTickCount();

    uint8_t mag_rx_buf[6] = {0};
    int16_t mag_data[3] = {0};

    while (1) {
        if(i2c_mem_read(IIS2MDC, OUTX_L_REG | 0x80, mag_rx_buf, sizeof(mag_rx_buf), pdMS_TO_TICKS(10)) == HAL_OK) {
            parse_mag_data(mag_rx_buf, mag_data);
            printf("Magnetometer data: X = %7d, Y = %7d, Z = %7d\r\n", mag_data[0], mag_data[1], mag_data[2]);

        } else {
            printf("Failed to read Magnetometer data\r\n");
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

void parse_mag_data(uint8_t *rx_buf, int16_t *data) {
    data[0] = (int16_t)((((int16_t)rx_buf[0]) << 8) | rx_buf[1]);
    data[1] = (int16_t)((((int16_t)rx_buf[2]) << 8) | rx_buf[3]);
    data[2] = (int16_t)((((int16_t)rx_buf[4]) << 8) | rx_buf[5]);
}
