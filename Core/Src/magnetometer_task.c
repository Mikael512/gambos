#include "magnetometer_task.h"


void MagnetometerTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    vTaskDelay(1000);

    printf("Magnetometer task started\r\n");

    xLastWakeTime = xTaskGetTickCount();

    uint8_t mag_rx_buf[6] = {0};
    int16_t mag_data[3] = {0};

    while (1) {
        if(i2c_mem_read(MAG_ADDRESS, OUT_X_H_M, mag_rx_buf, sizeof(mag_rx_buf), pdMS_TO_TICKS(100)) == HAL_OK) {
            parse_mag_data(mag_rx_buf, mag_data);
            printf("Data x: %7d, y: %7d, z: %7d\r\n", mag_data[0], mag_data[1], mag_data[2]);
        } else {
            printf("Failed to read Magnetometer data\r\n");
        }       

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

void parse_mag_data(uint8_t *rx_buf, int16_t *data) {
    data[0] = (int16_t)((((int16_t)rx_buf[0]) << 8) | rx_buf[1]);
    data[1] = (int16_t)((((int16_t)rx_buf[4]) << 8) | rx_buf[5]);
    data[2] = (int16_t)((((int16_t)rx_buf[2]) << 8) | rx_buf[3]);
}
