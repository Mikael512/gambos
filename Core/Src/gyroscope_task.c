#include <stdio.h>
#include "ism330dhcx.h"
#include "i2c_task.h"


void GyroscopeTask(void *pvParameters) {
    TickType_t xLastWakeTime;

    printf("Gyroscope task started\r\n");

    xLastWakeTime = xTaskGetTickCount();

    uint8_t gyro_rx_buf[6] = {0};
    int16_t gyro_data[3] = {0};

    while (1) {
        if(i2c_mem_read(ISM330DHCX, OUTX_L_G, gyro_rx_buf, sizeof(gyro_rx_buf), pdMS_TO_TICKS(10)) == HAL_OK) {
            parse_gyro_data(gyro_rx_buf, gyro_data);
            printf("Gyroscope data: X = %7d, Y = %7d, Z = %7d\r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
        } else {
            printf("Failed to read Gyroscope data\r\n");
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

void parse_gyro_data(uint8_t *rx_buf, int16_t *data)  {
	data[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
	data[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
	data[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
}