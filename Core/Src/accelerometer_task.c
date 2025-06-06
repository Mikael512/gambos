#include <stdio.h>
#include "imu_sensor.h"
#include "accelerometer_task.h"
#include "i2c_task.h"
#include "semphr.h"


void AccelerometerTask(void *pvParameters) {
    ImuSensor_t* imu = (ImuSensor_t*) pvParameters;
    TickType_t xLastWakeTime;

    printf("Accelerometer task started\r\n");

    xLastWakeTime = xTaskGetTickCount();

    uint8_t acc_rx_buf[6] = {0};
    int16_t acc_data[3] = {0};

    while (1) {
        if(i2c_mem_read(ACC_ADDRESS, OUT_X_L_A | 0x80, acc_rx_buf, sizeof(acc_rx_buf), pdMS_TO_TICKS(100)) == HAL_OK) {
            parse_acc_data(acc_rx_buf, acc_data);
            printf("Accelerometer data: X = %7d, Y = %7d, Z = %7d\r\n", acc_data[0], acc_data[1], acc_data[2]);
        } else {
            printf("Failed to read Accelerometer data\r\n");
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

void parse_acc_data(uint8_t *rx_buf, int16_t *data)  {
	data[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
	data[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
	data[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
}