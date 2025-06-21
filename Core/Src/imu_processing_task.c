#include <stdio.h>
#include "imu_processing_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bager_buffer.h"

#define CONSUMER_ID 0


void ImuProcessingTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("Imu processing task started\r\n");

    xLastWakeTime = xTaskGetTickCount();

    int16_3d_t acc_data = {0};
    int16_3d_t gyro_data = {0};
    int16_3d_t mag_data = {0};

    while (1) {
        pop_data(BUFFER_ACC, CONSUMER_ID, &acc_data);
        pop_data(BUFFER_GYRO, CONSUMER_ID, &gyro_data);
        pop_data(BUFFER_MAG, CONSUMER_ID, &mag_data);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

