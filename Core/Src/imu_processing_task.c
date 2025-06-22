#include <stdio.h>
#include "imu_processing_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bager_buffer.h"
#include "math.h"

#define CONSUMER_ID 0


void ImuProcessingTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("Imu processing task started\r\n");

    xLastWakeTime = xTaskGetTickCount();

    int16_3d_t acc_data = {0};
    int16_3d_t gyro_data = {0};
    int16_3d_t mag_data = {0};

    float roll, pitch;

    while (1) {
        pop_data(BUFFER_ACC, CONSUMER_ID, &acc_data);
        // pop_data(BUFFER_GYRO, CONSUMER_ID, &gyro_data);
        // pop_data(BUFFER_MAG, CONSUMER_ID, &mag_data);
   
        roll = atan2f((float)acc_data.y, sqrt((float)acc_data.x * (float)acc_data.x + (float)acc_data.z * (float)acc_data.z));
        pitch = atan2((float)acc_data.x, sqrt((float)acc_data.y * (float)acc_data.y + (float)acc_data.z * (float)acc_data.z));

        printf("Roll: %f, Pitch: %f\r\n", roll, pitch);


        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

