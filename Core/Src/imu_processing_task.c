#include <stdio.h>
#include "imu_processing_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bager_buffer.h"
#include "math.h"

#define CONSUMER_ID 0

#define LSB_DPS 131.07f


void ImuProcessingTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("Imu processing task started\r\n");

    xLastWakeTime = xTaskGetTickCount();

    int16_3d_t acc_data = {0};
    int16_3d_t gyro_data = {0};
    int16_3d_t mag_data = {0};

    // float roll, pitch, yaw = 0.0;

    while (1) {
        pop_data(BUFFER_ACC, CONSUMER_ID, &acc_data);
        pop_data(BUFFER_GYRO, CONSUMER_ID, &gyro_data);
        pop_data(BUFFER_MAG, CONSUMER_ID, &mag_data);
   
        // roll = atan2f((float)acc_data.y, sqrt((float)acc_data.x * (float)acc_data.x + (float)acc_data.z * (float)acc_data.z));
        // pitch = atan2((float)acc_data.x, sqrt((float)acc_data.y * (float)acc_data.y + (float)acc_data.z * (float)acc_data.z));
        // yaw += (gyro_data.z+45)*0.1/LSB_DPS;

        // printf("Roll: %f, Pitch: %f, Yaw: %f, Raw yaw: %d\r\n", roll, pitch, yaw, gyro_data.z+45);
        printf("acc data: X = %d, Y = %d, Z = %d\r\n", acc_data.x, acc_data.y, acc_data.z);
        printf("gyro data: X = %d, Y = %d, Z = %d\r\n", gyro_data.x, gyro_data.y, gyro_data.z);
        printf("mag data: X = %d, Y = %d, Z = %d\r\n", mag_data.x, mag_data.y, mag_data.z);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

