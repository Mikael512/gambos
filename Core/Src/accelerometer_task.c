#include <stdio.h>
#include "imu_sensor.h"
#include "accelerometer_task.h"
#include "i2c_task.h"
#include "semphr.h"

static uint8_t acc_rx_buf[6] = {0}; // Buffer to hold accelerometer data

static i2c_request_t acc_read_req = {
    .op = I2C_OP_MEM_READ,
    .dev_addr = ACC_ADDRESS,
    .reg_addr = OUT_X_L_A | 0x80,
    .rx_buf = acc_rx_buf,
    .rx_len = sizeof(acc_rx_buf),
    .tx_buf = NULL,
    .tx_len = 0,
    .done_sem = NULL,
};

static i2c_request_t* req_to_send; 

void AccelerometerTask(void *pvParameters) {
    ImuSensor_t* imu = (ImuSensor_t*) pvParameters;
    TickType_t xLastWakeTime;

    printf("Accelerometer task started\r\n");

    xLastWakeTime = xTaskGetTickCount();

    acc_read_req.done_sem = xSemaphoreCreateBinary();
    xSemaphoreTake(acc_read_req.done_sem, 0); // Initialize as "not available"

    while (1) {
        req_to_send = &acc_read_req;
        // Send the request address to the I2C task
        if (xQueueSend(i2c_queue, &req_to_send, 0) == pdPASS) {
            // Block here until the I2C task signals completion
            if (xSemaphoreTake(acc_read_req.done_sem, portMAX_DELAY) == pdTRUE) {
                // Successfully read data — parse it
                parse_acc_data(acc_rx_buf, imu);
                printf("Accelerometer data: X = %7d, Y = %7d, Z = %7d\r\n", imu->acc[0], imu->acc[1], imu->acc[2]);
            } else {
                // Timeout or failure — handle it
                printf("Failed to read Accelerometer data\r\n");
            }
        }
        xSemaphoreGive(i2cSemaphore);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(3000));
    }
}

void parse_acc_data(uint8_t *rx_buf, ImuSensor_t *dev) {
	dev->acc[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
	dev->acc[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
	dev->acc[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
}