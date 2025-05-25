#include "magnetometer_task.h"

static uint8_t mag_rx_buf[6] = {0};

static i2c_request_t mag_read_req = {
    .op = I2C_OP_MEM_READ,
    .dev_addr = MAG_ADDRESS,
    .reg_addr = OUT_X_H_M | 0x80,
    .rx_buf = mag_rx_buf,
    .rx_len = sizeof(mag_rx_buf),
    .tx_buf = NULL,
    .tx_len = 0,
    .done_sem = NULL,
};

static i2c_request_t* req_to_send;

void MagnetometerTask(void *pvParameters) {
    ImuSensor_t* imu = (ImuSensor_t*) pvParameters;
    TickType_t xLastWakeTime;

    printf("Magnetometer task started\r\n");

    xLastWakeTime = xTaskGetTickCount();

    mag_read_req.done_sem = xSemaphoreCreateBinary();
    xSemaphoreTake(mag_read_req.done_sem, 0);

    while (1) {
        req_to_send = &mag_read_req;

        if (xQueueSend(i2c_queue, &req_to_send, 0) == pdPASS) {
            if (xSemaphoreTake(mag_read_req.done_sem, portMAX_DELAY) == pdTRUE) {
                parse_mag_data(mag_rx_buf, imu);
                printf("Magnetometer data: X = %7d, Y = %7d, Z = %7d\r\n", imu->mag[0], imu->mag[1], imu->mag[2]);
            } else {
                printf("Failed to read Magnetometer data\r\n");
            }
        }
        xSemaphoreGive(i2cSemaphore);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

void parse_mag_data(uint8_t *rx_buf, ImuSensor_t *dev) {
    dev->mag[0] = (int16_t)((rx_buf[0] << 8) | rx_buf[1]);
    dev->mag[1] = (int16_t)((rx_buf[4] << 8) | rx_buf[5]);
    dev->mag[2] = (int16_t)((rx_buf[2] << 8) | rx_buf[3]);
}
