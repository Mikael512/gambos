#include "iis2mdc.h"
#include "i2c_task.h"
#include "bager_buffer.h"
#include <stdio.h>

const int16_t hard_iron[3] = { -202, 11, 5 };
const float soft_iron[3][3] = {
    { 0.43101121, -0.00091248, -0.00066664},
    {-0.00091248, 0.4229062, 0.00168367},
    {-0.00066664, 0.00168367, 0.419305}
};

void parse_mag_data(uint8_t *rx_buf, int16_3d_t *data) {
    data->x = (int16_t)((((int16_t)rx_buf[1]) << 8) | rx_buf[0]);
    data->y = (int16_t)((((int16_t)rx_buf[3]) << 8) | rx_buf[2]);
    data->z = (int16_t)((((int16_t)rx_buf[5]) << 8) | rx_buf[4]);
}

void soft_and_hard_correction(int16_3d_t *data) {
    int16_t xm = data->x - hard_iron[0];
    int16_t ym = data->y - hard_iron[1];
    int16_t zm = data->z - hard_iron[2];

    data->x = (soft_iron[0][0] * (float)xm + soft_iron[0][1] * (float)ym + soft_iron[0][2] * (float)zm);
    data->y = (soft_iron[1][0] * (float)xm + soft_iron[1][1] * (float)ym + soft_iron[1][2] * (float)zm);
    data->z = (soft_iron[2][0] * (float)xm + soft_iron[2][1] * (float)ym + soft_iron[2][2] * (float)zm);
}


void MagnetometerTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("Magnetometer task started\r\n");

    xLastWakeTime = xTaskGetTickCount();

    uint8_t mag_rx_buf[6] = {0};
    int16_3d_t mag_data = {0};

    while (1) {
        if(i2c_mem_read(IIS2MDC, OUTX_L_REG | 0x80, mag_rx_buf, sizeof(mag_rx_buf), pdMS_TO_TICKS(10)) == HAL_OK) {
            parse_mag_data(mag_rx_buf, &mag_data);
            soft_and_hard_correction(&mag_data);
            push_data(BUFFER_MAG, &mag_data);
            // printf("Magnetometer data: X = %7d, Y = %7d, Z = %7d\r\n", mag_data.x, mag_data.y, mag_data.z);

        } else {
            printf("Failed to read Magnetometer data\r\n");
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}


