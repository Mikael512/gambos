#include "imu_sensor.h"
#include "i2c_task.h"
#include <stdio.h>


// Define configuration values
static uint8_t acc_ctrl   = 0x67;
static uint8_t mag_cra    = 0x1C;
static uint8_t mag_crb    = 0x20;
static uint8_t mag_mr     = 0x00;
static uint8_t gyr_ctrl1  = 0x7F;
static uint8_t gyr_ctrl2  = 0x29;

// Requests (same order as above)
static i2c_request_t imu_config_requests[] = {
    {
        .op = I2C_OP_MEM_WRITE,
        .dev_addr = ACC_ADDRESS,
        .reg_addr = CTRL_REG1_A,
        .tx_buf = &acc_ctrl,
        .tx_len = 1,
        .rx_buf = NULL,
        .rx_len = 0,
        .done_sem = NULL,
    },
    {
        .op = I2C_OP_MEM_WRITE,
        .dev_addr = MFIELD_ADDRESS,
        .reg_addr = CRA_REG_M,
        .tx_buf = &mag_cra,
        .tx_len = 1,
        .rx_buf = NULL,
        .rx_len = 0,
        .done_sem = NULL,
    },
    {
        .op = I2C_OP_MEM_WRITE,
        .dev_addr = MFIELD_ADDRESS,
        .reg_addr = CRB_REG_M,
        .tx_buf = &mag_crb,
        .tx_len = 1,
        .rx_buf = NULL,
        .rx_len = 0,
        .done_sem = NULL,
    },
    {
        .op = I2C_OP_MEM_WRITE,
        .dev_addr = MFIELD_ADDRESS,
        .reg_addr = MR_REG_M,
        .tx_buf = &mag_mr,
        .tx_len = 1,
        .rx_buf = NULL,
        .rx_len = 0,
        .done_sem = NULL,
    },
    {
        .op = I2C_OP_MEM_WRITE,
        .dev_addr = GYRO_ADDRESS,
        .reg_addr = CTRL_REG1,
        .tx_buf = &gyr_ctrl1,
        .tx_len = 1,
        .rx_buf = NULL,
        .rx_len = 0,
        .done_sem = NULL,
    },
    {
        .op = I2C_OP_MEM_WRITE,
        .dev_addr = GYRO_ADDRESS,
        .reg_addr = CTRL_REG2,
        .tx_buf = &gyr_ctrl2,
        .tx_len = 1,
        .rx_buf = NULL,
        .rx_len = 0,
        .done_sem = NULL,
    }
};

volatile static i2c_request_t* req_to_send; 

HAL_StatusTypeDef imu_sensor_initialize(ImuSensor_t* dev, I2C_HandleTypeDef* i2c_handle) {
    dev->i2c_handle = i2c_handle;

    for (int i = 0; i < 3; ++i) {
        dev->acc[i] = 0;
        dev->mfield[i] = 0;
        dev->gyro[i] = 0;
    }

    const size_t count = sizeof(imu_config_requests) / sizeof(imu_config_requests[0]);
    for (size_t i = 0; i < count; ++i) {
		printf("Sending config request %p\r\n", &imu_config_requests[i]);
        req_to_send = &imu_config_requests[i];
        if (xQueueSend(i2c_queue, &req_to_send, portMAX_DELAY) != pdPASS) {
            printf("Failed to send config request %u\r\n", i);
            return HAL_ERROR;
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Optional delay between requests
    }

    return HAL_OK;
}