#include "i2c_task.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>

QueueHandle_t i2c_queue;
SemaphoreHandle_t i2cSemaphore;
static i2c_request_t* req = NULL;


void I2cTask(void *pvParameters) {
    I2C_HandleTypeDef* hi2c1 = (I2C_HandleTypeDef*) pvParameters;
    printf("I2C task started\r\n");

    while (1) {
        xSemaphoreTake(i2cSemaphore, portMAX_DELAY);
        if (xQueueReceive(i2c_queue, &req, portMAX_DELAY) == pdTRUE) {
            // printf("I2C request received: %p\r\n", req);
            HAL_StatusTypeDef result = HAL_ERROR;

            switch (req->op) {
                case I2C_OP_MEM_READ:
                    result = HAL_I2C_Mem_Read_IT(hi2c1, req->dev_addr, req->reg_addr, I2C_MEMADD_SIZE_8BIT, req->rx_buf, req->rx_len);
                    break;
                case I2C_OP_MEM_WRITE:
                    result = HAL_I2C_Mem_Write_IT(hi2c1, req->dev_addr, req->reg_addr, I2C_MEMADD_SIZE_8BIT, req->tx_buf, req->tx_len);
                    break;
                case I2C_OP_MASTER_TRANSMIT:
                    result = HAL_I2C_Master_Transmit_IT(hi2c1, req->dev_addr, req->tx_buf, req->tx_len);
                    break;
                case I2C_OP_MASTER_RECEIVE:
                    result = HAL_I2C_Master_Receive_IT(hi2c1, req->dev_addr, req->rx_buf, req->rx_len);
                    break;
            }
            if(result != HAL_OK) {
                printf("I2C operation failed: %d, HAL_I2C_GetError: 0x%lx\r\n", result, HAL_I2C_GetError(hi2c1));
                req = NULL;
                xSemaphoreGive(i2cSemaphore);
            }
            
        }
    }
}


void i2c_queue_init() {
	i2c_queue = xQueueCreate(I2C_QUEUE_SIZE, sizeof(i2c_request_t *));
	i2cSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(i2cSemaphore);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	// printf("I2C Rx Complete\r\n");
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(i2cSemaphore, &xHigherPriorityTaskWoken);

    if (req && req->done_sem) {
        xSemaphoreGiveFromISR(req->done_sem, &xHigherPriorityTaskWoken);
        req = NULL;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // printf("I2C Tx Complete\r\n");
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(i2cSemaphore, &xHigherPriorityTaskWoken);

    if (req && req->done_sem) {
        xSemaphoreGiveFromISR(req->done_sem, &xHigherPriorityTaskWoken);
        req = NULL;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    printf("I2C Error: 0x%lx\r\n", hi2c->ErrorCode);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(i2cSemaphore, &xHigherPriorityTaskWoken);

    if (req && req->done_sem) {
        xSemaphoreGiveFromISR(req->done_sem, &xHigherPriorityTaskWoken);
        req = NULL;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



