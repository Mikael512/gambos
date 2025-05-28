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

            // Handles case where the request is sent but done_sem has timed out 
            if (req->done_sem == NULL) {
                req = NULL;
                xSemaphoreGive(i2cSemaphore);
                continue;
            }

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

HAL_StatusTypeDef i2c_mem_read(uint16_t dev_addr, uint8_t reg_addr, uint8_t *rx_buf, uint16_t len, TickType_t timeout) {
    SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
    xSemaphoreTake(done_sem, 0); // Initialize as "not available"
    if (done_sem == NULL) return HAL_ERROR;

    i2c_request_t req = {
        .op = I2C_OP_MEM_READ,
        .dev_addr = dev_addr,
        .reg_addr = reg_addr,
        .rx_buf = rx_buf,
        .rx_len = len,
        .tx_buf = NULL,
        .tx_len = 0,
        .done_sem = done_sem
    };

    // Since xQueueSend copies the content of the passed argument and we want to 
    // push only the address of the request, we need to use another pointer
    i2c_request_t* req_to_send = &req; 

    // Send pointer to request into queue and wait if the queue is full
    if (xQueueSend(i2c_queue, &req_to_send, timeout) != pdPASS) {
        vSemaphoreDelete(done_sem);
        return HAL_ERROR;
    }

    // Wait for request to complete or timeout
    if (xSemaphoreTake(done_sem, timeout) != pdTRUE) {
        vSemaphoreDelete(done_sem);
        return HAL_TIMEOUT;
    }

    // Delete the semaphore after use since it is created on the heap
    vSemaphoreDelete(done_sem);
    return HAL_OK;
}

HAL_StatusTypeDef i2c_mem_write(uint16_t dev_addr, uint8_t reg_addr, uint8_t *tx_buf, uint16_t len, TickType_t timeout) {
    SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
    xSemaphoreTake(done_sem, 0); // Initialize as "not available"
    if (done_sem == NULL) return HAL_ERROR;

    i2c_request_t req = {
        .op = I2C_OP_MEM_WRITE,
        .dev_addr = dev_addr,
        .reg_addr = reg_addr,
        .rx_buf = NULL,
        .rx_len = 0,
        .tx_buf = tx_buf,
        .tx_len = len,
        .done_sem = done_sem
    };

    // Since xQueueSend copies the content of the passed argument and we want to 
    // push only the address of the request, we need to use another pointer
    i2c_request_t* req_to_send = &req; 

    // Send pointer to request into queue and wait if the queue is full
    if (xQueueSend(i2c_queue, &req_to_send, timeout) != pdPASS) {
        vSemaphoreDelete(done_sem);
        return HAL_ERROR;
    }

    // Wait for request to complete or timeout
    if (xSemaphoreTake(done_sem, timeout) != pdTRUE) {
        vSemaphoreDelete(done_sem);
        return HAL_TIMEOUT;
    }

    // Wait for request to complete or timeout
    vSemaphoreDelete(done_sem);
    return HAL_OK;
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



