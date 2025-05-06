/*
 * i2c_request.c
 *
 *  Created on: Mar 22, 2025
 *      Author: mikaelmarvin
 */

#include "i2c_request.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>

QueueHandle_t i2c_request_queue = NULL;
SemaphoreHandle_t i2cSemaphore = NULL;
i2c_request_t *current_request = NULL;


void i2c_request_queue_init() {
	i2c_request_queue = xQueueCreate(I2C_QUEUE_SIZE, sizeof(i2c_request_t *));
	i2cSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(i2cSemaphore);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	xSemaphoreGiveFromISR(i2cSemaphore, NULL);

	if (current_request && current_request->done_sem) {
        xSemaphoreGiveFromISR(current_request->done_sem, NULL);
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	printf("I2C Tx Complete\r\n");
	xSemaphoreGiveFromISR(i2cSemaphore, NULL);

}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	printf("I2C Error\r\n");
	xSemaphoreGiveFromISR(i2cSemaphore, NULL);

	if (current_request && current_request->done_sem) {
        xSemaphoreGiveFromISR(current_request->done_sem, NULL);
    }
}



