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

QueueHandle_t i2c_request_queue;

void choose_i2c_call(i2c_request_t *req, I2C_HandleTypeDef *i2c){
	HAL_StatusTypeDef error;
	switch (req->op_type) {
	case I2C_OP_MEM_READ:
		error = req->i2c_call.mem_read(i2c, req->dev_addr, req->reg_addr, I2C_MEMADD_SIZE_8BIT, req->rx_buf, req->len);
		break;
	case I2C_OP_MEM_WRITE:
		error = req->i2c_call.mem_write(i2c, req->dev_addr, req->reg_addr, I2C_MEMADD_SIZE_8BIT, req->tx_buf, req->len);
		break;
	case I2C_OP_MASTER_TRANSMIT:
		error = req->i2c_call.master_transmit(i2c, req->dev_addr, req->tx_buf, req->len);
		break;
	case I2C_OP_MASTER_RECEIVE:
		error = req->i2c_call.master_receive(i2c, req->dev_addr, req->rx_buf, req->len);
		break;
	default:
		printf("Unsupported I2C operation for item: %p.\r\n", req);
		error = HAL_ERROR;
		break;
	}

	if (error != HAL_OK) {
		printf("Failed to call i2c function for item: %p (err %d)\r\n", req, error);
	}
}

void i2c_request_queue_init() {
	i2c_request_queue = xQueueCreate(I2C_QUEUE_SIZE, sizeof(i2c_request_t *));
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	i2c_request_t *req = NULL;
	if (xQueueReceiveFromISR(i2c_request_queue, &req, NULL) == pdPASS) {
		if (req->parse_fn != NULL) {
			req->parse_fn(req->rx_buf, req->user_data);
		}
	}
	printf("Processed: %p, Q occupated: %ld\r\n", req, uxQueueMessagesWaitingFromISR(i2c_request_queue));
	if (xQueuePeekFromISR(i2c_request_queue, &req) == pdPASS) {
		choose_i2c_call(req, hi2c);
	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	i2c_request_t *req = NULL;
	if (xQueueReceiveFromISR(i2c_request_queue, &req, NULL) == pdPASS) {
		//printf("I2C Mem Write Callback processed.\r\n");
	}


	printf("Processed: %p, Q occupated: %ld\r\n", req, uxQueueMessagesWaitingFromISR(i2c_request_queue));
	if (xQueuePeekFromISR(i2c_request_queue, &req) == pdPASS) {
		choose_i2c_call(req, hi2c);
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	i2c_request_t *req = NULL;

	// Remove the failed request from the queue
	if (xQueueReceiveFromISR(i2c_request_queue, &req, NULL) == pdPASS) {
		printf("Request (%p) removed from queue due to error (0x%08lX), Q occupated: %ld\r\n", req, hi2c->ErrorCode, uxQueueMessagesWaitingFromISR(i2c_request_queue));
	}

	// Try the next one in queue
	if (xQueuePeekFromISR(i2c_request_queue, &req) == pdPASS) {
		choose_i2c_call(req, hi2c);
	}
}

HAL_StatusTypeDef i2c_submit_request(I2C_HandleTypeDef *i2c, i2c_request_t *req) {
	printf("Send to process: %p, Q occupated: %ld\r\n", req, uxQueueMessagesWaiting(i2c_request_queue));
	i2c_request_t * const req_to_send = req;
	BaseType_t status = xQueueSend(i2c_request_queue, &req_to_send, 0);
	if (status != pdPASS) {
		printf("Queue full: Failed to queue item: %p.\r\n", req);
		return HAL_BUSY;
	}

	if (uxQueueMessagesWaiting(i2c_request_queue) == 1) {
		choose_i2c_call(req, i2c);
		return HAL_OK;
	}

	return HAL_OK;
}

