/*
 * i2c_comm_it.h
 *
 *  Created on: Mar 22, 2025
 *      Author: Mikael Marvin
 */

#ifndef INC_I2C_REQUEST_H_
#define INC_I2C_REQUEST_H_

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define I2C_QUEUE_SIZE 32

extern QueueHandle_t i2c_request_queue;
extern SemaphoreHandle_t i2cSemaphore;

// Enum for supported I2C operations
typedef enum {
    I2C_OP_MEM_WRITE,
    I2C_OP_MEM_READ,
    I2C_OP_MASTER_TRANSMIT,
    I2C_OP_MASTER_RECEIVE
} i2c_op_t;

typedef struct {
    i2c_op_t op;
    uint16_t dev_addr;
    uint16_t reg_addr; // Only used for MEM ops
    uint8_t *tx_buf;
    uint8_t *rx_buf;
    size_t tx_len;
    size_t rx_len;
    SemaphoreHandle_t done_sem;
} i2c_request_t;

/*
 * Initialize the i2c queue that handles interrupt based i2c requests
 */
void i2c_request_queue_init();

#endif /* INC_I2C_REQUEST_H_ */
