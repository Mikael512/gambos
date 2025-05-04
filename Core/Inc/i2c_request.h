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

QueueHandle_t i2c_request_queue;
SemaphoreHandle_t i2cSemaphore;
uint32_t i2c_request_processed;

// Enum for supported I2C operations
typedef enum {
    I2C_OP_MEM_WRITE,
    I2C_OP_MEM_READ,
    I2C_OP_MASTER_TRANSMIT,
    I2C_OP_SLAVE_TRANSMIT,
    I2C_OP_MASTER_RECEIVE,
    I2C_OP_SLAVE_RECEIVE
} i2c_op_type_t;

typedef struct {
    uint8_t dev_addr;
    uint8_t reg_addr;
    uint8_t *tx_buf;
    uint8_t *rx_buf;
    uint16_t len;
    void *user_data;  // pointer to struct like ImuSensor_t
    void (*parse_fn)(uint8_t *rx_buf, void *user_data);
    i2c_op_type_t op_type;
    union {
        HAL_StatusTypeDef (*mem_read)(I2C_HandleTypeDef *i2c, uint8_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
        HAL_StatusTypeDef (*mem_write)(I2C_HandleTypeDef *i2c, uint8_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
        HAL_StatusTypeDef (*master_transmit)(I2C_HandleTypeDef *i2c, uint8_t DevAddress, uint8_t *pData, uint16_t Size);
        HAL_StatusTypeDef (*master_receive)(I2C_HandleTypeDef *i2c, uint8_t DevAddress, uint8_t *pData, uint16_t Size);
    } i2c_call; // This will hold specific function pointers for each I2C operation
} i2c_request_t;

/*
 * Initialize the i2c queue that handles interrupt based i2c requests
 */
void i2c_request_queue_init();

/*
 * Add request to the i2c queue
 */
HAL_StatusTypeDef i2c_submit_request(I2C_HandleTypeDef *i2c, i2c_request_t *req);

HAL_StatusTypeDef choose_i2c_call(i2c_request_t *req, I2C_HandleTypeDef *i2c);

#endif /* INC_I2C_REQUEST_H_ */
