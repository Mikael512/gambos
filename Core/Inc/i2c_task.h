#ifndef INC_I2C_TASK_H_
#define INC_I2C_TASK_H_

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define I2C_QUEUE_SIZE 16

extern QueueHandle_t i2c_queue;
extern SemaphoreHandle_t i2cSemaphore;

typedef enum {
    I2C_OP_MEM_WRITE,
    I2C_OP_MEM_READ,
    I2C_OP_MASTER_TRANSMIT,
    I2C_OP_MASTER_RECEIVE
} i2c_op_t;

typedef struct {
    i2c_op_t op;
    uint16_t dev_addr;
    uint16_t reg_addr;
    uint8_t *tx_buf;
    uint8_t *rx_buf;
    uint16_t tx_len;
    uint16_t rx_len;
    SemaphoreHandle_t done_sem;
} i2c_request_t;

void I2cTask(void *pvParameters);
void i2c_queue_init();

#endif /* INC_I2C_TASK_H_ */
