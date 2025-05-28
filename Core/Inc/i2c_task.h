#ifndef INC_I2C_TASK_H_
#define INC_I2C_TASK_H_

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define I2C_QUEUE_SIZE 16

/**
 * @brief Handle for the I2C request queue.
 * 
 * Queue stores i2c_request_t objects to be processed by I2cTask.
 */
extern QueueHandle_t i2c_queue;

/**
 * @brief Semaphore used to synchronize access to the I2C bus.
 */
extern SemaphoreHandle_t i2cSemaphore;

/**
 * @brief Enum defining types of I2C operations supported.
 */
typedef enum {
    I2C_OP_MEM_WRITE,       /**< Memory write operation */
    I2C_OP_MEM_READ,        /**< Memory read operation */
    I2C_OP_MASTER_TRANSMIT, /**< Master transmit without register */
    I2C_OP_MASTER_RECEIVE   /**< Master receive without register */
} i2c_op_t;

/**
 * @brief Structure defining a single I2C request.
 */
typedef struct {
    i2c_op_t op;                    /**< Operation type */
    uint16_t dev_addr;              /**< 7-bit device address (shifted or left-aligned) */
    uint16_t reg_addr;              /**< Register/memory address for MEM operations */
    uint8_t *tx_buf;                /**< Pointer to transmit buffer */
    uint8_t *rx_buf;                /**< Pointer to receive buffer */
    uint16_t tx_len;                /**< Number of bytes to transmit */
    uint16_t rx_len;                /**< Number of bytes to receive */
    SemaphoreHandle_t done_sem;     /**< Semaphore to signal completion */
} i2c_request_t;

/**
 * @brief FreeRTOS task that processes I2C requests from the queue.
 * 
 * This task runs indefinitely, waiting for requests to appear on the queue,
 * performs the requested I2C operation, and signals completion via the done_sem.
 * 
 * @param pvParameters I2c_HandleTypeDef pointer passed from the main application.
 */
void I2cTask(void *pvParameters);

/**
 * @brief Initialize the I2C queue and related synchronization objects.
 * 
 * Must be called before using the I2C interface.
 */
void i2c_queue_init(void);

/**
 * @brief Perform a blocking I2C memory read operation.
 * 
 * Sends a read request to the I2C task queue and waits for completion.
 * 
 * @param dev_addr 7-bit I2C device address (left-aligned or shifted)
 * @param reg_addr Register address to read from
 * @param rx_buf Buffer to store read data
 * @param len Number of bytes to read
 * @param timeout Timeout in RTOS ticks for queue send and semaphore wait
 * @return HAL_StatusTypeDef HAL status of the operation
 */
HAL_StatusTypeDef i2c_mem_read(uint16_t dev_addr, uint8_t reg_addr, uint8_t *rx_buf, uint16_t len, TickType_t timeout);

/**
 * @brief Perform a blocking I2C memory write operation.
 * 
 * Sends a write request to the I2C task queue and waits for completion.
 * 
 * @param dev_addr 7-bit I2C device address (left-aligned or shifted)
 * @param reg_addr Register address to write to
 * @param tx_buf Buffer containing data to write
 * @param len Number of bytes to write
 * @param timeout Timeout in RTOS ticks for queue send and semaphore wait
 * @return HAL_StatusTypeDef HAL status of the operation
 */
HAL_StatusTypeDef i2c_mem_write(uint16_t dev_addr, uint8_t reg_addr, uint8_t *tx_buf, uint16_t len, TickType_t timeout);

#endif /* INC_I2C_TASK_H_ */
