#ifndef INC_LOGGER_TASK_H
#define INC_LOGGER_TASK_H

#include "FreeRTOS.h"
#include <queue.h>
#include "semphr.h"
#include <string.h>      
#include <stdio.h>          
#include "stm32f4xx_hal.h"  

#define LOG_BUFFER_SIZE 64
#define LOG_QUEUE_SIZE 32


extern UART_HandleTypeDef huart2;
extern QueueHandle_t logQueue;
extern SemaphoreHandle_t dmaSemaphore;

void LoggerTask(void *pvParameters);
void logger_queue_init();

#endif /* INC_LOGGER_TASK_H */