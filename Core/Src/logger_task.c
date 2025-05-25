#include "logger_task.h"

QueueHandle_t logQueue;
SemaphoreHandle_t dmaSemaphore;


int _write(int file, char *ptr, int len) {
	if (len > LOG_BUFFER_SIZE) len = LOG_BUFFER_SIZE;  // Prevent overflow

	char logMessage[LOG_BUFFER_SIZE];
	memcpy(logMessage, ptr, len);
	logMessage[len] = '\0';  // Ensure null termination

	// Try to enqueue the message (fail if the queue is full)
	xQueueSendFromISR(logQueue, logMessage, NULL);

	return len;
}

void LoggerTask(void *pvParameters) {
	char logMessage[LOG_BUFFER_SIZE];
    printf("Logger task started\r\n");

	while (1) {
		// Wait until DMA is free before sending the next message
		xSemaphoreTake(dmaSemaphore, portMAX_DELAY);
		// Wait for a new log message
		if (xQueueReceive(logQueue, logMessage, portMAX_DELAY) == pdTRUE) {
			// Start DMA transfer (non-blocking)
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)logMessage, strlen(logMessage));
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		xSemaphoreGiveFromISR(dmaSemaphore, NULL);  // Release DMA for the next message
	}
}

void logger_queue_init() {
	logQueue = xQueueCreate(LOG_QUEUE_SIZE, LOG_BUFFER_SIZE);
	dmaSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(dmaSemaphore);  // Initialize as "available"
}