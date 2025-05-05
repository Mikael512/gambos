/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "imu_sensor.h"
#include "i2c_request.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_BUFFER_SIZE 128
#define LOG_QUEUE_SIZE 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;


/* USER CODE BEGIN PV */
static QueueHandle_t logQueue;
static SemaphoreHandle_t dmaSemaphore;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	if (len > LOG_BUFFER_SIZE) len = LOG_BUFFER_SIZE;  // Prevent overflow

	char logMessage[LOG_BUFFER_SIZE];
	memcpy(logMessage, ptr, len);
	logMessage[len] = '\0';  // Ensure null termination

	// Try to enqueue the message (fail if the queue is full)
	xQueueSendFromISR(logQueue, logMessage, NULL);

	return len;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		xSemaphoreGiveFromISR(dmaSemaphore, NULL);  // Release DMA for the next message
	}
}

void AccelerometerTask(void *pvParameters) {
  ImuSensor_t* imu = (ImuSensor_t*) pvParameters;
	TickType_t xLastWakeTime;

  printf("Accelerometer task started\r\n");

	xLastWakeTime = xTaskGetTickCount();
  SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();

  uint8_t acc_rx_buf[6] = {0}; // Buffer to hold accelerometer data

  while (1) {
      // Create a request to read 2 bytes from register 0x00
      i2c_request_t req = {
          .op = I2C_OP_MEM_READ,
          .dev_addr = ACC_ADDRESS,
          .reg_addr = OUT_X_L_A | 0x80,
          .rx_buf = acc_rx_buf,
          .rx_len = sizeof(acc_rx_buf),
          .tx_buf = NULL,
          .tx_len = 0,
          .done_sem = done_sem
      };

      // Send the request to the I2C task
      if (xQueueSend(i2c_request_queue, &req, portMAX_DELAY) == pdPASS) {
          // Block here until the I2C task signals completion
          if (xSemaphoreTake(done_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
              // Successfully read data — parse it
              parse_acc_data(acc_rx_buf, imu);
              printf("Accelerometer data: X = %7d, Y = %7d, Z = %7d\r\n", imu->acc[0], imu->acc[1], imu->acc[2]);
          } else {
              // Timeout or failure — handle it
              printf("Failed to read Accelerometer data\r\n");
          }
      }

      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(3000)); // 200Hz
  }
}

void MagnetometerTask(void *pvParameters) {
	ImuSensor_t* imu = (ImuSensor_t*) pvParameters;
	TickType_t xLastWakeTime;

	// Initialize the last wake time to the current time
	xLastWakeTime = xTaskGetTickCount();

	while(1) {
		imu_sensor_read_mfield(imu); // Example sensor reading function
		printf("Magnetometer data: X = %7d, Y = %7d, Z = %7d\r\n", imu->mfield[0], imu->mfield[1], imu->mfield[2]);

		// Delay until 100ms after the last wake time
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5)); // 200Hz
	}
}

void GyroscopeTask(void *pvParameters) {
	ImuSensor_t* imu = (ImuSensor_t*) pvParameters;
	TickType_t xLastWakeTime;

	// Initialize the last wake time to the current time
	xLastWakeTime = xTaskGetTickCount();

	while(1) {
		imu_sensor_read_gyro(imu); // Example sensor reading function
		printf("Gyroscope data: X = %7d, Y = %7d, Z = %7d\r\n", imu->gyro[0], imu->gyro[1], imu->gyro[2]);

		// Delay until 100ms after the last wake time
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5)); // 200Hz
	}
}

void ImuProcessingTask(void *pvParameters) {
	//ImuSensor_t* imu = (ImuSensor_t*) pvParameters;
	TickType_t xLastWakeTime;

	// Initialize the last wake time to the current time
	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		// Example: process the IMU sensor data
		// Read acceleration, gyroscope, and magnetometer values
		// Process the data (e.g., filtering, calculations, etc.)

		// Delay until 100ms after the last wake time
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));  // 100ms delay
	}
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

void I2cTask(void *pvParameters) {
  i2c_request_t req;
  printf("I2C task started\r\n");

  while (1) {
      if (xQueueReceive(i2c_request_queue, &req, portMAX_DELAY) == pdTRUE) {
          HAL_StatusTypeDef result = HAL_ERROR;
          printf("I2C request: %p\r\n", &req);

          switch (req.op) {
              case I2C_OP_MEM_READ:
                  result = HAL_I2C_Mem_Read_IT(&hi2c1, req.dev_addr << 1, req.reg_addr,
                                            I2C_MEMADD_SIZE_8BIT,
                                            req.rx_buf, req.rx_len);
                  break;

              case I2C_OP_MEM_WRITE:
                  result = HAL_I2C_Mem_Write_IT(&hi2c1, req.dev_addr << 1, req.reg_addr,
                                             I2C_MEMADD_SIZE_8BIT,
                                             req.tx_buf, req.tx_len);
                  break;

              case I2C_OP_MASTER_TRANSMIT:
                  result = HAL_I2C_Master_Transmit_IT(&hi2c1, req.dev_addr << 1,
                                                   req.tx_buf, req.tx_len);
                  break;

              case I2C_OP_MASTER_RECEIVE:
                  result = HAL_I2C_Master_Receive_IT(&hi2c1, req.dev_addr << 1,
                                                  req.rx_buf, req.rx_len);
                  break;
          }

          // Signal completion only if successful (or always, if you prefer)
          if (req.done_sem) {
              xSemaphoreGive(req.done_sem);
          }
      }
  }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  logQueue = xQueueCreate(LOG_QUEUE_SIZE, LOG_BUFFER_SIZE);
	dmaSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(dmaSemaphore);  // Initialize as "available"

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_DMA_Init();
	NVIC_SetPriorityGrouping(0);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize i2c queue for general communication
  i2c_request_queue_init();

  ImuSensor_t imu;
	printf("imu address: %p\r\n", &imu);
	imu_sensor_initialize(&imu, &hi2c1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  BaseType_t result;
  result = xTaskCreate(AccelerometerTask, "Accelerometer task", 512, &imu, 1, NULL);
  if (result != pdPASS) {
    printf("Failed to create Accelerometer task!\r\n");
  }
	result = xTaskCreate(LoggerTask, "Logger task", 1024, NULL, 1, NULL);
  if (result != pdPASS) {
    printf("Failed to create Logger task!\r\n");
  }
  result = xTaskCreate(I2cTask, "I2c task", 512, NULL, 1, NULL);
  if (result != pdPASS) {
      printf("Failed to create I2cTask!\r\n");
  }
	//xTaskCreate(MagnetometerTask, "Magnetometer task", 128, &imu, 1, NULL);
	//xTaskCreate(GyroscopeTask, "Gyroscope task", 128, NULL, 1, &imu);
	//xTaskCreate(ImuProcessingTask, "Imu processing task", 128, &imu, 1, NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(90);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
