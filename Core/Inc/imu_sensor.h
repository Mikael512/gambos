#ifndef INC_IMU_SENSOR_H_
#define INC_IMU_SENSOR_H_

#include "stm32f4xx_hal.h"

// Device addresses
#define ACC_ADDRESS			((uint8_t) 0x19 << 1)
#define MAG_ADDRESS			((uint8_t) 0x1E << 1)
#define GYRO_ADDRESS		((uint8_t) 0x6B << 1) // or 6B, 6A

// Accelerometer registers
#define CTRL_REG1_A			((uint8_t) 0x20)
#define OUT_X_L_A			((uint8_t) 0x28)
#define OUT_Y_L_A			((uint8_t) 0x2A)
#define OUT_Z_L_A			((uint8_t) 0x2C)

// Magnetomoeter registers
#define CRA_REG_M			((uint8_t) 0x00)
#define CRB_REG_M			((uint8_t) 0x01)
#define MR_REG_M			((uint8_t) 0x02)
#define OUT_X_H_M			((uint8_t) 0x03)
#define OUT_Z_H_M			((uint8_t) 0x05)
#define OUT_Y_H_M			((uint8_t) 0x07)

// Accelerometer and magnetomoter temperature sensor
#define TEMP_OUT_H_M		((uint8_t) 0x31)

// Gyroscope registers
#define WHO_AM_I			((uint8_t) 0x0F) // should output 0xD4
#define CTRL_REG1			((uint8_t) 0x20)
#define CTRL_REG2			((uint8_t) 0x21)
#define OUT_X_L				((uint8_t) 0x28)
#define OUT_Y_L				((uint8_t) 0x2A)
#define OUT_Z_L				((uint8_t) 0x2C)


/*
 * IMU sensor structure
 */
typedef struct {
	int16_t acc[3];
	int16_t mag[3];
	int16_t gyro[3];
} ImuSensor_t;


/*
 * Initialization task
 */
void SensorInitTask(void *pvParameters);

#endif /* INC_IMU_SENSOR_H_ */
