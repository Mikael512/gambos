/*
 * imu_sensor.h
 *
 *  Created on: Feb 28, 2025
 *      Author: Mikael Marvin
 */

#ifndef INC_IMU_SENSOR_H_
#define INC_IMU_SENSOR_H_

#include "stm32f4xx_hal.h"

// Device addresses
#define ACC_ADDRESS			(0x19 << 1)
#define MFIELD_ADDRESS		(0x1E << 1)
#define GYRO_ADDRESS		(0x6A << 1) // or 6B, 6A

// Accelerometer registers
#define CTRL_REG1_A			0x20
#define OUT_X_L_A			0x28
#define OUT_Y_L_A			0x2A
#define OUT_Z_L_A			0x2C

// Magnetomoeter registers
#define CRA_REG_M			0x00
#define CRB_REG_M			0x01
#define MR_REG_M			0x02
#define OUT_X_H_M			0x03
#define OUT_Z_H_M			0x05
#define OUT_Y_H_M			0x07

// Accelerometer and magnetomoter temperature sensor
#define TEMP_OUT_H_M		0x31

// Gyroscope registers
#define WHO_AM_I			0x0F // should output 0xD4
#define CTRL_REG1			0x20
#define CTRL_REG2			0x21
#define OUT_X_L				0x28
#define OUT_Y_L				0x2A
#define OUT_Z_L				0x2C

// Buffer used when parsing after getting the sensor data
#define RX_BUFFER_SIZE	6


void parse_acc_data(uint8_t *rx_buf, void *user_data);
void parse_mfield_data(uint8_t *rx_buf, void *user_data);
void parse_gyro_data(uint8_t *rx_buf, void *user_data);
/*
 * IMU sensor structure
 */
typedef struct {
	I2C_HandleTypeDef* i2c_handle;
	int16_t acc[3];
	int16_t mfield[3];
	int16_t gyro[3];
} ImuSensor_t;

/*
 * Initialization
 */
HAL_StatusTypeDef imu_sensor_initialize(ImuSensor_t* dev, I2C_HandleTypeDef* i2c_handle);

/*
 * Data acquisition
 */
void imu_sensor_read_acc(ImuSensor_t* dev);
void imu_sensor_read_mfield(ImuSensor_t* dev);
void imu_sensor_read_gyro(ImuSensor_t* dev);

#endif /* INC_IMU_SENSOR_H_ */
