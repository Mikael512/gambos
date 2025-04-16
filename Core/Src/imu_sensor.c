/*
 * lsm303dlhc.c
 *
 *  Created on: Feb 28, 2025
 *      Author: Mikael Marvin
 */

#include "imu_sensor.h"
#include "i2c_request.h"
#include "helper.h"
#include <stdio.h>


static uint8_t acc_rx_buf[6];
static uint8_t mfield_rx_buf[6];
static uint8_t gyro_rx_buf[6];

static uint8_t data1 = 0x67;
static uint8_t data2 = 0x1C;
static uint8_t data3 = 0x20;
static uint8_t data4 = 0x00;
static uint8_t data5 = 0x7F;
static uint8_t data6 = 0x29;

static i2c_request_t acc_read_req = {
		.dev_addr = ACC_ADDRESS,
		.reg_addr = OUT_X_L_A | 0x80,
		.tx_buf = NULL,
		.rx_buf = acc_rx_buf,
		.len = RX_BUFFER_SIZE,
		.user_data = NULL,
		.parse_fn = parse_acc_data,
		.op_type = I2C_OP_MEM_READ,
		.i2c_call.mem_read = HAL_I2C_Mem_Read_IT,
};

static i2c_request_t mfield_read_req = {
		.dev_addr = MFIELD_ADDRESS,
		.reg_addr = OUT_X_H_M,
		.tx_buf = NULL,
		.rx_buf = mfield_rx_buf,
		.len = RX_BUFFER_SIZE,
		.user_data = NULL,
		.parse_fn = parse_mfield_data,
		.op_type = I2C_OP_MEM_READ,
		.i2c_call.mem_read = HAL_I2C_Mem_Read_IT,
};

static i2c_request_t gyro_read_req = {
		.dev_addr = GYRO_ADDRESS,
		.reg_addr = OUT_X_L | 0x80,
		.tx_buf = NULL,
		.rx_buf = gyro_rx_buf,
		.len = RX_BUFFER_SIZE,
		.user_data = NULL,
		.parse_fn = parse_gyro_data,
		.op_type = I2C_OP_MEM_READ,
		.i2c_call.mem_read = HAL_I2C_Mem_Read_IT,
};

static i2c_request_t write_request1 = {
    .dev_addr = ACC_ADDRESS,
    .reg_addr = CTRL_REG1_A,
    .tx_buf = &data1,
    .rx_buf = NULL,
    .len = 1,
    .user_data = NULL,
	.parse_fn = NULL,
    .op_type = I2C_OP_MEM_WRITE,
    .i2c_call.mem_write = HAL_I2C_Mem_Write_IT,
};

static i2c_request_t write_request2 = {
    .dev_addr = MFIELD_ADDRESS,
    .reg_addr = CRA_REG_M,
    .tx_buf = &data2,
    .rx_buf = NULL,
    .len = 1,
    .user_data = NULL,
	.parse_fn = NULL,
    .op_type = I2C_OP_MEM_WRITE,
    .i2c_call.mem_write = HAL_I2C_Mem_Write_IT,
};

static i2c_request_t write_request3 = {
    .dev_addr = MFIELD_ADDRESS,
    .reg_addr = CRB_REG_M,
    .tx_buf = &data3,
    .rx_buf = NULL,
    .len = 1,
    .user_data = NULL,
	.parse_fn = NULL,
    .op_type = I2C_OP_MEM_WRITE,
    .i2c_call.mem_write = HAL_I2C_Mem_Write_IT,
};

static i2c_request_t write_request4 = {
    .dev_addr = MFIELD_ADDRESS,
    .reg_addr = MR_REG_M,
    .tx_buf = &data4,
    .rx_buf = NULL,
    .len = 1,
    .user_data = NULL,
	.parse_fn = NULL,
    .op_type = I2C_OP_MEM_WRITE,
    .i2c_call.mem_write = HAL_I2C_Mem_Write_IT,
};

static i2c_request_t write_request5 = {
    .dev_addr = GYRO_ADDRESS,
    .reg_addr = CTRL_REG1,
    .tx_buf = &data5,
    .rx_buf = NULL,
    .len = 1,
    .user_data = NULL,
	.parse_fn = NULL,
    .op_type = I2C_OP_MEM_WRITE,
    .i2c_call.mem_write = HAL_I2C_Mem_Write_IT,
};

static i2c_request_t write_request6 = {
    .dev_addr = GYRO_ADDRESS,
    .reg_addr = CTRL_REG2,
    .tx_buf = &data6,
    .rx_buf = NULL,
    .len = 1,
    .user_data = NULL,
	.parse_fn = NULL,
    .op_type = I2C_OP_MEM_WRITE,
    .i2c_call.mem_write = HAL_I2C_Mem_Write_IT,
};



HAL_StatusTypeDef imu_sensor_initialize(ImuSensor_t* dev, I2C_HandleTypeDef* i2c_handle) {
	dev->i2c_handle = i2c_handle;

	for (uint8_t i = 0; i < 3; ++i){
		dev->acc[i] = 0;
		dev->mfield[i] = 0;
		dev->gyro[i] = 0;
	}

	// Add ImuSensor_t reference to the request structure
	acc_read_req.user_data = dev;
	mfield_read_req.user_data = dev;
	gyro_read_req.user_data = dev;

	// Accelerometer output data rate (ODR) 200 Hz, normal mode, all axes enabled
	i2c_submit_request(dev->i2c_handle, &write_request1);

	// Magnetometer output data rate (ODR) 220 Hz
	i2c_submit_request(dev->i2c_handle, &write_request2);

	// Magnetometer range +/-1.3 Gauss
	i2c_submit_request(dev->i2c_handle, &write_request3);

	// Magnetometer continuous conversion mode
	i2c_submit_request(dev->i2c_handle, &write_request4);

	// Gyroscope output data rate (ODR) 190 Hz and bandwidth (BW) 70 Hz
	i2c_submit_request(dev->i2c_handle, &write_request5);

	// Gyroscope high-pass filter normal mode and high-pass filter cut off frequency 0.018 Hz
	i2c_submit_request(dev->i2c_handle, &write_request6);

	return HAL_OK;
}

void imu_sensor_read_acc(ImuSensor_t* dev) {
	i2c_submit_request(dev->i2c_handle, &acc_read_req);
}

void imu_sensor_read_mfield(ImuSensor_t* dev) {
	i2c_submit_request(dev->i2c_handle, &mfield_read_req);
}

void imu_sensor_read_gyro(ImuSensor_t* dev) {
	i2c_submit_request(dev->i2c_handle, &gyro_read_req);
}

void parse_acc_data(uint8_t *rx_buf, void *user_data) {
	ImuSensor_t *dev = (ImuSensor_t *)user_data;
	dev->acc[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
	dev->acc[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
	dev->acc[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
}

void parse_mfield_data(uint8_t *rx_buf, void *user_data) {
	ImuSensor_t *dev = (ImuSensor_t *)user_data;
	dev->acc[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
	dev->acc[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
	dev->acc[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
}

void parse_gyro_data(uint8_t *rx_buf, void *user_data) {
	ImuSensor_t *dev = (ImuSensor_t *)user_data;
	dev->acc[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
	dev->acc[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
	dev->acc[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
}
