#ifndef INC_ACCELEROMETER_TASK_H_
#define INC_ACCELEROMETER_TASK_H_

#include "imu_sensor.h"


void AccelerometerTask(void *pvParameters);
void parse_acc_data(uint8_t *rx_buf, int16_t *data);

#endif /* INC_ACCELEROMETER_TASK_H_ */
