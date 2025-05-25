#ifndef INC_MAGNETOMETER_TASK_H_
#define INC_MAGNETOMETER_TASK_H_

#include <stdio.h>
#include "imu_sensor.h"
#include "i2c_task.h"
#include "semphr.h"

void MagnetometerTask(void *pvParameters);
void parse_mag_data(uint8_t *rx_buf, ImuSensor_t *dev);

#endif /* INC_MAGNETOMETER_TASK_H_ */