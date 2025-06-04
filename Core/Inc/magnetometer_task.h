#ifndef INC_MAGNETOMETER_TASK_H_
#define INC_MAGNETOMETER_TASK_H_

#include <stdio.h>
#include "ism330dhcx.h"
#include "i2c_task.h"
#include "semphr.h"

void MagnetometerTask(void *pvParameters);
void parse_mag_data(uint8_t *rx_buf, int16_t *data);

#endif /* INC_MAGNETOMETER_TASK_H_ */