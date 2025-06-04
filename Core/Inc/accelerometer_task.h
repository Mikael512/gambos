#ifndef INC_ACCELEROMETER_TASK_H_
#define INC_ACCELEROMETER_TASK_H_

#include "ism330dhcx.h"


void AccelerometerTask(void *pvParameters);
void parse_acc_data(uint8_t *rx_buf, int16_t *data);

#endif /* INC_ACCELEROMETER_TASK_H_ */
