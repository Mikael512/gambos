#ifndef INC_ISM330DHCX_H_
#define INC_ISM330DHCX_H_

#include "stm32f4xx_hal.h"

// Device addresses
#define ISM330DHCX			((uint8_t) 0x6B << 1)
#define WHO_AM_I            ((uint8_t) 0x0F)        // outputs 0x6B

// Accelerometer registers
#define CTRL1_XL			((uint8_t) 0x10) // write 0x40 for 104 Hz and 2g scale
#define OUTX_L_A			((uint8_t) 0x28)
#define OUTY_L_A			((uint8_t) 0x2A)
#define OUTZ_L_A			((uint8_t) 0x2C)

// Gyroscope registers
#define CTRL2_G			    ((uint8_t) 0x11) // write 0x40 for 104 Hz and 250 dps scale
#define OUTX_L_G			((uint8_t) 0x22)
#define OUTY_L_G			((uint8_t) 0x24)
#define OUTZ_L_G			((uint8_t) 0x26)


/*
 * Initialization task
 */
void SensorInitTask(void *pvParameters);

#endif /* INC_ISM330DHCX_H_ */
