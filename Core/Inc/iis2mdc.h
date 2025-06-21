#ifndef INC_IIS2MDC_H
#define INC_IIS2MDC_H

#include "stm32f4xx_hal.h"

// Device addresses
#define IIS2MDC 			((uint8_t) 0x1E << 1)
#define IIS2MDC_WHOAMI      ((uint8_t) 0x4F)        // outputs 0x40

// Magnetometer registers
#define CFG_REG_A			((uint8_t) 0x60) // write 0x88 for 50 Hz and continuous mode
#define CFG_REG_B			((uint8_t) 0x61)
#define OUTX_L_REG			((uint8_t) 0x68)
#define OUTY_L_REG			((uint8_t) 0x6A)
#define OUTZ_L_REG			((uint8_t) 0x6C)

/*
 * Tasks for sensor data reading
 */
void MagnetometerTask(void *pvParameters);

#endif /* INC_IIS2MDC_H */
