/*
 * helper.c
 *
 *  Created on: Mar 27, 2025
 *      Author: mikaelmarvin
 */

#include "helper.h"


const char *i2c_op_to_string(i2c_op_type_t op) {
	switch (op) {
	case I2C_OP_MEM_WRITE: return "I2C_OP_MEM_WRITE";
	case I2C_OP_MEM_READ: return "I2C_OP_MEM_READ";
	case I2C_OP_MASTER_TRANSMIT: return "I2C_OP_MASTER_TRANSMIT";
	case I2C_OP_SLAVE_TRANSMIT: return "I2C_OP_SLAVE_TRANSMIT";
	case I2C_OP_MASTER_RECEIVE: return "I2C_OP_MASTER_RECEIVE";
	case I2C_OP_SLAVE_RECEIVE: return "I2C_OP_SLAVE_RECEIVE";
	default: return "UNKNOWN_OP";
	}
}
