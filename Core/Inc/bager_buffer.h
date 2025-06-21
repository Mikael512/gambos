#ifndef BAGER_BUFFER_H
#define BAGER_BUFFER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// Custom data types
typedef struct __attribute__((packed)) {
    int16_t x;
    int16_t y;
    int16_t z;
} int16_3d_t;

typedef enum {
    BUFFER_ACC = 0,
    BUFFER_GYRO,
    BUFFER_MAG,
    BUFFER_COUNT
} bager_buffer_id_t;

// Circular buffer struct for any type
typedef struct {
    uint32_t head;
    uint32_t element_size;
    uint32_t buffer_size;
    uint8_t *data;  // The buffer is statically defined with the macro DEFINE_BUFFER ... 
} circular_buffer_t;

// Full buffer with multiple consumer tails
typedef struct {
    circular_buffer_t buffer;
    uint32_t *consumer_tails;    // The tails buffer depends on the number of consumer tasks
    uint32_t consumer_number;
} bager_buffer_t;

// Array of buffer instances
extern bager_buffer_t bager_buffers[BUFFER_COUNT];

// Interface
void init_bbuffers();
void push_data(bager_buffer_id_t id, void *value);
bool pop_data(bager_buffer_id_t id, uint32_t consumer_id, void *out_value);

#endif
