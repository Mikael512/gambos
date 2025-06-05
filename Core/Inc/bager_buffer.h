#ifndef BAGER_BUFFER_H
#define BAGER_BUFFER_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

#define MAX_CONSUMERS 1
#define BUFFER_SIZE 8

typedef enum {
    BUFFER_ACCEL = 0,
    BUFFER_GYRO,
    BUFFER_MAG,
    BUFFER_COUNT
} bager_buffer_id;

typedef struct {
    uint32_t head;
    uint32_t data[BUFFER_SIZE];
} circular_buffer_t;

typedef struct {
    circular_buffer_t buffer;
    uint32_t tails[MAX_CONSUMERS];
} bager_buffer_t;

extern bager_buffer_t bager_buffers[BUFFER_COUNT];

void init_buffers();
void push_data(bager_buffer_id id, uint32_t *value);
bool pop_data(bager_buffer_id id, uint32_t consumer_id, uint32_t* out_value);

#endif
