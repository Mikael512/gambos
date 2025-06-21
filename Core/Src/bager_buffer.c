#include "bager_buffer.h"
#include <string.h>

// === INSTRUCTIONS TO ADD NEW BAGER BUFFERS ===
// 1. In the bager_buffer.h header file add the new enumeration (bager_buffer_id_t)
// 2. In this file, using the macro function DEFINE_BUFFER add the new buffer
// 3. Check what type of data you want to use
// 4. Make sure the number of consumers for each match the defined one
// 5. Each consumer should have a unique ID
// 6. Make sure to add the buffer in the init_bbuffers function


// === MACRO TO DEFINE EACH BUFFER ===
#define DEFINE_BUFFER(id, type, consumer_num, size)                             \
    /* Define the circular buffer storage array */                              \
    type id##_storage[size];                                                    \
                                                                                \
    /* Define the array that holds read positions for each consumer */          \
    uint32_t id##_consumer_tails[consumer_num];                                 \
                                                                                \
    /* Define the buffer structure */                                           \
    bager_buffer_t id##_buffer = {                                              \
        .buffer = {                                                             \
            .head = 0,                                                          \
            .element_size = sizeof(type),                                       \
            .data = (uint8_t *)id##_storage,                                    \
            .buffer_size = size                                                 \
        },                                                                      \
        .consumer_tails = (uint32_t *) id##_consumer_tails,                     \
        .consumer_number = consumer_num                                         \
    }

// Define all buffer instances 
DEFINE_BUFFER(acc, int16_3d_t, 1, 8);
DEFINE_BUFFER(gyro, int16_3d_t, 1, 8);
DEFINE_BUFFER(mag, int16_3d_t, 1, 8);

// Array of buffers used by application
bager_buffer_t bager_buffers[BUFFER_COUNT];

void init_bbuffers() {
    bager_buffers[BUFFER_ACC] = acc_buffer;
    bager_buffers[BUFFER_GYRO] = gyro_buffer;
    bager_buffers[BUFFER_MAG] = mag_buffer;
}

void push_data(bager_buffer_id_t id, void *value) {
    bager_buffer_t *buf = &bager_buffers[id];
    circular_buffer_t *cb = &buf->buffer;

    uint32_t index = cb->head % bager_buffers[id].buffer.buffer_size;
    memcpy(cb->data + (index * cb->element_size), value, cb->element_size);
    cb->head++;
}

bool pop_data(bager_buffer_id_t id, uint32_t consumer_id, void *out_value) {
    if (consumer_id >= bager_buffers[id].consumer_number) return false;

    bager_buffer_t *buf = &bager_buffers[id];
    circular_buffer_t *cb = &buf->buffer;
    uint32_t *tail = &buf->consumer_tails[consumer_id];

    if (*tail >= cb->head) return false;

    uint32_t index = (*tail % bager_buffers[id].buffer.buffer_size);
    memcpy(out_value, cb->data + (index * cb->element_size), cb->element_size);
    (*tail)++;
    return true;
}
