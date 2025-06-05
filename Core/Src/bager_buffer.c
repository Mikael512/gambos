#include "bager_buffer.h"
#include <string.h>


bager_buffer_t bager_buffers[BUFFER_COUNT];

void init_buffers() {
    for (uint32_t i = 0; i < BUFFER_COUNT; i++) {
        bager_buffers[i].buffer.head  = 0;
        for (uint32_t j = 0; j < MAX_CONSUMERS; j++) {
            bager_buffers[i].tails[j] = 0;
        }
    }
}

void push_data(bager_buffer_id id, uint32_t *value) {
    uint32_t *head = &bager_buffers[id].buffer.head;
    bager_buffers[id].buffer.data[*head] = *value;
    *head = (*head + 1) % BUFFER_SIZE;

    uint32_t *tail;
    for (uint32_t i = 0; i < MAX_CONSUMERS; ++i) {
        tail = &bager_buffers[id].tails[i];
        if (*head == *tail) {
            *tail = (*tail + 1) % MAX_CONSUMERS;
        }
    }
}

bool pop_data(bager_buffer_id id, uint32_t consumer_id, uint32_t *out_value) {
    uint32_t *tail = &bager_buffers[id].tails[consumer_id];
    uint32_t *head = &bager_buffers[id].buffer.head;
    *out_value = bager_buffers[id].buffer.data[*tail];

    if ((*tail + 1) % MAX_CONSUMERS != *head) {
        *tail = (*tail + 1) % MAX_CONSUMERS;
        return true;
    } else {
        return false;
    }
}