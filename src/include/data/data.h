#ifndef DATA_H
#define DATA_H

#include <pico/sync.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    DATA_UNINITIALIZED,
    DATA_READY,  // initialized, nobody on the other end yet (no host, port closed)
    DATA_ACTIVE, // connected and moving data
    DATA_ERROR
} data_status;

struct data_src { // things like sensors
    void (*src_init)(void);
    void (*src_teardown)(void);
    data_status status;
};

// byte ring between producers and a sink's drain, single consumer
typedef struct {
    uint8_t *buf;
    size_t size; // must be a power of two
    size_t head, count;
    critical_section_t lock;
} data_ring;

struct data_sink { // things like streaming
    int (*sink_init)(struct data_sink *sink);
    void (*sink_teardown)(struct data_sink *sink);
    // non-blocking, safe from either core. all or nothing so packets never get split
    bool (*sink_push)(struct data_sink *sink, const void *data, size_t len);
    // call every loop iteration: services the transport and drains the ring into it
    void (*sink_task)(struct data_sink *sink);

    data_ring ring;
    uint32_t dropped;            // pushes rejected because the ring was full or the sink wasn't active
    volatile data_status status; // written by sink_task, read by producers
};

void data_ring_init(data_ring *ring, uint8_t *buf, size_t size);

// copies all of data in or nothing, returns false if there isn't room
bool data_ring_write(data_ring *ring, const void *data, size_t len);

// pops up to max bytes, returns how many were copied out
size_t data_ring_read(data_ring *ring, void *out, size_t max);

size_t data_ring_count(data_ring *ring);

void data_ring_reset(data_ring *ring);

#endif
