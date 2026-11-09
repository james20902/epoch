// every data source should have an array of contributors structs contributing to the overall data struct
// each of these contributor structs should contain A. what kind of data they're sending and B. watchdog configurations.
// C. functions to call to try and kick the source back on

#include "data/data.h"
#include <assert.h>
#include <string.h>

void data_ring_init(data_ring *ring, uint8_t *buf, size_t size) {
    assert(size && !(size & (size - 1)));

    ring->buf = buf;
    ring->size = size;
    ring->head = 0;
    ring->count = 0;
    critical_section_init(&ring->lock);
}

bool data_ring_write(data_ring *ring, const void *data, size_t len) {
    const uint8_t *src = data;
    bool ok = false;

    critical_section_enter_blocking(&ring->lock);
    if (len <= ring->size - ring->count) {
        // split the copy where it wraps past the end of buf
        size_t first = ring->size - ring->head;
        if (first > len) {
            first = len;
        }
        memcpy(&ring->buf[ring->head], src, first);
        memcpy(ring->buf, src + first, len - first);

        ring->head = (ring->head + len) & (ring->size - 1);
        ring->count += len;
        ok = true;
    }
    critical_section_exit(&ring->lock);

    return ok;
}

size_t data_ring_read(data_ring *ring, void *out, size_t max) {
    uint8_t *dst = out;

    critical_section_enter_blocking(&ring->lock);
    size_t len = ring->count < max ? ring->count : max;
    size_t tail = (ring->head - ring->count) & (ring->size - 1);

    size_t first = ring->size - tail;
    if (first > len) {
        first = len;
    }
    memcpy(dst, &ring->buf[tail], first);
    memcpy(dst + first, ring->buf, len - first);

    ring->count -= len;
    critical_section_exit(&ring->lock);

    return len;
}

size_t data_ring_count(data_ring *ring) { return ring->count; }

void data_ring_reset(data_ring *ring) {
    critical_section_enter_blocking(&ring->lock);
    ring->head = 0;
    ring->count = 0;
    critical_section_exit(&ring->lock);
}
