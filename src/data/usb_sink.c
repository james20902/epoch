#include "data/usb_sink.h"
#include "tusb.h"

// TODO, payload is a known size, buffer len should be a multiple of the payload size...
#define RING_SIZE 4096 // must be a power of two
#define DRAIN_CHUNK 64 // one full speed bulk packet

static uint8_t ring_buf[RING_SIZE];

static int usb_init(struct data_sink *sink) {
    const tusb_rhport_init_t rh_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO,
    };

    data_ring_init(&sink->ring, ring_buf, sizeof(ring_buf));
    sink->dropped = 0;

    // also enables the USB IRQ on the calling core
    if (!tusb_init(USB_SINK_RHPORT, &rh_init)) {
        sink->status = DATA_ERROR;
        return -1;
    }

    sink->status = DATA_READY;
    return 0;
}

static void usb_teardown(struct data_sink *sink) {
    // drop the pullup first so the host sees an unplug
    tud_disconnect();
    tud_deinit(USB_SINK_RHPORT);
    data_ring_reset(&sink->ring);
    sink->status = DATA_UNINITIALIZED;
}

static bool usb_push(struct data_sink *sink, const void *data, size_t len) {
    // nothing is queued while the port is closed so the host never opens onto stale data
    if (sink->status != DATA_ACTIVE || !data_ring_write(&sink->ring, data, len)) {
        sink->dropped++;
        return false;
    }
    return true;
}

static void usb_task(struct data_sink *sink) {
    if (sink->status == DATA_UNINITIALIZED || sink->status == DATA_ERROR) {
        return;
    }

    tud_task();

    // connected means the host has the port open (DTR set)
    if (!tud_cdc_connected()) {
        if (sink->status == DATA_ACTIVE) {
            data_ring_reset(&sink->ring);
        }
        sink->status = DATA_READY;
        return;
    }
    sink->status = DATA_ACTIVE;

    uint8_t chunk[DRAIN_CHUNK];
    uint32_t space;

    while ((space = tud_cdc_write_available()) > 0) {
        size_t n = data_ring_read(&sink->ring, chunk, space < sizeof(chunk) ? space : sizeof(chunk));
        if (!n) {
            break;
        }
        tud_cdc_write(chunk, n);
    }
    tud_cdc_write_flush();
}

struct data_sink usb_sink = {
    .sink_init = usb_init,
    .sink_teardown = usb_teardown,
    .sink_push = usb_push,
    .sink_task = usb_task,
    .status = DATA_UNINITIALIZED,
};
