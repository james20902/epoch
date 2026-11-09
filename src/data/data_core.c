#include "data/data_core.h"
#include "data/usb_sink.h"
#include <pico/time.h>
#include <pico/types.h>

static struct data_sink *const sinks[] = {
    &usb_sink,
};

#define NUM_SINKS (sizeof(sinks) / sizeof(sinks[0]))

static volatile data_core_state state = DATA_CORE_INITIALIZE;

static absolute_time_t data_start;
static volatile struct payload p = {0};

data_core_state data_core_state_get(void) { return state; }


void data_core_main(void) {
    // data handling

    // assuming all sensors are hooked up to DMA/interrupts, the job of this core is simply to package and timestamp said data

    // data will be timestamped according to this core clock, which means there will be some desync between time of data acquisition
    // vs core timestamp, I'm hoping the delay
    while (1) {
        // sinks get serviced in every state, USB has to keep answering the host to stay enumerated
        for (unsigned i = 0; i < NUM_SINKS; i++) {
            sinks[i]->sink_task(sinks[i]);
        }

        switch (state) {
            case DATA_CORE_INITIALIZE:
                // start the clock
                data_start = get_absolute_time();
                // a sink that fails init sits in DATA_ERROR, the rest carry on without it
                for (unsigned i = 0; i < NUM_SINKS; i++) {
                    sinks[i]->sink_init(sinks[i]);
                }

                // TODO: bring up sources here (init_ahrs) once they're wrapped as data_src

                state = DATA_CORE_WAITING_SOURCES;
            break;
            case DATA_CORE_WAITING_SOURCES:
            // waiting for sources to self test and calibrate
                state = DATA_CORE_STREAMING;
            break;
            case DATA_CORE_STREAMING:
                // read sources, timestamp and package into the data struct, then sink_push to every sink
                // update timestamp
                p.time_us = absolute_time_diff_us(data_start, get_absolute_time());

                for (unsigned i = 0; i < NUM_SINKS; i++) {
                    sinks[i]->sink_push(sinks[i], (void*)&p, PAYLOAD_SIZE);
                }
            break;
            default:
            break;
        }
    }
}
