#ifndef DATA_CORE_H
#define DATA_CORE_H

/*
 * Core1 state machine: owns every data source and sink. Sources are brought
 * up and read here, and sinks are initialized and serviced here, so their
 * IRQs and stacks all live on core1.
 */

#include <stdint.h>

typedef enum {
    DATA_CORE_UNDEFINED,
    DATA_CORE_INITIALIZE,      // bring up sinks, then sources
    DATA_CORE_WAITING_SOURCES, // sources self test / calibrate
    DATA_CORE_STREAMING        // read sources, timestamp + package, push to sinks
} data_core_state;

struct payload {
    int64_t time_us;
};

#define PAYLOAD_SIZE sizeof(struct payload)

// core1 entry, never returns
void data_core_main(void);

// for core0 to follow the data core's progress
data_core_state data_core_state_get(void);

#endif
