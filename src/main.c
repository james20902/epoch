#include "main.h"
#include "data/data_core.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"

#define SYS_CLOCK_KHZ 200000

int main()
{
    system_state state = INITIALIZE;

    while (1) {
        switch (state) {
            case INITIALIZE: 
                set_sys_clock_khz(SYS_CLOCK_KHZ, true);

                multicore_reset_core1();
                multicore_launch_core1(data_core_main);
                state = WAITING_DATA;
            break;
            case WAITING_DATA:
            // waiting for sensors to self test and calibrate
            break;
            case WARMUP:
            // wait for ring buffers/cache to populate
            // https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_xip_cache
            break;
            case GROUND:
            // consume data from some data source (this data should be a fixed predefined struct)
            // different data sources can be: live data, streamed simulated, file

            // "process data", for now we just want to print
            break;
            default:
            break;
        }
    }
}
