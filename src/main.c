#include "main.h"
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
                
                state = WAITING_DATA;
            break;
            default:
            break;
        }
    }
}
