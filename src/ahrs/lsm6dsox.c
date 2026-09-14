#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ahrs/lsm6dsox.h"

static

static int lsm6dsoxinit(struct *lsm6dsox) {
    if (!lsm6dsox) {
        return -1;
    }

    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(I2C_INSTANCE(1), 100 * 1000);
    gpio_set_function(lsm6dsox->gpio_clock, GPIO_FUNC_I2C);
    gpio_set_function(lsm6dsox->gpio_data, GPIO_FUNC_I2C);
    gpio_pull_up(lsm6dsox->gpio_clock);
    gpio_pull_up(lsm6dsox->gpio_data);
 

 
}