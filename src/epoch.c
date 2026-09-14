#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

#define LED_PIN 25
#define SERVO_1_GPIO 0
#define SERVO_2_GPIO 1

#define SYS_CLOCK_KHZ 200000

// 200MHz / 250 = 800kHz counter tick (1.25us)
#define PWM_CLKDIV 250.0f
// 16000 ticks * 1.25us = 20ms frame (50Hz)
#define PWM_WRAP 15999

#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000

static inline uint16_t servo_us_to_level(uint16_t us) {
    // clamp to the servo's mechanical range, then convert to a counter level.
    if (us < SERVO_MIN_US) {
        us = SERVO_MIN_US;
    }
    if (us > SERVO_MAX_US) {
        us = SERVO_MAX_US;
    }
    // 1000us needs to return 800
    // 2000us needs to return 1600
    return (uint16_t)(us * 0.8f);
}

static void servo_set_us(uint gpio, uint16_t us) {
    // this call is double buffered, the updated comparison value
    // will not take affect until a wrap has occurred
    pwm_set_gpio_level(gpio, servo_us_to_level(us));
}

int main()
{
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // set GPIO to PWM
    gpio_set_function(SERVO_1_GPIO, GPIO_FUNC_PWM);
    gpio_set_function(SERVO_2_GPIO, GPIO_FUNC_PWM);

    // 50hz frame; a 1-2ms pulse sits inside it
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, PWM_CLKDIV);
    pwm_config_set_wrap(&cfg, PWM_WRAP);

    // GPIO 0 and 1 are channels A and B of one slice, so a single init covers both
    pwm_init(pwm_gpio_to_slice_num(SERVO_1_GPIO), &cfg, true);

    while (1) {
        servo_set_us(SERVO_1_GPIO, 1000);
        servo_set_us(SERVO_2_GPIO, 1000);
        gpio_put(LED_PIN, 0);
        sleep_ms(1000);
        servo_set_us(SERVO_1_GPIO, 1333);
        servo_set_us(SERVO_2_GPIO, 1333);
        gpio_put(LED_PIN, 1);
        sleep_ms(1000);
    }
}
