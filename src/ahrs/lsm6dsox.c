#include "ahrs/lsm6dsox.h"
#include "ahrs/lsm6dsox_reg.h"
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <pico/sync.h>
#include <pico/time.h>
#include <stdio.h>
#include <string.h>

#define BOOT_TIME_MS 10
#define RESET_TIMEOUT_US 50000
#define XFER_TIMEOUT_US 1000 // per blocking register transaction

// burst covers OUT_TEMP_L..OUTZ_H_A: temp, gyro xyz, accel xyz
#define BURST_DATA_LEN (LSM6DSOX_OUTZ_H_A - LSM6DSOX_OUT_TEMP_L + 1)
#define BURST_LEN (1 + BURST_DATA_LEN) // address byte + data

#define RING_SIZE 8 // must be a power of two
#define RING_MASK (RING_SIZE - 1)

static spi_inst_t *spi;
static uint cs_pin;

static int dma_tx, dma_rx;

// fixed burst: read flag | start address, rest are dummy bytes to clock MISO
static const uint8_t tx_buf[BURST_LEN] = {0x80 | LSM6DSOX_OUT_TEMP_L};
// rx_buf[0] is clocked in during the address byte and is garbage
static uint8_t rx_buf[BURST_LEN];

static lsm6dsox_sample_t ring[RING_SIZE];
static uint ring_head, ring_count;
static critical_section_t ring_lock;

static volatile bool busy;          // DMA burst in flight
static volatile bool stream_active; // INT1 stream enabled
static volatile uint32_t overruns;
static volatile uint32_t missed_drdy;

static inline void cs_select(void) { gpio_put(cs_pin, 0); }

static inline void cs_deselect(void) { gpio_put(cs_pin, 1); }

void isr_handle() {

}

/*
 * Configuration
 */

static int configure_device(void) {
  int32_t ret;
  uint8_t whoami;
  uint8_t rst;

  /*
   * reset before anything else: the chip keeps its registers across an MCU
   * reset, and if it was left in 3-wire mode SDO never drives so WHO_AM_I
   * would read garbage. SW_RESET is a write, which works in either mode.
   */
  lsm6dsox_ctrl3_c_t ctrl3 = {.if_inc = 1, .sw_reset = 1};
  ret = lsm6dsox_write_reg(LSM6DSOX_CTRL3_C, (uint8_t *)&ctrl3, 1);
  if (ret) {
    return ret;
  }

  absolute_time_t deadline = make_timeout_time_us(RESET_TIMEOUT_US);
  do {
    if (time_reached(deadline)) {
      return LSM6DSOX_DMA_ERR_RESET;
    }
    ret = lsm6dsox_reset_get(&rst);
    if (ret) {
      return ret;
    }
  } while (rst);

  ret = lsm6dsox_device_id_get(&whoami);
  if (ret) {
    return ret;
  }
  if (whoami != LSM6DSOX_ID) {
    return LSM6DSOX_DMA_ERR_WHOAMI;
  }

  // interface
  ret = lsm6dsox_i3c_disable_set(LSM6DSOX_I3C_DISABLE);
  ret = ret ? ret : lsm6dsox_i2c_interface_set(LSM6DSOX_I2C_DISABLE);
  ret = ret ? ret : lsm6dsox_spi_mode_set(LSM6DSOX_SPI_4_WIRE);
  ret = ret ? ret : lsm6dsox_auto_increment_set(PROPERTY_ENABLE);
  ret = ret ? ret : lsm6dsox_block_data_update_set(PROPERTY_ENABLE);

  // sensors
  ret = ret ? ret : lsm6dsox_xl_full_scale_set(LSM6DSOX_16g);
  ret = ret ? ret : lsm6dsox_gy_full_scale_set(LSM6DSOX_2000dps);
  ret = ret ? ret : lsm6dsox_xl_data_rate_set(LSM6DSOX_XL_ODR_208Hz);
  ret = ret ? ret : lsm6dsox_gy_data_rate_set(LSM6DSOX_GY_ODR_208Hz);

  // gyro data-ready on INT1, pulsed so every sample gives a fresh edge
  lsm6dsox_pin_int1_route_t route = {0};
  route.drdy_g = 1;
  ret = ret ? ret : lsm6dsox_data_ready_mode_set(LSM6DSOX_DRDY_PULSED);
  ret = ret ? ret : lsm6dsox_pin_int1_route_set(route);
  if (ret) {
    return ret;
  }

  // read back what the stream depends on
  uint8_t ctrl[3]; // CTRL1_XL, CTRL2_G, CTRL3_C
  lsm6dsox_int1_ctrl_t int1_ctrl;
  ret = lsm6dsox_read_reg(LSM6DSOX_CTRL1_XL, ctrl, sizeof(ctrl));
  ret = ret ? ret
            : lsm6dsox_read_reg(LSM6DSOX_INT1_CTRL, (uint8_t *)&int1_ctrl, 1);
  if (ret) {
    return ret;
  }

  lsm6dsox_ctrl1_xl_t *ctrl1_xl = (lsm6dsox_ctrl1_xl_t *)&ctrl[0];
  lsm6dsox_ctrl2_g_t *ctrl2_g = (lsm6dsox_ctrl2_g_t *)&ctrl[1];
  lsm6dsox_ctrl3_c_t *ctrl3_c = (lsm6dsox_ctrl3_c_t *)&ctrl[2];

  if (ctrl1_xl->odr_xl != (LSM6DSOX_XL_ODR_208Hz & 0x0F) ||
      ctrl1_xl->fs_xl != (LSM6DSOX_16g & 0x03) ||
      ctrl2_g->odr_g != (LSM6DSOX_GY_ODR_208Hz & 0x0F) ||
      ctrl2_g->fs_g != (LSM6DSOX_2000dps & 0x07) || !ctrl3_c->bdu ||
      !ctrl3_c->if_inc || ctrl3_c->sim || !int1_ctrl.int1_drdy_g) {
    return LSM6DSOX_DMA_ERR_VERIFY;
  }

  return LSM6DSOX_DMA_OK;
}

int lsm6dsox_init(spi_inst_t *spi_inst, uint miso, uint mosi, uint clk,
                      uint cs, uint int1, uint baud) {
  int ret;

  spi = spi_inst;
  cs_pin = cs;

  critical_section_init(&ring_lock);

  // CS idles high before the bus comes up
  gpio_init(cs_pin);
  gpio_put(cs_pin, 1);
  gpio_set_dir(cs_pin, GPIO_OUT);

  gpio_init(int1);
  gpio_set_dir(int1, GPIO_IN);
  gpio_pull_down(int1);

  // LSM6DSOX is SPI mode 3: clock idles high, sample on rising edge
  spi_init(spi, baud);
  spi_set_format(spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_set_function(miso, GPIO_FUNC_SPI);
  gpio_set_function(mosi, GPIO_FUNC_SPI);
  gpio_set_function(clk, GPIO_FUNC_SPI);

  sleep_ms(BOOT_TIME_MS);

  ret = configure_device();
  if (ret) {
    return ret;
  }

  gpio_add_raw_irq_handler(int1, isr_handle);
  irq_set_enabled(IO_IRQ_BANK0, true);


  return LSM6DSOX_DMA_OK;
}

void lsm6dsox_self_test(void) {
  uint8_t tx_buffer[1000];
  float_t val_st_off[3];
  int16_t data_raw[3];
  float_t val_st_on[3];
  float_t test_val[3];
  uint8_t st_result;
  uint8_t whoamI;
  uint8_t drdy;
  uint8_t rst;
  uint8_t i;
  uint8_t j;

  /* Check device ID */
  lsm6dsox_device_id_get(&whoamI);

  // spin until we have connection
  if (whoamI != LSM6DSOX_ID) {
    while (1)
      ;
  }

  /* Restore default configuration */
  lsm6dsox_reset_set(PROPERTY_ENABLE);

  do {
    lsm6dsox_reset_get(&rst);
  } while (rst);

  /* Configure interface */
  lsm6dsox_i3c_disable_set(LSM6DSOX_I3C_DISABLE);
  lsm6dsox_i2c_interface_set(LSM6DSOX_I2C_DISABLE);
  lsm6dsox_spi_mode_set(LSM6DSOX_SPI_4_WIRE);

  /* Enable Block Data Update */
  lsm6dsox_block_data_update_set(PROPERTY_ENABLE);

  /*
   * Accelerometer Self Test
   */
  /* Set Output Data Rate */
  lsm6dsox_xl_data_rate_set(LSM6DSOX_XL_ODR_52Hz);
  /* Set full scale */
  lsm6dsox_xl_full_scale_set(LSM6DSOX_4g);
  /* Wait stable output */
  sleep_ms(100);

  /* Check if new value available */
  do {
    lsm6dsox_xl_flag_data_ready_get(&drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6dsox_acceleration_raw_get(data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsox_xl_flag_data_ready_get(&drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dsox_acceleration_raw_get(data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsox_from_fs4_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm6dsox_xl_self_test_set(LSM6DSOX_XL_ST_NEGATIVE);
  // lsm6dsox_xl_self_test_set( LSM6DSOX_XL_ST_POSITIVE);
  /* Wait stable output */
  sleep_ms(100);

  /* Check if new value available */
  do {
    lsm6dsox_xl_flag_data_ready_get(&drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6dsox_acceleration_raw_get(data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsox_xl_flag_data_ready_get(&drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dsox_acceleration_raw_get(data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsox_from_fs4_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabsf((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  st_result = ST_PASS;

  for (i = 0; i < 3; i++) {
    if ((MIN_ST_LIMIT_mg > test_val[i]) || (test_val[i] > MAX_ST_LIMIT_mg)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm6dsox_xl_self_test_set(LSM6DSOX_XL_ST_DISABLE);
  /* Disable sensor. */
  lsm6dsox_xl_data_rate_set(LSM6DSOX_XL_ODR_OFF);
  /*
   * Gyroscope Self Test
   */
  /* Set Output Data Rate */
  lsm6dsox_gy_data_rate_set(LSM6DSOX_GY_ODR_208Hz);
  /* Set full scale */
  lsm6dsox_gy_full_scale_set(LSM6DSOX_2000dps);
  /* Wait stable output */
  sleep_ms(100);

  /* Check if new value available */
  do {
    lsm6dsox_gy_flag_data_ready_get(&drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6dsox_angular_rate_raw_get(data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsox_gy_flag_data_ready_get(&drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dsox_angular_rate_raw_get(data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsox_from_fs2000_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm6dsox_gy_self_test_set(LSM6DSOX_GY_ST_POSITIVE);
  // lsm6dsox_gy_self_test_set( LIS2DH12_GY_ST_NEGATIVE);
  /* Wait stable output */
  sleep_ms(100);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsox_gy_flag_data_ready_get(&drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dsox_angular_rate_raw_get(data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsox_from_fs2000_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabsf((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if ((MIN_ST_LIMIT_mdps > test_val[i]) ||
        (test_val[i] > MAX_ST_LIMIT_mdps)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm6dsox_gy_self_test_set(LSM6DSOX_GY_ST_DISABLE);
  /* Disable sensor. */
  lsm6dsox_gy_data_rate_set(LSM6DSOX_GY_ODR_OFF);

  if (st_result == ST_PASS) {
    printf("Self Test - PASS\r\n");
    // snprintf((char *)tx_buffer, sizeof(tx_buffer), "Self Test - FAIL\r\n" );
  }

  else {
    printf("Self Test - FAIL\r\n");
    // snprintf((char *)tx_buffer, sizeof(tx_buffer), "Self Test - FAIL\r\n" );
  }
}