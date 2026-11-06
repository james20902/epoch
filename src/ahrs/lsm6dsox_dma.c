#include "ahrs/lsm6dsox_dma.h"
#include "ahrs/lsm6dsox_reg.h"
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <pico/sync.h>
#include <pico/time.h>

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
static uint int1_pin;

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

/*
 * Blocking register access. These replace the __weak versions in
 * lsm6dsox_reg.c, so every ST _set/_get helper uses them.
 */

static int32_t xfer_blocking(uint8_t reg, const uint8_t *tx, uint8_t *rx,
                             uint16_t len) {
  if (stream_active) {
    return LSM6DSOX_DMA_ERR_BUSY;
  }

  spi_hw_t *hw = spi_get_hw(spi);
  absolute_time_t deadline = make_timeout_time_us(XFER_TIMEOUT_US);
  int32_t ret = LSM6DSOX_DMA_OK;

  // drop anything stale in the rx fifo
  while (spi_is_readable(spi)) {
    (void)hw->dr;
  }

  cs_select();

  // lockstep one byte out, one byte in so the rx fifo can never overflow
  for (uint32_t i = 0; i <= len; i++) {
    uint8_t out = (i == 0) ? reg : (tx ? tx[i - 1] : 0);

    while (!spi_is_writable(spi)) {
      if (time_reached(deadline)) {
        ret = LSM6DSOX_DMA_ERR_TIMEOUT;
        goto done;
      }
    }
    hw->dr = out;

    while (!spi_is_readable(spi)) {
      if (time_reached(deadline)) {
        ret = LSM6DSOX_DMA_ERR_TIMEOUT;
        goto done;
      }
    }
    uint8_t in = (uint8_t)hw->dr;

    if (i > 0 && rx) {
      rx[i - 1] = in;
    }
  }

done:
  cs_deselect();
  return ret;
}

int32_t lsm6dsox_read_reg(uint8_t reg, uint8_t *data, uint16_t len) {
  return xfer_blocking(reg | 0x80, NULL, data, len);
}

int32_t lsm6dsox_write_reg(uint8_t reg, const uint8_t *data, uint16_t len) {
  return xfer_blocking(reg & 0x7F, data, NULL, len);
}

/*
 * Streaming path
 */

static inline int16_t le16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void int1_isr(void) {
  if (!(gpio_get_irq_event_mask(int1_pin) & GPIO_IRQ_EDGE_RISE)) {
    return;
  }
  gpio_acknowledge_irq(int1_pin, GPIO_IRQ_EDGE_RISE);

  if (busy) {
    missed_drdy++;
    return;
  }
  busy = true;

  cs_select();

  dma_channel_set_read_addr(dma_tx, tx_buf, false);
  dma_channel_set_trans_count(dma_tx, BURST_LEN, false);
  dma_channel_set_write_addr(dma_rx, rx_buf, false);
  dma_channel_set_trans_count(dma_rx, BURST_LEN, false);

  // start both together so rx is draining as soon as tx starts clocking
  dma_start_channel_mask((1u << dma_tx) | (1u << dma_rx));
}

static void dma_rx_isr(void) {
  if (!dma_channel_get_irq0_status(dma_rx)) {
    return;
  }
  dma_channel_acknowledge_irq0(dma_rx);

  // rx completing means the last byte has been fully shifted in
  cs_deselect();

  const uint8_t *d = &rx_buf[1];

  critical_section_enter_blocking(&ring_lock);

  lsm6dsox_sample_t *s = &ring[ring_head];
  s->temp = le16(&d[0]);
  for (int i = 0; i < 3; i++) {
    s->gyro[i] = le16(&d[2 + 2 * i]);
    s->accel[i] = le16(&d[8 + 2 * i]);
  }
  ring_head = (ring_head + 1) & RING_MASK;

  // full: newest overwrites oldest
  if (ring_count == RING_SIZE) {
    overruns++;
  } else {
    ring_count++;
  }

  critical_section_exit(&ring_lock);

  busy = false;
}

static void setup_dma(void) {
  io_rw_32 *spi_dr = &spi_get_hw(spi)->dr;

  dma_tx = dma_claim_unused_channel(true);
  dma_channel_config c = dma_channel_get_default_config(dma_tx);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_dreq(&c, spi_get_dreq(spi, true));
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  dma_channel_configure(dma_tx, &c, spi_dr, tx_buf, BURST_LEN, false);

  dma_rx = dma_claim_unused_channel(true);
  c = dma_channel_get_default_config(dma_rx);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_dreq(&c, spi_get_dreq(spi, false));
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  dma_channel_configure(dma_rx, &c, rx_buf, spi_dr, BURST_LEN, false);

  dma_channel_set_irq0_enabled(dma_rx, true);
  irq_add_shared_handler(DMA_IRQ_0, dma_rx_isr,
                         PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  irq_set_enabled(DMA_IRQ_0, true);
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

int lsm6dsox_dma_init(spi_inst_t *spi_inst, uint miso, uint mosi, uint clk,
                      uint cs, uint int1, uint baud) {
  int ret;

  spi = spi_inst;
  cs_pin = cs;
  int1_pin = int1;

  critical_section_init(&ring_lock);

  // CS idles high before the bus comes up
  gpio_init(cs_pin);
  gpio_put(cs_pin, 1);
  gpio_set_dir(cs_pin, GPIO_OUT);

  gpio_init(int1_pin);
  gpio_set_dir(int1_pin, GPIO_IN);
  gpio_pull_down(int1_pin);

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

  setup_dma();

  gpio_add_raw_irq_handler(int1_pin, int1_isr);
  irq_set_enabled(IO_IRQ_BANK0, true);

  lsm6dsox_dma_stream_enable(true);

  return LSM6DSOX_DMA_OK;
}

void lsm6dsox_dma_stream_enable(bool en) {
  if (en) {
    stream_active = true;
    gpio_acknowledge_irq(int1_pin, GPIO_IRQ_EDGE_RISE);
    gpio_set_irq_enabled(int1_pin, GPIO_IRQ_EDGE_RISE, true);
  } else {
    gpio_set_irq_enabled(int1_pin, GPIO_IRQ_EDGE_RISE, false);
    // let an in-flight burst finish before handing the bus back
    while (busy) {
      tight_loop_contents();
    }
    stream_active = false;
  }
}

bool lsm6dsox_dma_read(lsm6dsox_sample_t *out) {
  bool ok = false;

  critical_section_enter_blocking(&ring_lock);
  if (ring_count) {
    uint tail = (ring_head - ring_count) & RING_MASK;
    *out = ring[tail];
    ring_count--;
    ok = true;
  }
  critical_section_exit(&ring_lock);

  return ok;
}

uint lsm6dsox_dma_available(void) { return ring_count; }

uint32_t lsm6dsox_dma_overruns(void) { return overruns; }

uint32_t lsm6dsox_dma_missed_drdy(void) { return missed_drdy; }
