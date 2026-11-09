#ifndef LSM6DSOX_DMA_H
#define LSM6DSOX_DMA_H

#include <hardware/spi.h>
#include <pico/types.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * Singleton LSM6DSOX driver:
 *  - register configuration is blocking (polled SPI with timeouts); it
 *    overrides the __weak lsm6dsox_read_reg/lsm6dsox_write_reg so every ST
 *    helper in lsm6dsox_reg.c goes through it
 *  - sensor data is streamed: INT1 data-ready -> fixed 15 byte SPI DMA burst
 *    (OUT_TEMP_L..OUTZ_H_A) -> DMA IRQ pushes into a ring buffer
 */

// return codes
#define LSM6DSOX_DMA_OK 0
#define LSM6DSOX_DMA_ERR_TIMEOUT -1   // SPI FIFO never became ready
#define LSM6DSOX_DMA_ERR_BUSY -2      // blocking access while streaming
#define LSM6DSOX_DMA_ERR_WHOAMI -3    // wrong / no device on the bus
#define LSM6DSOX_DMA_ERR_RESET -4     // software reset never completed
#define LSM6DSOX_DMA_ERR_VERIFY -5    // config register read-back mismatch

/* Self test limits. */
#define MIN_ST_LIMIT_mg 50.0f
#define MAX_ST_LIMIT_mg 1700.0f
#define MIN_ST_LIMIT_mdps 150000.0f
#define MAX_ST_LIMIT_mdps 700000.0f

/* Self test results. */
#define ST_PASS 1U
#define ST_FAIL 0U

// raw samples, in device LSB (convert with lsm6dsox_from_fs*_to_*)
typedef struct {
  int16_t temp;
  int16_t gyro[3];
  int16_t accel[3];
} lsm6dsox_sample_t;

int lsm6dsox_init(spi_inst_t *spi_inst, uint miso, uint mosi, uint clk,
                      uint cs, uint int1, uint baud);

#endif
