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

// raw samples, in device LSB (convert with lsm6dsox_from_fs*_to_*)
typedef struct {
  int16_t temp;
  int16_t gyro[3];
  int16_t accel[3];
} lsm6dsox_sample_t;

int lsm6dsox_dma_init(spi_inst_t *spi_inst, uint miso, uint mosi, uint clk,
                      uint cs, uint int1, uint baud);

// pops the oldest sample, returns false if the buffer is empty
bool lsm6dsox_dma_read(lsm6dsox_sample_t *out);

uint lsm6dsox_dma_available(void);

// enable/disable the data-ready stream; blocking register access is only
// allowed while the stream is disabled
void lsm6dsox_dma_stream_enable(bool en);

// samples dropped because the ring buffer was full
uint32_t lsm6dsox_dma_overruns(void);

// data-ready edges ignored because a transfer was still in flight
uint32_t lsm6dsox_dma_missed_drdy(void);

#endif
