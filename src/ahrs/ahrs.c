#include "ahrs/ahrs.h"
#include "ahrs/lsm6dsox_dma.h"

#define GYRO_MISO 16
#define GYRO_MOSI 19
#define GYRO_CLK 18
#define GYRO_CS 17
#define GYRO_INT1 20 // TODO: placeholder, set to the GPIO wired to INT1

#define GYRO_SPI_BAUD (10 * 1000 * 1000)

int init_ahrs() {
  // configures the gyro over blocking SPI, then starts the DMA data stream
  return lsm6dsox_dma_init(spi0, GYRO_MISO, GYRO_MOSI, GYRO_CLK, GYRO_CS,
                           GYRO_INT1, GYRO_SPI_BAUD);
}
