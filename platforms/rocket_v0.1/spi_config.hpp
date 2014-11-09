#ifndef SPI_CONFIG_HPP_
#define SPI_CONFIG_HPP_

// Mutex to lock output buffer
static Mutex spi_mtx;

// MPU6000 SPI configuration
const SPIConfig mpu6000_spicfg = {
  NULL,
  GPIOC,
  14,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

void spiPlatformInit(void);

#endif // SPI_CONFIG_HPP_
