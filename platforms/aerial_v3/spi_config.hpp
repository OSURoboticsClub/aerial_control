#ifndef SPI_CONFIG_HPP_
#define SPI_CONFIG_HPP_

// Mutex to lock output buffer
static Mutex spi_mtx;

// MPU6000 SPI configuration
const SPIConfig mpu6000_spicfg = {
  NULL,
  GPIOB,
  2,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

void spiPlatformInit(void);
void _spiExchange(SPIDriver *spid, const SPIConfig *spicfg, uint16_t bufsize, uint8_t *txbuf, uint8_t *rxbuf);

#endif // SPI_CONFIG_HPP_
