#ifndef SPI_CONFIG_HPP_
#define SPI_CONFIG_HPP_

// SPI configuration
const SPIConfig spi1_config = {
  NULL,
  GPIOB,
  2,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

void spiPlatformInit(void);

#endif // SPI_CONFIG_HPP_
