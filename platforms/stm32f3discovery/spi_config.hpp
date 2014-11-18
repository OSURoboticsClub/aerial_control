#ifndef SPI_CONFIG_HPP_
#define SPI_CONFIG_HPP_

// L3GD20 SPI configuration
const SPIConfig l3gd20_spi_config = {
  NULL,
  GPIOE,
  GPIOE_SPI1_CS,
  SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA,
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

void spiPlatformInit(void);

#endif // SPI_CONFIG_HPP_
