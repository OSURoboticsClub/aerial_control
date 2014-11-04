#ifndef SPI_CONFIG_HPP_
#define SPI_CONFIG_HPP_

// L3GD20 SPI configuration
// TODO(yoos): This is actually MPU-6000 config from last year.
const SPIConfig l3gd20_spi_config = {
  NULL,
  GPIOB,
  2,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

#endif // SPI_CONFIG_HPP_
