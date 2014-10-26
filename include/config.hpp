#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <hal.h>

// TODO: Put this in the Makefile
#include <configs/osuar_apollo.h>

static const float DT = 0.001;

// L3GD20 SPI configuration
static const SPIConfig l3gd20_spi_config = {
  NULL,
  GPIOE,
  GPIOE_SPI1_CS,
  SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA,
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

// LSM303DHLC I2C configuration
static const I2CConfig lsm303dlhc_i2c_config = {
  0x00902025, // voodoo magic
  0,
  0
};

// USART1 configuration
static const SerialConfig usart1_config = {
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

#endif
