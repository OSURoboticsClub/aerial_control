#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <hal.h>

// Platform config
#include <i2c_config.hpp>
#include <pwm_config.hpp>
#include <spi_config.hpp>
#include <usart_config.hpp>

// Unit config
#include <unit_config.hpp>

const float DT = 0.001;

// L3GD20 SPI configuration
const SPIConfig l3gd20_spi_config = {
  NULL,
  GPIOE,
  GPIOE_SPI1_CS,
  SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA,
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

// LSM303DHLC I2C configuration
const I2CConfig lsm303dlhc_i2c_config = {
  0x00902025, // voodoo magic
  0,
  0
};

#endif
