#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <hal.h>

// Drivers
//#include <drivers/l3gd20.hpp>
//#include <drivers/lsm303dlhc.hpp>

// Systems
//#include <system/default_multirotor_vehicle_system.hpp>

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

// System config
// TODO(yoos): Can we make this safer?
//static L3GD20 gyro(&SPID1);
//static LSM303DLHC accel(&I2CD1);
//static DefaultMultirotorVehicleSystem system(&accel, &gyro);
#define GYRO L3GD20
#define ACCEL LSM303DLHC
#define SYSTEM DefaultMultirotorVehicleSystem

#endif
