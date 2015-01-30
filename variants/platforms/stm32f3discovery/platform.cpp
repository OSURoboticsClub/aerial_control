#include "variant/platform.hpp"

#include "hal.h"

#include "drivers/l3gd20.hpp"
#include "drivers/lsm303dlhc.hpp"
#include "sensor/accelerometer.hpp"
#include "sensor/gyroscope.hpp"
#include "variant/i2c_platform.hpp"
#include "variant/pwm_platform.hpp"
#include "variant/spi_platform.hpp"
#include "variant/usart_platform.hpp"

// L3GD20 SPI configuration
static const SPIConfig L3GD20_CONFIG {
  NULL,
  GPIOE,
  GPIOE_SPI1_CS,
  SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA,
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

// LSM303DHLC I2C configuration
static const I2CConfig LSM303_CONFIG {
  0x00902025, // voodoo magic
  0,
  0
};

static const i2caddr_t LSM303_I2C_ACC_ADDRESS = 0x19;

Platform::Platform() {
}

template <>
Gyroscope& Platform::get() {
  static L3GD20 gyro(&SPID1, &L3GD20_CONFIG);
  return gyro;
}

template <>
LSM303DLHC& Platform::get() {
  static LSM303DLHC accel(&I2CD1, &LSM303_CONFIG, LSM303_I2C_ACC_ADDRESS);
  return accel;
}

template <>
Accelerometer& Platform::get() {
  return get<LSM303DLHC>();
}

template <>
Magnetometer& Platform::get() {
  return get<LSM303DLHC>();
}

template <>
I2CPlatform& Platform::get() {
  static I2CPlatform i2cPlatform;
  return i2cPlatform;
}

template <>
PWMPlatform& Platform::get() {
  static PWMPlatform pwmPlatform;
  return pwmPlatform;
}

template <>
SPIPlatform& Platform::get() {
  static SPIPlatform spiPlatform;
  return spiPlatform;
}

template <>
USARTPlatform& Platform::get() {
  static USARTPlatform usartPlatform;
  return usartPlatform;
}

void Platform::init() {
  get<I2CPlatform>();
  get<PWMPlatform>();
  get<SPIPlatform>();
  get<USARTPlatform>();

  get<Gyroscope>().init();
  get<Accelerometer>().init();
}
