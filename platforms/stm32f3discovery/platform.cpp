#include "variant/platform.hpp"

#include "drivers/l3gd20.hpp"
#include "drivers/lsm303dlhc.hpp"
#include "sensor/accelerometer.hpp"
#include "sensor/gyroscope.hpp"
#include "variant/i2c_platform.hpp"
#include "variant/pwm_platform.hpp"
#include "variant/spi_platform.hpp"
#include "variant/usart_platform.hpp"

// L3GD20 SPI configuration
static const SPIConfig l3gd20_spi_config = {
  NULL,
  GPIOE,
  GPIOE_SPI1_CS,
  SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA,
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

static I2CPlatform i2cPlatform;
static PWMPlatform pwmPlatform;
static SPIPlatform spiPlatform;
static USARTPlatform usartPlatform;

static L3GD20 gyro(&SPID1, &l3gd20_spi_config);
static LSM303DLHC accel(&I2CD1);

Platform::Platform() {
}

void Platform::init() {
  gyro.init();
  accel.init();
}

template <>
Gyroscope& Platform::get() {
  return gyro;
}

template <>
Accelerometer& Platform::get() {
  return accel;
}

template <>
PWMPlatform& Platform::get() {
  return pwmPlatform;
}
