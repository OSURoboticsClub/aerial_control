#include "variant/platform.hpp"

#include "hal.h"

#include "drivers/mpu9250.hpp"
#include "drivers/ms5611.hpp"
#include "drivers/ublox_neo7.hpp"

#include "variant/digital_platform.hpp"
#include "variant/i2c_platform.hpp"
#include "variant/pwm_platform.hpp"
#include "variant/sdc_platform.hpp"
#include "variant/spi_platform.hpp"
#include "variant/usart_platform.hpp"

// MPU9250 SPI configuration
static const SPIConfig MPU9250_CONFIG {
  NULL,
  GPIOC,
  14,
  SPI_CR1_BR_0   // 21000000/2^1 = 10500000
};

// MS5611 SPI configuration
static const SPIConfig MS5611_CONFIG {
  NULL,
  GPIOC,
  13,
  SPI_CR1_BR_0   // 21000000/2^1 = 10500000
};

Platform::Platform() {
}

template <>
MPU9250& Platform::get() {
  static MPU9250 imu(&SPID3, &MPU9250_CONFIG);
  return imu;
}

template <>
UBloxNEO7& Platform::get() {
  static UBloxNEO7 gps(&SD6);
  return gps;
}

template <> Accelerometer& Platform::get() { return get<MPU9250>(); }
template <> Barometer&     Platform::get() { return get<MS5611(); }
template <> GPS&           Platform::get() { return get<UBloxNEO7>(); }
template <> Gyroscope&     Platform::get() { return get<MPU9250>(); }

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
SDCPlatform& Platform::get() {
  static SDCPlatform sdcPlatform;
  return sdcPlatform;
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
  get<DigitalPlatform>();
  get<I2CPlatform>();
  get<PWMPlatform>();
  get<SPIPlatform>();
  get<USARTPlatform>();

  get<SDCPlatform>();   // Initialize SDIO after SPI.

  // Initialize sensors
  get<MPU9250>().init();
  get<MS5611>().init();
  get<UBloxNEO7>().init();
}
