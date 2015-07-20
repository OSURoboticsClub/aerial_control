#include "variant/platform.hpp"

#include "hal.h"

#include "drivers/mpu9250.hpp"
#include "drivers/ms5611.hpp"
#include "drivers/ublox_neo7.hpp"

#include "variant/digital_platform.hpp"
#include "variant/i2c_platform.hpp"
#include "variant/icu_platform.hpp"
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
MS5611& Platform::get() {
  static MS5611 bar(&SPID3, &MS5611_CONFIG);
  return bar;
}

template <>
UBloxNEO7& Platform::get() {
  static UBloxNEO7 gps(&SD6);
  return gps;
}

template <> Accelerometer& Platform::get() { return get<MPU9250>(); }
template <> Barometer&     Platform::get() { return get<MS5611>(); }
template <> GPS&           Platform::get() { return get<UBloxNEO7>(); }
template <> Gyroscope&     Platform::get() { return get<MPU9250>(); }

template <>
DigitalPlatform& Platform::get() {
  return DigitalPlatform::getInstance();
}

template <>
I2CPlatform& Platform::get() {
  return I2CPlatform::getInstance();
}

template <>
ICUPlatform& Platform::get() {
  return ICUPlatform::getInstance();
}

template <>
PWMPlatform& Platform::get() {
  return PWMPlatform::getInstance();
}

template <>
SDCPlatform& Platform::get() {
  return SDCPlatform::getInstance();
}

template <>
SPIPlatform& Platform::get() {
  return SPIPlatform::getInstance();
}

template <>
USARTPlatform& Platform::get() {
  return USARTPlatform::getInstance();
}

void Platform::init() {
  get<DigitalPlatform>();
  get<I2CPlatform>();
  get<ICUPlatform>();
  get<PWMPlatform>();
  get<SPIPlatform>();
  get<USARTPlatform>();

  get<SDCPlatform>();   // Initialize SDIO after SPI.

  // Initialize sensors
  get<MPU9250>().init();
  get<MS5611>().init();
  get<UBloxNEO7>().init();
}
