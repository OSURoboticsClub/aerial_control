#include "variant/platform.hpp"

#include "hal.h"

#include "drivers/h3lis331dl.hpp"
#include "drivers/mpu9250.hpp"
#include "drivers/ms5611.hpp"
#include "drivers/ublox_neo7.hpp"
#include "variant/i2c_platform.hpp"
#include "variant/pwm_platform.hpp"
#include "variant/spi_platform.hpp"
#include "variant/usart_platform.hpp"

// H3LIS331DL SPI configuration
// TODO(yoos): verify clock speed
static const SPIConfig H3LIS331DL_CONFIG {
  NULL,
  GPIOC,
  15,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

// MPU9250 SPI configuration
static const SPIConfig MPU9250_CONFIG {
  NULL,
  GPIOC,
  14,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

// MS5611 SPI configuration
// TODO(yoos): verify clock speed
static const SPIConfig MS5611_CONFIG {
  NULL,
  GPIOC,
  13,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

Platform::Platform() {
}

template <>
H3LIS331DL& Platform::get() {
  static H3LIS331DL acc(&SPID1, &H3LIS331DL_CONFIG);
  return acc;
}

template <>
MPU9250& Platform::get() {
  static MPU9250 imu(&SPID1, &MPU9250_CONFIG);
  return imu;
}

template <>
MS5611& Platform::get() {
  static MS5611 bar(&SPID1, &MS5611_CONFIG);
  return bar;
}

template <>
UBloxNEO7& Platform::get() {
  static UBloxNEO7 gps(&SD6);
  return gps;
}

template <> Accelerometer& Platform::getIdx(int idx) {
  if (idx == 1)                           { return get<H3LIS331DL>(); }
  else                                    { return get<MPU9250>(); }
}
template <> Barometer&    Platform::get() { return get<MS5611>(); }
template <> GPS&          Platform::get() { return get<UBloxNEO7>(); }
template <> Gyroscope&    Platform::get() { return get<MPU9250>(); }
template <> Magnetometer& Platform::get() { return get<MPU9250>(); }

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

  // Initialize sensors
  get<H3LIS331DL>().init();
  get<MPU9250>().init();
  get<MS5611>().init();
  get<UBloxNEO7>().init();
}
