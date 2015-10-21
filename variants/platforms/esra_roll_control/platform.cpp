#include "variant/platform.hpp"

#include "hal.h"

#include "drivers/h3lis331dl.hpp"
#include "drivers/mpu6000.hpp"
#include "drivers/ms5611.hpp"
#include "drivers/ublox_neo7.hpp"
#include "variant/digital_platform.hpp"
#include "variant/i2c_platform.hpp"
#include "variant/pwm_platform.hpp"
#include "variant/sdc_platform.hpp"
#include "variant/spi_platform.hpp"
#include "variant/usart_platform.hpp"

// H3LIS331DL SPI configuration
static const SPIConfig H3LIS331DL_CONFIG {
  NULL,
  GPIOC,
  15,
  SPI_CR1_BR_1   // 21000000/2^2 = 5250000
};

// MPU6000 SPI configuration
static const SPIConfig MPU6000_CONFIG {
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
H3LIS331DL& Platform::get() {
  static H3LIS331DL acc(&SPID3, &H3LIS331DL_CONFIG);
  return acc;
}

template <>
MPU6000& Platform::get() {
  static MPU6000 imu(&SPID3, &MPU6000_CONFIG);
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

template <> Accelerometer& Platform::getIdx(int idx) {
  if (idx == 1)                           { return get<H3LIS331DL>(); }
  else                                    { return get<MPU6000>(); }
}
template <> Barometer&    Platform::get() { return get<MS5611>(); }
template <> GPS&          Platform::get() { return get<UBloxNEO7>(); }
template <> Gyroscope&    Platform::get() { return get<MPU6000>(); }

template <>
DigitalPlatform& Platform::get() {
  return DigitalPlatform::getInstance();
}

template <>
I2CPlatform& Platform::get() {
  return I2CPlatform::getInstance();
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
  get<PWMPlatform>();
  get<SPIPlatform>();
  get<USARTPlatform>();   // Do this last so the 500ms delay hack doesn't mess with other stuff.

  get<SDCPlatform>();   // Initialize SDIO after SPI.

  // Initialize sensors
  get<H3LIS331DL>().init();
  get<MPU6000>().init();
  get<MS5611>().init();
  get<UBloxNEO7>().init();
}
