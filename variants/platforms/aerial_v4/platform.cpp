#include "variant/platform.hpp"

#include "hal.h"

#include "drivers/mpu9250.hpp"
#include "sensor/accelerometer.hpp"
#include "sensor/gyroscope.hpp"

#include "drivers/ublox_neo7.hpp"
#include "sensor/gps.hpp"

#include "variant/i2c_platform.hpp"
#include "variant/pwm_platform.hpp"
#include "variant/spi_platform.hpp"
#include "variant/usart_platform.hpp"

Platform::Platform() {
}

// MS5611 SPI configuration
// TODO(yoos): verify clock speed
static const SPIConfig MS5611_CONFIG {
  NULL,
  GPIOC,
  13,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

// MPU9250 SPI configuration
static const SPIConfig MPU9250_CONFIG {
  NULL,
  GPIOC,
  14,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

// H3LIS331DL SPI configuration
// TODO(yoos): verify clock speed
static const SPIConfig H3LIS331DL_CONFIG {
  NULL,
  GPIOC,
  15,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

template <>
MPU9250& Platform::get() {
  static MPU9250 imu(&SPID1, &MPU9250_CONFIG);
  return imu;
}

template <>
Gyroscope& Platform::get() {
  return get<MPU9250>();
}

template <>
Accelerometer& Platform::get() {
  return get<MPU9250>();
}

template <>
UBloxNEO7& Platform::get() {
  static UBloxNEO7 gps(&SD6);
  return gps;
}

template <>
GPS& Platform::get() {
  return get<UBloxNEO7>();
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

  // Initialize IMU
  get<MPU9250>().init();

  // Initialize GPS
  get<UBloxNEO7>().init();
}
