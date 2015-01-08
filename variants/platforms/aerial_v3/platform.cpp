#include "variant/platform.hpp"

#include "hal.h"

#include "drivers/mpu6000.hpp"
#include "sensor/accelerometer.hpp"
#include "sensor/gyroscope.hpp"
#include "variant/i2c_platform.hpp"
#include "variant/pwm_platform.hpp"
#include "variant/spi_platform.hpp"
#include "variant/usart_platform.hpp"

// MPU6000 SPI configuration
static const SPIConfig mpu6000_spicfg = {
  NULL,
  GPIOB,
  2,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

Platform::Platform() {
}

template <>
MPU6000& Platform::get() {
  static MPU6000 imu(&SPID1, &mpu6000_spicfg);
  return imu;
}

template <>
Gyroscope& Platform::get() {
  return get<MPU6000>();
}

template <>
Accelerometer& Platform::get() {
  return get<MPU6000>();
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
  get<MPU6000>().init();
}
