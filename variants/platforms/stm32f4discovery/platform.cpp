#include "variant/platform.hpp"

#include "hal.h"

#include "variant/i2c_platform.hpp"
#include "variant/pwm_platform.hpp"
#include "variant/spi_platform.hpp"
#include "variant/usart_platform.hpp"

Platform::Platform() {
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

  get<ICUPlatform>().init();
}
