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
  return I2CPlatform::getInstance();
}

template <>
PWMPlatform& Platform::get() {
  return PWMPlatform::getInstance();
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
  get<I2CPlatform>();
  get<PWMPlatform>();
  get<SPIPlatform>();
  get<USARTPlatform>();

  get<ICUPlatform>().init();
}
