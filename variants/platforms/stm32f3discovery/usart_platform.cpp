#include "variant/usart_platform.hpp"

#include "hal.h"

// USART1 configuration
static const SerialConfig USART1_CONFIG {
  38400,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

USARTPlatform::USARTPlatform() {
  sdStart(&SD1, &USART1_CONFIG);
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
}

chibios_rt::BaseSequentialStreamInterface& USARTPlatform::getPrimaryStream() {
  return reinterpret_cast<chibios_rt::BaseSequentialStreamInterface&>(SD1);
}
