#include "variant/usart_platform.hpp"

#include "hal.h"

// USART1 configuration
static const SerialConfig USART1_CONFIG {
  38400,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

// USART3 configuration
static const SerialConfig USART3_CONFIG {
  38400,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

USARTPlatform::USARTPlatform() {
  sdStart(&SD1, &USART1_CONFIG);
  sdStart(&SD3, &USART3_CONFIG);
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7)); // USART1 TX
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7)); // USART1 RX
  palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7)); // USART3 TX
  palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7)); // USART3 RX
}

chibios_rt::BaseSequentialStreamInterface& USARTPlatform::getPrimaryStream() {
  return reinterpret_cast<chibios_rt::BaseSequentialStreamInterface&>(SD1);
}
