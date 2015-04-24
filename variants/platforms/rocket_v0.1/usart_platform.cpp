#include "variant/usart_platform.hpp"

#include "hal.h"

// UART2 configuration (XBee)
static const SerialConfig UART2_CONFIG {
  38400,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

// UART4 configuration (Unused)
static const SerialConfig UART4_CONFIG {
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

// UART6 configuration (GPS)
static const SerialConfig UART6_CONFIG {
  9600,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

USARTPlatform::USARTPlatform() {
  // Force RX high on startup to prevent XBee from entering SPI mode
  palSetPadMode(GPIOA, 3, PAL_MODE_OUTPUT_PUSHPULL);   // RX
  palSetPad(GPIOA, 3);
  chThdSleepMilliseconds(500);

  // UART2
  sdStart(&SD2, &UART2_CONFIG);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));   // TX
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));   // RX

  // UART4
  sdStart(&SD4, &UART4_CONFIG);
  palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(8));   // TX
  palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(8));   // RX

  // UART6
  sdStart(&SD6, &UART6_CONFIG);
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(8));   // TX
  palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(8));   // RX
}

chibios_rt::BaseSequentialStreamInterface& USARTPlatform::getPrimaryStream() {
  return reinterpret_cast<chibios_rt::BaseSequentialStreamInterface&>(SD2);
}
