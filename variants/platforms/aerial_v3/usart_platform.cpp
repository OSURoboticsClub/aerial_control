#include "variant/usart_platform.hpp"

#include "hal.h"

// USART1 configuration
static const SerialConfig usart1_config = {
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

// USART3 configuration
static const SerialConfig usart3_config = {
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

USARTPlatform::USARTPlatform() {
  sdStart(&SD1, &usart1_config);
  sdStart(&SD3, &usart3_config);
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7)); // USART1 TX
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7)); // USART1 RX
  palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7)); // USART3 TX
  palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7)); // USART RX
}
