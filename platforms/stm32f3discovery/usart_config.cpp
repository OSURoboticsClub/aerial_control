#include <hal.h>

#include <usart_config.hpp>

void usartPlatformInit(void) {
  sdStart(&SD1, &usart1_config);
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
}

