#include <hal.h>

#include <usart_config.hpp>

void usartPlatformInit(void) {
  sdStart(&SD1, &usart1_config);
  sdStart(&SD3, &usart3_config);
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));   // USART1 TX
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));   // USART1 RX
  palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7));   // USART3 TX
  palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7));   // USART RX
}

