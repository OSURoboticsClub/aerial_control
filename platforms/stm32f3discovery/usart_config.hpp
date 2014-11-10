#ifndef USART_CONFIG_HPP_
#define USART_CONFIG_HPP_

#include <hal.h>

// USART1 configuration
const SerialConfig usart1_config = {
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

void usartPlatformInit(void);

#endif // USART_CONFIG_HPP_
