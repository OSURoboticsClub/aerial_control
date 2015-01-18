#ifndef USART_PLATFORM_HPP_
#define USART_PLATFORM_HPP_

#include "ch.hpp"
#include "hal.h"

class USARTPlatform {
public:
  USARTPlatform();

  chibios_rt::BaseSequentialStreamInterface& getPrimaryStream();
};

#endif
