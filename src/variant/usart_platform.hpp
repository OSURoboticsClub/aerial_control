#ifndef USART_PLATFORM_HPP_
#define USART_PLATFORM_HPP_

#include "ch.hpp"
#include "hal.h"

class USARTPlatform {
public:
  USARTPlatform();

  /**
   * Get the singleton instance.
   */
  static USARTPlatform& getInstance() {
    static USARTPlatform platform;
    return platform;
  }

  chibios_rt::BaseSequentialStreamInterface& getPrimaryStream();
};

#endif
