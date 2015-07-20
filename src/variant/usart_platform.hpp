#ifndef USART_PLATFORM_HPP_
#define USART_PLATFORM_HPP_

#include "ch.hpp"
#include "hal.h"

class USARTPlatform {
public:
  /**
   * Get the singleton instance.
   */
  static USARTPlatform& getInstance() {
    static USARTPlatform platform;
    return platform;
  }

  chibios_rt::BaseSequentialStreamInterface& getPrimaryStream();

private:
  USARTPlatform();

  USARTPlatform(USARTPlatform& platform) = delete;
  void operator=(USARTPlatform& platform) = delete;
};

#endif
