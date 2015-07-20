#ifndef SDC_PLATFORM_HPP_
#define SDC_PLATFORM_HPP_

#include "ch.hpp"
#include "hal.h"

class SDCPlatform {
public:
  /**
   * Get the singleton instance.
   */
  static SDCPlatform& getInstance() {
    static SDCPlatform platform;
    return platform;
  }

  SDCDriver& getSDCDriver();

private:
  SDCPlatform();
};

#endif
