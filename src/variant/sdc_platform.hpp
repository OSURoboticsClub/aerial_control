#ifndef SDC_PLATFORM_HPP_
#define SDC_PLATFORM_HPP_

#include "ch.hpp"
#include "hal.h"

class SDCPlatform {
public:
  SDCPlatform();

  SDCDriver& getSDCDriver();
};

#endif
