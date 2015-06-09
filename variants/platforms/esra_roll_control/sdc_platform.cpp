#include "variant/sdc_platform.hpp"

#include "hal.h"

// SDC1 configuration
static const SDCConfig SDC1_CONFIG {
  0   // Dummy
};

SDCPlatform::SDCPlatform() {
  sdcStart(&SDCD1, &SDC1_CONFIG);
  palSetPadMode(GPIOB, 12, PAL_MODE_INPUT_PULLUP);    // uSD-CD
}

SDCDriver& SDCPlatform::getSDCDriver() {
  return SDCD1;
}
