#include "variant/sdc_platform.hpp"

#include "hal.h"

// SDC1 configuration
static const SDCConfig SDC1_CONFIG {
  0   // Dummy
};

SDCPlatform::SDCPlatform() {
  sdcStart(&SDCD1, &SDC1_CONFIG);
  palSetPadMode(GPIOB, 12, PAL_MODE_INPUT_PULLUP);    // uSD-CD
  //palSetPadMode(GPIOC,  8, PAL_MODE_ALTERNATE(12));   // SDIO-D0
  //palSetPadMode(GPIOC,  9, PAL_MODE_ALTERNATE(12));   // SDIO-D1
  //palSetPadMode(GPIOC, 10, PAL_MODE_ALTERNATE(12));   // SDIO-D2
  //palSetPadMode(GPIOC, 11, PAL_MODE_ALTERNATE(12));   // SDIO-D3
  //palSetPadMode(GPIOC, 12, PAL_MODE_ALTERNATE(12));   // SDIO-CK
  //palSetPadMode(GPIOD,  2, PAL_MODE_ALTERNATE(12));   // SDIO-CMD
}

SDCDriver& SDCPlatform::getSDCDriver() {
  return SDCD1;
}
