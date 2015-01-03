#include "variant/spi_platform.hpp"

#include "hal.h"

SPIPlatform::SPIPlatform() {
  spiInit();
  // TODO: spiStart is called in spi_device.cpp
  // spiStart(&SPID1, &l3gd20_spi_config);
}
