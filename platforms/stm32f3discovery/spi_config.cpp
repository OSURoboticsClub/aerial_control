#include <hal.h>

#include <spi_config.hpp>

void spiPlatformInit(void) {
  spiInit();
  spiStart(&SPID1, &l3gd20_spi_config);
}
