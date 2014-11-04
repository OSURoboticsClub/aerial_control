#include <hal.h>

#include <spi_config.hpp>

void spiPlatformInit(void) {
  spiStart(&SPID1, &spi1_config);
}
