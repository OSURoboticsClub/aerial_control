#include "variant/spi_platform.hpp"

#include "hal.h"

SPIPlatform::SPIPlatform() {
  palSetPadMode(GPIOB, 3, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST); // SPI1 SCK
  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST); // SPI1 MISO
  palSetPadMode(GPIOB, 5, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST); // SPI1 MOSI
  palSetPadMode(GPIOB, 2, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); // MPU-6000 CS
  palSetPad(GPIOB, 2); // Unselect
}
