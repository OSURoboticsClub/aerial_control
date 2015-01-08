#include "variant/spi_platform.hpp"

#include "hal.h"

SPIPlatform::SPIPlatform() {
  palSetPadMode(GPIOB, 2, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); // MPU-6000 CS
  palSetPad(GPIOB, 2); //Unselect
}
