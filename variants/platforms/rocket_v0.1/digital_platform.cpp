#include "variant/digital_platform.hpp"

#include "hal.h"

DigitalPlatform::DigitalPlatform() {
  palSetPadMode(GPIOA, 4, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 5, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOC, 4, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOC, 5, PAL_MODE_OUTPUT_PUSHPULL);

  palClearPad(GPIOA, 4);
  palClearPad(GPIOA, 5);
  palClearPad(GPIOC, 4);
  palClearPad(GPIOC, 5);
}

void DigitalPlatform::set(std::uint8_t ch, bool on) {
  switch (ch) {
  case 0:
    palWritePad(GPIOA, 4, on);
    break;
  case 1:
    palWritePad(GPIOA, 5, on);
    break;
  case 2:
    palWritePad(GPIOC, 4, on);
    break;
  case 3:
    palWritePad(GPIOC, 5, on);
    break;
  default:
    break;
  }
}
