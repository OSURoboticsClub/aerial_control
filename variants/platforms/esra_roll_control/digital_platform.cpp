#include "variant/digital_platform.hpp"

#include "hal.h"

DigitalPlatform::DigitalPlatform() {
  palSetPadMode(GPIOC, 4, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOC, 5, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 12, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 15, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, 2, PAL_MODE_OUTPUT_PUSHPULL);

  palClearPad(GPIOC, 4);
  palClearPad(GPIOC, 5);
  palClearPad(GPIOA, 12);
  palClearPad(GPIOA, 15);
  palClearPad(GPIOB, 2);
}

void DigitalPlatform::set(std::uint8_t ch, bool on) {
  switch (ch) {
    case 0: palWritePad(GPIOC, 4, on); break;
    case 1: palWritePad(GPIOC, 5, on); break;
    case 2: palWritePad(GPIOA, 12, on); break;
    case 3: palWritePad(GPIOA, 15, on); break;
    case 4: palWritePad(GPIOB, 2, on); break;
    default: break;
  }
}
