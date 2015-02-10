#include "variant/adc_platform.hpp"

#include "hal.h"

ADCPlatform::ADCPlatform() {
  /**
   * Initialize ADC driver 1 and set the following as ADC inputs: PC0, PC1
   */
  adcStart(&ADCD1, NULL);

  palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
}
