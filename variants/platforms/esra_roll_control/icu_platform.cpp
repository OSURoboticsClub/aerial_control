#include "variant/icu_platform.hpp"

#include "hal.h"

static void icuCallback(ICUDriver *icup) {
  auto& platform = ICUPlatform::getInstance();
  platform.trigger(icup);
}

static const ICUConfig ICU_CONFIG = {
  ICU_INPUT_ACTIVE_HIGH,
  200000,   // 200 kHz ICU clock frequency
  NULL,
  icuCallback,
  NULL,
  ICU_CHANNEL_1,
  0
};

ICUPlatform::ICUPlatform() {
  icuInit();
  icuStart(&ICUD2, &ICU_CONFIG);
  palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_PULLDOWN | PAL_MODE_ALTERNATE(1));
  icuEnable(&ICUD2);
}
