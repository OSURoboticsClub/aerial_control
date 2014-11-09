#include <hal.h>

#include <pwm_config.hpp>

void *pwmPlatformInit(void) {
  pwmStart(&PWMD3, &motor_pwm_config);
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(4));
}

// TODO(yoos): process multiple drivers
void pwmPlatformSet(uint8_t ch, float dc) {
  pwmcnt_t width = PWM_PERCENTAGE_TO_WIDTH(&PWMD1, dc * 10000);
  pwmEnableChannel(&PWMD1, ch, width);
}

