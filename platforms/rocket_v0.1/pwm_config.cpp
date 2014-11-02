#include <hal.h>

#include <pwm_config.hpp>

PWMDriver *pwmPlatformInit(void) {
  pwmStart(&PWMD1, &motor_pwm_config);
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(4));
  return &PWMD1;
}

// TODO(yoos): process multiple drivers
void pwmPlatformSet(uint8_t ch, float dc) {
  pwmcnt_t width = PWM_PERCENTAGE_TO_WIDTH(&PWMD1, dc * 10000);
  pwmEnableChannel(&PWMD1, ch, width);
}

