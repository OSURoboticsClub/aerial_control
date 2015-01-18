#include "variant/pwm_platform.hpp"

#include "hal.h"

static const PWMConfig motor_pwm_config = {
  500000,    // 500 kHz PWM clock frequency.
  2000,      // PWM period 2.0 ms.
  NULL,      // No callback.
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },   // Channel configurations
  0,0   // HW dependent
};

static const PWMConfig servo_pwm_config = {
  50000,     // 50 kHz PWM clock frequency.
  1000,      // PWM period 20.0 ms.
  NULL,      // No callback.
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },   // Channel configurations
  0,0   // HW dependent
};

PWMPlatform::PWMPlatform() {
  pwmStart(&PWMD8, &motor_pwm_config);
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(4));

  pwmStart(&PWMD4, &servo_pwm_config);
  palSetPadMode(GPIOD, 12, PAL_MODE_ALTERNATE(2));
  palSetPadMode(GPIOD, 13, PAL_MODE_ALTERNATE(2));
  palSetPadMode(GPIOD, 14, PAL_MODE_ALTERNATE(2));
  palSetPadMode(GPIOD, 15, PAL_MODE_ALTERNATE(2));
}

void PWMPlatform::set(std::uint8_t ch, float dc) {
  if(ch >= 0 && ch <= 3) {
    pwmcnt_t width = PWM_PERCENTAGE_TO_WIDTH(&PWMD8, dc * 10000.0f);
    pwmEnableChannel(&PWMD8, ch, width);
  } else if(ch >= 4 && ch <= 7) {
    pwmcnt_t width = PWM_PERCENTAGE_TO_WIDTH(&PWMD4, dc * 10000.0f);
    pwmEnableChannel(&PWMD4, ch - 4, width);
  }
}
