#include "variant/pwm_platform.hpp"

#include "hal.h"

static const PWMConfig MOTOR_PWM_CONFIG {
  500000,    // 500 kHz PWM clock frequency.
  1000,      // PWM period 2.0 ms.
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
  pwmStart(&PWMD1, &MOTOR_PWM_CONFIG);
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(4));
}

void PWMPlatform::set(std::uint8_t ch, float dc) {
  pwmcnt_t width = PWM_PERCENTAGE_TO_WIDTH(&PWMD1, dc * 10000.0f);
  pwmEnableChannel(&PWMD1, ch, width);
}
