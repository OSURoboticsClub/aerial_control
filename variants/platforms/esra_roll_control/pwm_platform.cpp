#include "variant/pwm_platform.hpp"

#include "hal.h"

// TODO(yoos): v4.0 will switch these around so we can share a 50 Hz timer
// channel between servos and status LEDs.
static const PWMConfig PWMD1_CONFIG {
  500000,    // 500 kHz PWM clock frequency.
  1000,      // PWM period 2.0 ms.
  NULL,      // No callback.
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },   // Channel configurations
  0,0   // HW dependent
};

static const PWMConfig PWMD3_CONFIG {
  500000,    // 500 kHz PWM clock frequency.
  1000,      // PWM period 2.0 ms.
  NULL,      // No callback.
  {
    {PWM_OUTPUT_DISABLED, NULL},      // RSSI
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},   // Status R
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},   // Status G
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}    // Status B
  },   // Channel configurations
  0,0   // HW dependent
};

static const PWMConfig PWMD4_CONFIG {
  500000,    // 500 kHz PWM clock frequency.
  2000,      // PWM period 4.0 ms.
  NULL,      // No callback.
  {
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },   // Channel configurations
  0,0   // HW dependent
};

PWMPlatform::PWMPlatform() {
  // TIM1
  pwmStart(&PWMD1, &PWMD1_CONFIG);
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(1));

  // TIM3
  pwmStart(&PWMD3, &PWMD3_CONFIG);
  palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(2));
  palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(2));
  palSetPadMode(GPIOB, 0, PAL_MODE_ALTERNATE(2));
  palSetPadMode(GPIOB, 1, PAL_MODE_ALTERNATE(2));

  // TIM4
  pwmStart(&PWMD4, &PWMD4_CONFIG);
  palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(2));
  palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(2));
}

void PWMPlatform::set(std::uint8_t ch, float dc) {
  if (ch < 4) {
    pwmcnt_t width = PWM_PERCENTAGE_TO_WIDTH(&PWMD1, dc * 10000.0f);
    pwmEnableChannel(&PWMD1, ch, width);
  }
  else if (ch < 8) {
    pwmcnt_t width = PWM_PERCENTAGE_TO_WIDTH(&PWMD4, dc * 10000.0f);
    pwmEnableChannel(&PWMD4, ch-4, width);
  }
  else if (ch < 12) {
    pwmcnt_t width = PWM_PERCENTAGE_TO_WIDTH(&PWMD3, dc * 10000.0f);
    pwmEnableChannel(&PWMD3, ch-8, width);
  }
}
