#include "variant/pwm_platform.hpp"

#include "hal.h"

// PWM config for ESCs using TIM8 channels 1-4 (C6, C7, C8, C9).
// See datasheet page 29 for available timers and their capabilities.
// See datasheet page 45 for pinouts.
static const PWMConfig PWMD8_CONFIG {
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

// PWM config for servos using:
// PA8 - TIM1 channel 1
// PB0 - TIM3 channel 3
// PB1 - TIM3 channel 4
static const PWMConfig PWMD1_CONFIG {
	50000,   // 50 kHz PWM clock frequency.
	1000,    // PWM period 20 ms.
	NULL,    // No callback.
	{
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},
		{PWM_OUTPUT_DISABLED, NULL},
		{PWM_OUTPUT_DISABLED, NULL},
		{PWM_OUTPUT_DISABLED, NULL}
	},   // Channel configurations
	0,0   // HW dependent
};

static const PWMConfig PWMD3_CONFIG {
	50000,   // 50 kHz PWM clock frequency.
	1000,    // PWM period 20 ms.
	NULL,    // No callback.
	{
		{PWM_OUTPUT_DISABLED, NULL},
		{PWM_OUTPUT_DISABLED, NULL},
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},
		{PWM_OUTPUT_ACTIVE_HIGH, NULL}
	},   // Channel configurations
	0,0   // HW dependent
};

PWMPlatform::PWMPlatform() {
  // TIM8
  pwmStart(&PWMD8, &PWMD8_CONFIG);
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(3));
  palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(3));
  palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(3));
  palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(3));

  // TIM1
  pwmStart(&PWMD1, &PWMD1_CONFIG);
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));

  // TIM3
  pwmStart(&PWMD3, &PWMD3_CONFIG);
  palSetPadMode(GPIOB, 0, PAL_MODE_ALTERNATE(2));
  palSetPadMode(GPIOB, 1, PAL_MODE_ALTERNATE(2));
}

void PWMPlatform::set(std::uint8_t ch, float dc) {
  if (ch < 4) {
    pwmcnt_t width = PWM_PERCENTAGE_TO_WIDTH(&PWMD8, dc * 10000.0f);
    pwmEnableChannel(&PWMD8, ch, width);
  }
  else if (ch < 5) {
    pwmcnt_t width = PWM_PERCENTAGE_TO_WIDTH(&PWMD1, dc * 10000.0f);
    pwmEnableChannel(&PWMD1, ch-4, width);
  }
  else if (ch < 7) {
    pwmcnt_t width = PWM_PERCENTAGE_TO_WIDTH(&PWMD3, dc * 10000.0f);
    pwmEnableChannel(&PWMD3, 2+ch-5, width);
  }
}
