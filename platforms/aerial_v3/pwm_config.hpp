#ifndef PWM_CONFIG_HPP_
#define PWM_CONFIG_HPP_

#include <hal.h>

const PWMConfig motor_pwm_config = {
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

void *pwmPlatformInit(void);
void pwmPlatformSet(uint8_t ch, float dc);

#endif // PWM_CONFIG_HPP_
