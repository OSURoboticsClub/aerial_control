template <int motor_count>
PWMMotorMapper<motor_count>::PWMMotorMapper() {
  for(int i = 0; i < motor_count; i++) {
    channels[i] = i; // TODO: Make this configurable
  }
}

template <int motor_count>
void PWMMotorMapper<motor_count>::init() {
  PWMConfig pwmConfig = {
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

  pwmStart(&PWMD1, &pwmConfig);
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));
}

template <int motor_count>
void PWMMotorMapper<motor_count>::setMotorSpeed(int motor, float percent) {
  pwmcnt_t dc = PWM_PERCENTAGE_TO_WIDTH(pwm, percent * 10000);
  pwmEnableChannel(pwm, channels[motor], dc);
}
