#include <pwm_motor.hpp>

PWMMotor::PWMMotor(PWMDriver *pwm, pwmchannel_t chan) {
  // TODO: Initialize the pwm driver somewhere
  this->pwm = pwm;
  this->chan = chan;
}

void PWMMotor::setSpeed(float percent) {
  pwmcnt_t dc = PWM_PERCENTAGE_TO_WIDTH(pwm, percent * 10000);
  pwmEnableChannel(pwm, chan, dc);
}
