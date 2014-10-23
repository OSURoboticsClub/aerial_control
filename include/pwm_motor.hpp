#ifndef PWM_MOTOR_HPP_
#define PWM_MOTOR_HPP_

#include <hal.h>
#include <motor.hpp>

class PWMMotor : public Motor {
public:
  PWMMotor(PWMDriver *pwm, pwmchannel_t chan);

  void setSpeed(float percent);

private:
  PWMDriver *pwm;
  pwmchannel_t chan;
};

#endif
