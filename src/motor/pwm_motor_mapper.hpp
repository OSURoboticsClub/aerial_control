#ifndef PWM_MOTOR_MAPPER_HPP_
#define PWM_MOTOR_MAPPER_HPP_

#include <array>

#include "hal.h"

#include "controller/setpoint_types.hpp"
#include "motor/motor_mapper.hpp"
#include "variant/pwm_platform.hpp"

template <int motor_count>
class PWMMotorMapper : public MotorMapper {
public:
  void init() override;
  virtual void run(bool armed, actuator_setpoint_t& input) = 0;

protected:
  PWMMotorMapper(PWMPlatform& pwmPlatform);

  void setMotorSpeeds(bool armed, int chanOff, const std::array<float, motor_count>& percents,
      float rangeMin = 0.0f, float rangeMax = 1.0f);

private:
  PWMPlatform& pwmPlatform;
};

#include "motor/pwm_motor_mapper.tpp"

#endif
