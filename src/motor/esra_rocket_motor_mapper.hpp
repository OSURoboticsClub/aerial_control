#ifndef ESRA_ROCKET_MOTOR_MAPPER_HPP_
#define ESRA_ROCKET_MOTOR_MAPPER_HPP_

#include "controller/setpoint_types.hpp"
#include "motor/pwm_motor_mapper.hpp"

class EsraRocketMotorMapper : public PWMMotorMapper<4> {
public:
  EsraRocketMotorMapper(PWMPlatform& pwmPlatform);

  void init() override;
  void run(bool armed, actuator_setpoint_t& input) override;
};

#endif
