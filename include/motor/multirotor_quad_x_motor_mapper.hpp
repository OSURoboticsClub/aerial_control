#ifndef QUAD_X_MULTIROTOR_MOTOR_MAPPER_HPP_
#define QUAD_X_MULTIROTOR_MOTOR_MAPPER_HPP_

#include <controller/setpoint_types.hpp>
#include <motor/pwm_motor_mapper.hpp>

class MultirotorQuadXMotorMapper : public PWMMotorMapper<4> {
public:
  MultirotorQuadXMotorMapper();

  void init() override;
  void run(actuator_setpoint_t& input) override;
};

#endif
