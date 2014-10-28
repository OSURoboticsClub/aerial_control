#ifndef QUAD_X_MULTIROTOR_MOTOR_MAPPER_HPP_
#define QUAD_X_MULTIROTOR_MOTOR_MAPPER_HPP_

#include <motor/pwm_motor_mapper.hpp>

class MultirotorQuadXMotorMapper : public PWMMotorMapper<4> {
public:
  MultirotorQuadXMotorMapper();

  void init();
  void run(controller_output_t& input);
};

#endif
