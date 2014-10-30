#ifndef MOTOR_MAPPER_HPP_
#define MOTOR_MAPPER_HPP_

#include <controller/setpoint_types.hpp>

class MotorMapper {
public:
  virtual void init() =0;
  virtual void run(actuator_setpoint_t& input) =0;
};

#endif
