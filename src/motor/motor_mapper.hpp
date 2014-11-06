#ifndef MOTOR_MAPPER_HPP_
#define MOTOR_MAPPER_HPP_

#include <controller/setpoint_types.hpp>

class MotorMapper {
public:
  virtual void init() =0;

  /**
   * Maps roll/pitch/yaw setpoints to motor output values.
   */
  virtual void run(actuator_setpoint_t& input) =0;
};

#endif
