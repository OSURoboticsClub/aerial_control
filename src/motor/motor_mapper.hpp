#ifndef MOTOR_MAPPER_HPP_
#define MOTOR_MAPPER_HPP_

#include "controller/setpoint_types.hpp"

/**
 * Maps roll/pitch/yaw setpoints to hardware controllers.
 */
class MotorMapper {
public:
  virtual void run(bool armed, ActuatorSetpoint& input) = 0;
};

#endif
