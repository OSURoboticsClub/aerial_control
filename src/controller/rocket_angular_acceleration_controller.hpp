#ifndef ROCKET_ANGULAR_ACCELERATION_CONTROLLER_HPP_
#define ROCKET_ANGULAR_ACCELERATION_CONTROLLER_HPP_

#include "controller/controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"

class RocketAngularAccelerationController : public Controller<AngularAccelerationSetpoint, ActuatorSetpoint> {
public:
  RocketAngularAccelerationController();

  ActuatorSetpoint run(const AttitudeEstimate& estimate, const AngularAccelerationSetpoint& input) override;

private:
  PIDController rollAccPid;
  PIDController pitchAccPid;
  PIDController yawAccPid;
};

#endif
