#ifndef ANGULAR_ACCELERATION_CONTROLLER_HPP_
#define ANGULAR_ACCELERATION_CONTROLLER_HPP_

#include "controller/controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"

class AngularAccelerationController : public Controller<AngularAccelerationSetpoint, ActuatorSetpoint> {
public:
  AngularAccelerationController();

  ActuatorSetpoint run(const WorldEstimate& world, const AngularAccelerationSetpoint& input) override;

private:
  PIDController rollAccPid;
  PIDController pitchAccPid;
  PIDController yawAccPid;
};

#endif
