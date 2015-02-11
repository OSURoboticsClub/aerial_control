#ifndef ANGULAR_VELOCITY_CONTROLLER_HPP_
#define ANGULAR_VELOCITY_CONTROLLER_HPP_

#include "controller/controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"

class AngularVelocityController : public Controller<AngularVelocitySetpoint, AngularAccelerationSetpoint> {
public:
  AngularVelocityController();

  AngularAccelerationSetpoint run(const AttitudeEstimate& estimate, const AngularVelocitySetpoint& input) override;

private:
  PIDController rollVelPid;
  PIDController pitchVelPid;
  PIDController yawVelPid;
};

#endif
