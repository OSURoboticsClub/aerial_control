#ifndef ANGULAR_POSITION_CONTROLLER_HPP_
#define ANGULAR_POSITION_CONTROLLER_HPP_

#include "controller/controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"

class AngularPositionController : public Controller<AngularPositionSetpoint, AngularVelocitySetpoint> {
public:
  AngularPositionController();

  AngularVelocitySetpoint run(const WorldEstimate& world, const AngularPositionSetpoint& input) override;

private:
  PIDController rollPosPid;
  PIDController pitchPosPid;
  PIDController yawPosPid;
};

#endif
