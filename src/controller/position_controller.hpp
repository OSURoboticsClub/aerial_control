#ifndef POSITION_CONTROLLER_HPP_
#define POSITION_CONTROLLER_HPP_

#include "controller/controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"

class PositionController : public Controller<PositionSetpoint, AngularPositionSetpoint> {
public:
  PositionController();

  AngularPositionSetpoint run(const WorldEstimate& world, const PositionSetpoint& input) override;

private:
  PIDController latPid;
  PIDController lonPid;
  PIDController altitudePid;
};

#endif
