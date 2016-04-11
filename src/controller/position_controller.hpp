#ifndef POSITION_CONTROLLER_HPP_
#define POSITION_CONTROLLER_HPP_

#include "global_parameters.hpp"
#include "controller/controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"
#include "params/parameter_repository.hpp"

class PositionController : public Controller<PositionSetpoint, AngularPositionSetpoint> {
public:
  PositionController(ParameterRepository& params);

  AngularPositionSetpoint run(const WorldEstimate& world, const PositionSetpoint& input) override;

private:
  ParameterRepository& params;

  PIDController latPid;
  PIDController lonPid;
  PIDController altitudePid;
};

#endif
