#include "controller/position_controller.hpp"

#include <algorithm>

PositionController::PositionController(ParameterRepository& params)
  : params(params) {
}

AngularPositionSetpoint PositionController::run(const WorldEstimate& world, const PositionSetpoint& input) {
  // TODO(kyle): Weird effects at the poles?
  float globalRollPosSp = latPid.calculate(input.lat, world.loc.lat, params.get(GlobalParameters::PARAM_DT));
  float globalPitchPosSp = lonPid.calculate(input.lon, world.loc.lon, params.get(GlobalParameters::PARAM_DT));
  float throttleSp = altitudePid.calculate(input.alt, world.loc.alt, params.get(GlobalParameters::PARAM_DT));

  // TODO(kyle): Transform from global angular setpoints to local angular
  // setpoints.
  float bodyRollPosSp = 0.0f;
  float bodyPitchPosSp = 0.0f;

  AngularPositionSetpoint setpoint {
    .rollPos = bodyRollPosSp,
    .pitchPos = bodyPitchPosSp,
    .yawPos = input.yawPos,
    .throttle = throttleSp
  };

  return setpoint;
}
