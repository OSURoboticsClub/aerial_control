#include "controller/position_controller.hpp"

#include <algorithm>

#include "unit_config.hpp"

PositionController::PositionController()
  : latPid(10.0, 0.0, 0.0),
    lonPid(10.0, 0.0, 0.0),
    altitudePid(10.0, 0.0, 0.0) {
}

AngularPositionSetpoint PositionController::run(const WorldEstimate& world, const PositionSetpoint& input) {
  // TODO(kyle): Weird effects at the poles?
  float globalRollPosSp = latPid.calculate(input.lat, world.loc.lat, unit_config::DT);
  float globalPitchPosSp = lonPid.calculate(input.lon, world.loc.lon, unit_config::DT);
  float throttleSp = altitudePid.calculate(input.alt, world.loc.alt, unit_config::DT);

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
