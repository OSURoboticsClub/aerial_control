#include "controller/position_controller.hpp"

#include <algorithm>

#include "unit_config.hpp"

PositionController::PositionController()
  : latPid(10.0, 0.0, 0.0),
    lonPid(10.0, 0.0, 0.0),
    altitudePid(10.0, 0.0, 0.0) {
}

AngularPositionSetpoint PositionController::run(const WorldEstimate& world, const PositionSetpoint& input) {
  // TODO(kyle): Transform from local x/y to global lat/long (need mag)

  float rollPosSp = latPid.calculate(input.latitude, 0 /* TODO: world.loc.latitude */, unit_config::DT);
  float pitchPosSp = lonPid.calculate(input.longitude, 0 /* TODO: world.loc.longitude */, unit_config::DT);
  float throttleSp = altitudePid.calculate(input.altitude, 0 /* TODO: world.loc.altitude */, unit_config::DT);

  AngularPositionSetpoint setpoint {
    .rollPos = rollPosSp,
    .pitchPos = pitchPosSp,
    .yawPos = input.yawPos,
    .throttle = throttleSp
  };

  return setpoint;
}
