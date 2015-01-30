#include "controller/position_controller.hpp"

#include <algorithm>

#include "unit_config.hpp"

PositionController::PositionController()
  : latPid(10.0, 0.0, 0.0),
    lonPid(10.0, 0.0, 0.0),
    altitudePid(10.0, 0.0, 0.0) {
}

angular_position_setpoint_t PositionController::run(const attitude_estimate_t& estimate, const position_setpoint_t& input) {
  // TODO(kyle): Transform from local x/y to global lat/long (need mag)

  float rollPosSp = latPid.calculate(input.latitude_sp, 0 /* TODO: estimate.latitude */, unit_config::DT);
  float pitchPosSp = lonPid.calculate(input.longitude_sp, 0 /* TODO: estimate.longitude */, unit_config::DT);
  float throttleSp = altitudePid.calculate(input.altitude_sp, 0 /* TODO: estimate.altitude */, unit_config::DT);

  angular_position_setpoint_t setpoint {
    .roll_pos_sp = rollPosSp,
    .pitch_pos_sp = pitchPosSp,
    .yaw_pos_sp = input.yaw_pos_sp,
    .throttle_sp = throttleSp
  };

  return setpoint;
}
