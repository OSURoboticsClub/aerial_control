#include "controller/angular_position_controller.hpp"

#include <algorithm>

#include "unit_config.hpp"

AngularPositionController::AngularPositionController()
  : rollPosPid(unit_config::ANGPOS_X_KP, unit_config::ANGPOS_X_KI, unit_config::ANGPOS_X_KD),
    pitchPosPid(unit_config::ANGPOS_Y_KP, unit_config::ANGPOS_Y_KI, unit_config::ANGPOS_Y_KD),
    yawPosPid(unit_config::ANGPOS_Z_KP, unit_config::ANGPOS_Z_KI, unit_config::ANGPOS_Z_KD) {
}

angular_velocity_setpoint_t AngularPositionController::run(const attitude_estimate_t& estimate, const angular_position_setpoint_t& input) {
  // Limit to maximum angles
  float rollPosSp = std::max(-unit_config::MAX_PITCH_ROLL_POS, std::min(unit_config::MAX_PITCH_ROLL_POS, input.roll_pos_sp));
  float pitchPosSp = std::max(-unit_config::MAX_PITCH_ROLL_POS, std::min(unit_config::MAX_PITCH_ROLL_POS, input.pitch_pos_sp));

  // Run PID controllers
  float rollVelSp = rollPosPid.calculate(rollPosSp, estimate.roll, unit_config::DT);
  float pitchVelSp = pitchPosPid.calculate(pitchPosSp, estimate.pitch, unit_config::DT);
  float yawVelSp = yawPosPid.calculate(input.yaw_pos_sp, estimate.yaw, unit_config::DT);

  // Output
  angular_velocity_setpoint_t setpoint {
    .roll_vel_sp = rollVelSp,
    .pitch_vel_sp = pitchVelSp,
    .yaw_vel_sp = yawVelSp,
    .throttle_sp = input.throttle_sp
  };

  return setpoint;
}
