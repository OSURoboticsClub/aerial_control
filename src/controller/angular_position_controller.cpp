#include <controller/angular_position_controller.hpp>

#include <hal_config.hpp>

#include <algorithm>

AngularPositionController::AngularPositionController()
  : rollPosPid(ANGPOS_X_KP, ANGPOS_X_KI, ANGPOS_X_KD),
    pitchPosPid(ANGPOS_Y_KP, ANGPOS_Y_KI, ANGPOS_Z_KD),
    yawPosPid(ANGPOS_Z_KP, ANGPOS_Z_KI, ANGPOS_Z_KD) {
}

angular_velocity_setpoint_t AngularPositionController::run(const attitude_estimate_t& estimate, const angular_position_setpoint_t& input) {
  // Limit to maximum angles
  float rollPosSp = std::max(-MAX_PITCH_ROLL_POS, std::min(MAX_PITCH_ROLL_POS, input.roll_pos_sp));
  float pitchPosSp = std::max(-MAX_PITCH_ROLL_POS, std::min(MAX_PITCH_ROLL_POS, input.pitch_pos_sp));

  float rollVelSp = rollPosPid.calculate(rollPosSp, estimate.roll, DT);
  float pitchVelSp = pitchPosPid.calculate(pitchPosSp, estimate.pitch, DT);
  float yawVelSp = yawPosPid.calculate(input.yaw_pos_sp, estimate.yaw, DT);

  angular_velocity_setpoint_t setpoint = {
    .roll_vel_sp = rollVelSp,
    .pitch_vel_sp = pitchVelSp,
    .yaw_vel_sp = yawVelSp,
    .throttle_sp = input.throttle_sp
  };

  return setpoint;
}
