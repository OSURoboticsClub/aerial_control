#include <controller/angular_position_controller.hpp>

#include <hal_config.hpp>

#include <algorithm>

AngularPositionController::AngularPositionController()
  : pitchPosPid(10.0, 0.0, 0.0),
    rollPosPid(10.0, 0.0, 0.0),
    yawPosPid(10.0, 0.0, 0.0) {
}

angular_velocity_setpoint_t AngularPositionController::run(const attitude_estimate_t& estimate, const angular_position_setpoint_t& input) {
  // Limit to maximum angles
  float pitchPosSp = std::max(-MAX_PITCH_ROLL_POS, std::min(MAX_PITCH_ROLL_POS, input.pitch_pos_sp));
  float rollPosSp = std::max(-MAX_PITCH_ROLL_POS, std::min(MAX_PITCH_ROLL_POS, input.roll_pos_sp));

  float pitchVelSp = pitchPosPid.calculate(pitchPosSp, estimate.pitch, DT);
  float rollVelSp = rollPosPid.calculate(rollPosSp, estimate.roll, DT);
  float yawVelSp = yawPosPid.calculate(input.yaw_pos_sp, estimate.yaw, DT);

  angular_velocity_setpoint_t setpoint = {
    .pitch_vel_sp = pitchVelSp,
    .roll_vel_sp = rollVelSp,
    .yaw_vel_sp = yawVelSp,
    .throttle_sp = input.throttle_sp
  };

  return setpoint;
}
