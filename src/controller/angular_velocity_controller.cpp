#include "controller/angular_velocity_controller.hpp"

#include <algorithm>

#include "unit_config.hpp"

AngularVelocityController::AngularVelocityController()
  : rollVelPid(unit_config::ANGVEL_X_KP, unit_config::ANGVEL_X_KI, unit_config::ANGVEL_X_KD),
    pitchVelPid(unit_config::ANGVEL_Y_KP, unit_config::ANGVEL_Y_KI, unit_config::ANGVEL_Y_KD),
    yawVelPid(unit_config::ANGVEL_Z_KP, unit_config::ANGVEL_Z_KI, unit_config::ANGVEL_Z_KD) {
}

angular_acceleration_setpoint_t AngularVelocityController::run(const attitude_estimate_t& estimate, const angular_velocity_setpoint_t& input) {
  // Limit to maximum angular velocities
  float rollVelSp = std::max(-unit_config::MAX_PITCH_ROLL_VEL, std::min(unit_config::MAX_PITCH_ROLL_VEL, input.roll_vel_sp));
  float pitchVelSp = std::max(-unit_config::MAX_PITCH_ROLL_VEL, std::min(unit_config::MAX_PITCH_ROLL_VEL, input.pitch_vel_sp));

  // Run PID controllers
  float rollAccSp = rollVelPid.calculate(rollVelSp, estimate.roll_vel, unit_config::DT);
  float pitchAccSp = pitchVelPid.calculate(pitchVelSp, estimate.pitch_vel, unit_config::DT);
  float yawAccSp = yawVelPid.calculate(input.yaw_vel_sp, estimate.yaw_vel, unit_config::DT);

  // Output
  angular_acceleration_setpoint_t setpoint {
    .roll_acc_sp = rollAccSp,
    .pitch_acc_sp = pitchAccSp,
    .yaw_acc_sp = yawAccSp,
    .throttle_sp = input.throttle_sp
  };

  return setpoint;
}
