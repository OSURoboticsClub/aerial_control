#include "controller/angular_velocity_controller.hpp"

#include <algorithm>

#include "unit_config.hpp"

AngularVelocityController::AngularVelocityController()
  : rollVelPid(unit_config::ANGVEL_X_KP, unit_config::ANGVEL_X_KI, unit_config::ANGVEL_X_KD),
    pitchVelPid(unit_config::ANGVEL_Y_KP, unit_config::ANGVEL_Y_KI, unit_config::ANGVEL_Y_KD),
    yawVelPid(unit_config::ANGVEL_Z_KP, unit_config::ANGVEL_Z_KI, unit_config::ANGVEL_Z_KD) {
}

AngularAccelerationSetpoint AngularVelocityController::run(const WorldEstimate& world, const AngularVelocitySetpoint& input) {
  // Limit to maximum angular velocities
  float rollVelSp = std::max(-unit_config::MAX_PITCH_ROLL_VEL, std::min(unit_config::MAX_PITCH_ROLL_VEL, input.rollVel));
  float pitchVelSp = std::max(-unit_config::MAX_PITCH_ROLL_VEL, std::min(unit_config::MAX_PITCH_ROLL_VEL, input.pitchVel));

  // Run PID controllers
  float rollAccSp = rollVelPid.calculate(rollVelSp, world.att.rollVel, unit_config::DT);
  float pitchAccSp = pitchVelPid.calculate(pitchVelSp, world.att.pitchVel, unit_config::DT);
  float yawAccSp = yawVelPid.calculate(input.yawVel, world.att.yawVel, unit_config::DT);

  // Output
  AngularAccelerationSetpoint setpoint {
    .rollAcc = rollAccSp,
    .pitchAcc = pitchAccSp,
    .yawAcc = yawAccSp,
    .throttle = input.throttle
  };

  return setpoint;
}
