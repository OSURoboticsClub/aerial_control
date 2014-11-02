#include <controller/angular_velocity_controller.hpp>

#include <config.hpp>

#include <algorithm>

AngularVelocityController::AngularVelocityController()
  : pitchVelPid(10.0, 0.0, 0.0),
    rollVelPid(10.0, 0.0, 0.0),
    yawVelPid(10.0, 0.0, 0.0) {
}

actuator_setpoint_t AngularVelocityController::run(const attitude_estimate_t& estimate, const angular_velocity_setpoint_t& input) {
  float pitchActuatorSp = pitchVelPid.calculate(input.pitch_vel_sp, estimate.pitch_vel, DT);
  float rollActuatorSp = rollVelPid.calculate(input.roll_vel_sp, estimate.roll_vel, DT);
  float yawActuatorSp = yawVelPid.calculate(input.yaw_vel_sp, estimate.yaw_vel, DT);

  // Limit to maximum angular velocities
  pitchActuatorSp = std::max(-MAX_PITCH_ROLL_VEL, std::min(MAX_PITCH_ROLL_VEL, pitchActuatorSp));
  rollActuatorSp = std::max(-MAX_PITCH_ROLL_VEL, std::min(MAX_PITCH_ROLL_VEL, rollActuatorSp));

  actuator_setpoint_t setpoint = {
    .pitch_sp = pitchActuatorSp,
    .roll_sp = rollActuatorSp,
    .yaw_sp = yawActuatorSp,
    .throttle_sp = input.throttle_sp
  };

  return setpoint;
}
