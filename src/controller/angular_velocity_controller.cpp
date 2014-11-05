#include <controller/angular_velocity_controller.hpp>

#include <hal_config.hpp>

#include <algorithm>

AngularVelocityController::AngularVelocityController()
  : rollVelPid(10.0, 0.0, 0.0),
    pitchVelPid(10.0, 0.0, 0.0),
    yawVelPid(10.0, 0.0, 0.0) {
}

actuator_setpoint_t AngularVelocityController::run(const attitude_estimate_t& estimate, const angular_velocity_setpoint_t& input) {
  float rollActuatorSp = rollVelPid.calculate(input.roll_vel_sp, estimate.roll_vel, DT);
  float pitchActuatorSp = pitchVelPid.calculate(input.pitch_vel_sp, estimate.pitch_vel, DT);
  float yawActuatorSp = yawVelPid.calculate(input.yaw_vel_sp, estimate.yaw_vel, DT);

  // Limit to maximum angular velocities
  rollActuatorSp = std::max(-MAX_PITCH_ROLL_VEL, std::min(MAX_PITCH_ROLL_VEL, rollActuatorSp));
  pitchActuatorSp = std::max(-MAX_PITCH_ROLL_VEL, std::min(MAX_PITCH_ROLL_VEL, pitchActuatorSp));

  actuator_setpoint_t setpoint = {
    .roll_sp = rollActuatorSp,
    .pitch_sp = pitchActuatorSp,
    .yaw_sp = yawActuatorSp,
    .throttle_sp = input.throttle_sp
  };

  return setpoint;
}
