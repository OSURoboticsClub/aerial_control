#include "controller/angular_velocity_controller.hpp"

#include <algorithm>

#include "global_parameters.hpp"
#include "util/math.hpp"

AngularVelocityController::AngularVelocityController(ParameterRepository& params)
  : params(params) {
  params.def(PARAM_PID_ROLL_KP, 0.0);
  params.def(PARAM_PID_ROLL_KI, 0.0);
  params.def(PARAM_PID_ROLL_KD, 0.0);
  params.def(PARAM_PID_PITCH_KP, 0.0);
  params.def(PARAM_PID_PITCH_KI, 0.0);
  params.def(PARAM_PID_PITCH_KD, 0.0);
  params.def(PARAM_PID_YAW_KP, 0.0);
  params.def(PARAM_PID_YAW_KI, 0.0);
  params.def(PARAM_PID_YAW_KD, 0.0);
  params.def(PARAM_MAX_PITCH_ROLL_VEL, 0.0);
}

AngularAccelerationSetpoint AngularVelocityController::run(const WorldEstimate& world, const AngularVelocitySetpoint& input) {
  // Update parameters
  rollVelPid.setGains(params.get(PARAM_PID_ROLL_KP), params.get(PARAM_PID_ROLL_KI), params.get(PARAM_PID_ROLL_KD));
  pitchVelPid.setGains(params.get(PARAM_PID_PITCH_KP), params.get(PARAM_PID_PITCH_KI), params.get(PARAM_PID_PITCH_KD));
  yawVelPid.setGains(params.get(PARAM_PID_YAW_KP), params.get(PARAM_PID_YAW_KI), params.get(PARAM_PID_YAW_KD));

  // Limit to maximum angular velocities
  float rollVelSp = clip(input.rollVel, -params.get(PARAM_MAX_PITCH_ROLL_VEL), params.get(PARAM_MAX_PITCH_ROLL_VEL));
  float pitchVelSp = clip(input.pitchVel, params.get(PARAM_MAX_PITCH_ROLL_VEL), params.get(PARAM_MAX_PITCH_ROLL_VEL));

  // Run PID controllers
  float rollAccSp = rollVelPid.calculate(rollVelSp, world.att.rollVel, params.get(GlobalParameters::PARAM_DT));
  float pitchAccSp = pitchVelPid.calculate(pitchVelSp, world.att.pitchVel, params.get(GlobalParameters::PARAM_DT));
  float yawAccSp = yawVelPid.calculate(input.yawVel, world.att.yawVel, params.get(GlobalParameters::PARAM_DT));

  // Output
  AngularAccelerationSetpoint setpoint {
    .rollAcc = rollAccSp,
    .pitchAcc = pitchAccSp,
    .yawAcc = yawAccSp,
    .throttle = input.throttle
  };

  return setpoint;
}
