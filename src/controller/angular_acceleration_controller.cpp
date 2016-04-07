#include "controller/angular_acceleration_controller.hpp"

#include <algorithm>

#include "global_parameters.hpp"
#include "util/math.hpp"

AngularAccelerationController::AngularAccelerationController(ParameterRepository& params)
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
  params.def(PARAM_MAX_PITCH_ROLL_ACC, 0.0);
}

ActuatorSetpoint AngularAccelerationController::run(const WorldEstimate& world, const AngularAccelerationSetpoint& input) {
  // Update parameters
  rollAccPid.setGains(params.get(PARAM_PID_ROLL_KP), params.get(PARAM_PID_ROLL_KI), params.get(PARAM_PID_ROLL_KD));
  pitchAccPid.setGains(params.get(PARAM_PID_PITCH_KP), params.get(PARAM_PID_PITCH_KI), params.get(PARAM_PID_PITCH_KD));
  yawAccPid.setGains(params.get(PARAM_PID_YAW_KP), params.get(PARAM_PID_YAW_KI), params.get(PARAM_PID_YAW_KD));

  // Limit to maximum angular accelerations
  float rollAccSp = clip(input.rollAcc, -params.get(PARAM_MAX_PITCH_ROLL_ACC), params.get(PARAM_MAX_PITCH_ROLL_ACC));
  float pitchAccSp = clip(input.pitchAcc, params.get(PARAM_MAX_PITCH_ROLL_ACC), params.get(PARAM_MAX_PITCH_ROLL_ACC));

  // Run PID controllers
  float rollActuatorSp = rollAccPid.calculate(rollAccSp, world.att.rollAcc, params.get(GlobalParameters::PARAM_DT));
  float pitchActuatorSp = pitchAccPid.calculate(pitchAccSp, world.att.pitchAcc, params.get(GlobalParameters::PARAM_DT));
  float yawActuatorSp = yawAccPid.calculate(input.yawAcc, world.att.yawAcc, params.get(GlobalParameters::PARAM_DT));

  // Output
  ActuatorSetpoint setpoint {
    .roll = rollActuatorSp,
    .pitch = pitchActuatorSp,
    .yaw = yawActuatorSp,
    .throttle = input.throttle
  };

  return setpoint;
}

